// Version: v2.20
//! # Firmware Principal — Rover Olympus / Arduino Mega 2560
//!
//! ## Loop principal (20 ms / ciclo):
//!   1. `msm.tick()`            — watchdog: sin PING en 100 ciclos (~2 s) → FAULT
//!   2. HC-SR04 + VL53L0X (cada 5 ciclos) — emergencia < 20/15 cm → FAULT
//!   3. Stall detection         — encoders via ISR → msm.update_safety(mask)
//!   4. `iface.poll_command()`  — trama ASCII desde USART0 (USB/RPi5)
//!   5. `msm.process(cmd)`      — transición de estado + calcula DriveOutput
//!   6. `sync_drive!()`         — aplica DriveOutput a los 6 motores
//!   7. `iface.send_response()` — respuesta ASCII
//!   8. Sensores analógicos (cada 25 ciclos ~500 ms) — corriente + temperatura
//!   9. Telemetría (~1 s)       — TLM:<SAFETY>:<MASK>
//!
//! ## Asignación de pines:
//!   - USART0 (USB) @ 115200 → RPi5 / PC
//!   - Timer2 D9(FR/RPWM) D10(FL/RPWM)
//!   - Timer3 D5(CR/RPWM) | Timer4 D6(CL/RPWM) D7(RR/RPWM) D8(RL/RPWM)
//!   - [mixed-drivers/all-bts7960] Timer1 D12(CR/LPWM) D11(CL/LPWM) D13(RR/LPWM)  (CR/CL LPWM+EN intercambiados, bringup 2026-05-31)
//!                                 Timer0 D4(RL/LPWM)
//!   - [all-bts7960]               Timer5 D44(FR/LPWM/OC5C) D45(FL/LPWM/OC5B)
//!   - EN/Dirección: D22–D25 (FR/FL L298N IN1/IN2  ó  BTS R_EN/L_EN bajo all-bts7960)
//!                   D28–D31, D34–D37 (CR–RL EN)
//!   - HC-SR04: D38(Trigger) D39(Echo)
//!   - VL53L0X: D42(SDA/PL7) D43(SCL/PL6) - soft I2C, avoids TWI conflict
//!   - Encoders fase A: D21(INT0/FR) D20(INT1/FL) D19(INT2/CR) D18(INT3/CL)
//!                      D2(INT4/RR) D3(INT5/RL)
//!   - Encoders fase B (default/mixed): D44(PL5/FR) D45(PL4/FL) D46(PL3/CR)
//!                                      D47(PL2/CL) D48(PL1/RR) D49(PL0/RL) — Port L pull-up
//!   - Encoders fase B (all-bts7960):   A13(PK5/FR) A14(PK6/FL) — Port K pull-up
//!                                      D46–D49 idénticos al modo mixed para CR/CL/RR/RL
//!   - ACS712-30A: A0(FR) A1(FL) A2(CR) A3(CL) A4(RR) A5(RL)
//!   - LM335:      A6
//!
//! ## Diseño de encoders:
//!   Los 6 QuadratureEncoders son `static` para ser accesibles desde las ISRs.
//!   Motor NFP-5840-31ZY-EN: encoder diferencial 6 cables (A, A', B, B', VCC, GND).
//!   Se usan A y B; A' y B' (señales complementarias) quedan sin conectar.
//!   Cada ISR se dispara en CUALQUIER flanco de A (EICRA=0x55, EICRB=0x05),
//!   lee el estado instantáneo de A (PINx) y B (PINL ó PINK bajo all-bts7960
//!   para FR/FL) y llama on_edge(a, b).
//!   Decodificación x2: forward = a_high XOR b_high.
//!   El loop principal lee los contadores (signed: + adelante, - atrás) y
//!   detecta stall si no cambian durante STALL_THRESHOLD ciclos mientras la
//!   velocidad supera STALL_SPEED_MIN.
//!   Ver consideration_implementation.md §6 para la decisión de diseño.

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use panic_halt as _;
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer2Pwm, Timer3Pwm, Timer4Pwm};
#[cfg(any(feature = "mixed-drivers", feature = "all-bts7960"))]
use arduino_hal::simple_pwm::{Timer0Pwm, Timer1Pwm};
#[cfg(feature = "all-bts7960")]
use arduino_hal::simple_pwm::Timer5Pwm;
use rover_low_level_controller::command_interface::{CommandInterface, RxRingBuffer};
#[cfg(any(not(feature = "all-bts7960"), feature = "rl-l298n"))]
use rover_low_level_controller::motor_control::l298n::L298NMotor;
#[cfg(any(feature = "mixed-drivers", feature = "all-bts7960"))]
use rover_low_level_controller::motor_control::bts7960::BTS7960Motor;
use rover_low_level_controller::motor_control::SixWheelRover;
use rover_low_level_controller::config::*;
use rover_low_level_controller::sensors::hc_sr04::HCSR04;
use rover_low_level_controller::sensors::{QuadratureEncoder, Encoder};
use rover_low_level_controller::sensors::{ACS712, LM335, NTCThermistor, VL53L0X, INA3221, TF02, check_temp_c};
#[cfg(not(feature = "no-mpu"))]
use rover_low_level_controller::sensors::mpu9250::MPU9250;
use rover_low_level_controller::ramp::DriveRamp;
use rover_low_level_controller::relay::RelayModule;
use arduino_hal::prelude::*;
use rover_low_level_controller::state_machine::{
    format_response, parse_command, BankMode, Command, MasterStateMachine, Response, RoverState,
    SafetyState, SensorFrame,
};

// ─── Encoders estáticos (accesibles desde ISRs) ───────────────────────────────
//
// Orden del stall_mask (bit N = motor N):
//   bit 0 = Front Right  (INT0 / D21 fase A, B=D44/PL5 default·mixed | A13/PK5 all-bts7960)
//   bit 1 = Front Left   (INT1 / D20 fase A, B=D45/PL4 default·mixed | A14/PK6 all-bts7960)
//   bit 2 = Center Right (INT2 / D19 fase A, D46/PL3 fase B)
//   bit 3 = Center Left  (INT3 / D18 fase A, D47/PL2 fase B)
//   bit 4 = Rear Right   (INT4 / D2  fase A, D48/PL1 fase B)
//   bit 5 = Rear Left    (INT5 / D3  fase A, D49/PL0 fase B)

static ENCODER_FR: QuadratureEncoder = QuadratureEncoder::new(); // FR — INT0/D21, B=D44/PL5 ó A13/PK5 (all-bts)
static ENCODER_FL: QuadratureEncoder = QuadratureEncoder::new(); // FL — INT1/D20, B=D45/PL4 ó A14/PK6 (all-bts)
static ENCODER_CR: QuadratureEncoder = QuadratureEncoder::new(); // CR — INT2/D19, B=D46/PL3
static ENCODER_CL: QuadratureEncoder = QuadratureEncoder::new(); // CL — INT3/D18, B=D47/PL2
static ENCODER_RR: QuadratureEncoder = QuadratureEncoder::new(); // RR — INT4/D2,  B=D48/PL1
static ENCODER_RL: QuadratureEncoder = QuadratureEncoder::new(); // RL — INT5/D3,  B=D49/PL0

// ─── Registros AVR para las ISRs de encoders ─────────────────────────────────
//
// Fase A: INT0–INT3 en Port D (PD0–PD3), INT4–INT5 en Port E (PE4–PE5).
// Fase B: todos en Port L (PL5–PL0), un solo read de PINL cubre los 6 encoders.
// Direcciones de data memory (= I/O address + 0x20 para puertos clásicos).
const PIND_ADDR: *const u8 = 0x29 as *const u8; // Port D input: D18=PD3, D19=PD2, D20=PD1, D21=PD0
const PINE_ADDR: *const u8 = 0x2C as *const u8; // Port E input: D2=PE4, D3=PE5
const PINL_ADDR: *const u8 = 0x109 as *const u8; // Port L input: D44=PL5…D49=PL0
#[cfg(feature = "all-bts7960")]
const PINK_ADDR: *const u8 = 0x106 as *const u8; // Port K input: A13=PK5, A14=PK6

// ─── ISRs — any edge en Fase A (EICRA=0x55, EICRB=0x05) ─────────────────────

#[avr_device::interrupt(atmega2560)]
fn INT0() {
    let pind = unsafe { core::ptr::read_volatile(PIND_ADDR) };
    // Fase B FR: D44/PL5 en default/mixed; A13/PK5 bajo all-bts7960 (D44 pasa a Timer5C LPWM).
    #[cfg(not(feature = "all-bts7960"))]
    let b = (unsafe { core::ptr::read_volatile(PINL_ADDR) } & (1 << 5)) != 0;
    #[cfg(feature = "all-bts7960")]
    let b = (unsafe { core::ptr::read_volatile(PINK_ADDR) } & (1 << 5)) != 0;
    ENCODER_FR.on_edge((pind & (1 << 0)) != 0, b);
}

#[avr_device::interrupt(atmega2560)]
fn INT1() {
    let pind = unsafe { core::ptr::read_volatile(PIND_ADDR) };
    // Fase B FL: D45/PL4 en default/mixed; A14/PK6 bajo all-bts7960 (D45 pasa a Timer5B LPWM).
    #[cfg(not(feature = "all-bts7960"))]
    let b = (unsafe { core::ptr::read_volatile(PINL_ADDR) } & (1 << 4)) != 0;
    #[cfg(feature = "all-bts7960")]
    let b = (unsafe { core::ptr::read_volatile(PINK_ADDR) } & (1 << 6)) != 0;
    ENCODER_FL.on_edge((pind & (1 << 1)) != 0, b);
}

#[avr_device::interrupt(atmega2560)]
fn INT2() {
    let pind = unsafe { core::ptr::read_volatile(PIND_ADDR) };
    let pinl = unsafe { core::ptr::read_volatile(PINL_ADDR) };
    ENCODER_CR.on_edge((pind & (1 << 2)) != 0, (pinl & (1 << 3)) != 0);
}

#[avr_device::interrupt(atmega2560)]
fn INT3() {
    let pind = unsafe { core::ptr::read_volatile(PIND_ADDR) };
    let pinl = unsafe { core::ptr::read_volatile(PINL_ADDR) };
    ENCODER_CL.on_edge((pind & (1 << 3)) != 0, (pinl & (1 << 2)) != 0);
}

#[avr_device::interrupt(atmega2560)]
fn INT4() {
    let pine = unsafe { core::ptr::read_volatile(PINE_ADDR) };
    let pinl = unsafe { core::ptr::read_volatile(PINL_ADDR) };
    ENCODER_RR.on_edge((pine & (1 << 4)) != 0, (pinl & (1 << 1)) != 0);
}

#[avr_device::interrupt(atmega2560)]
fn INT5() {
    let pine = unsafe { core::ptr::read_volatile(PINE_ADDR) };
    let pinl = unsafe { core::ptr::read_volatile(PINL_ADDR) };
    ENCODER_RL.on_edge((pine & (1 << 5)) != 0, (pinl & (1 << 0)) != 0);
}

// ─── Ring buffer USART RX (interrupt-driven) ─────────────────────────────────
//
// El FIFO hardware USART del ATmega2560 tiene solo 3 bytes. Con delay_ms(20)
// en el loop, un comando de 5+ bytes llega completo en ~434 µs y los últimos
// bytes se pierden por overflow (DOR). Ver docs/debug_usart_overflow.md.
//
// Solución: la ISR USART_RX copia cada byte recibido en este ring buffer de
// 64 bytes. poll_from_ring() lo drena en cada iteración del loop sin importar
// si el CPU estuvo bloqueado los últimos 20 ms.
//
// MODO TEST (actual): USART0 (USB) — ISR: USART0_RX, registro: UDR0
// MODO PRODUCCIÓN:    USART3 (RPi) — cambiar ISR a USART3_RX y leer UDR3

static RX_BUF:     RxRingBuffer = RxRingBuffer::new();
static TF02_RX_BUF: RxRingBuffer = RxRingBuffer::new();

/// ISR USART0 RX Complete — copia byte recibido al ring buffer.
/// En producción (USART3) renombrar a USART3_RX y leer UDR3.
#[avr_device::interrupt(atmega2560)]
fn USART0_RX() {
    let byte = unsafe {
        (*avr_device::atmega2560::USART0::ptr()).udr0().read().bits()
    };
    unsafe { RX_BUF.push(byte); }
}

/// ISR USART2 RX Complete — almacena cada byte del TF02 antes de que el FIFO se llene.
/// El TF02 envía 9 bytes en ~781 µs (115200 baud); sin ISR los bytes 3-9 se pierden.
#[avr_device::interrupt(atmega2560)]
fn USART2_RX() {
    let byte = unsafe {
        (*avr_device::atmega2560::USART2::ptr()).udr2().read().bits()
    };
    unsafe { TF02_RX_BUF.push(byte); }
}

// ─── Macro auxiliar ──────────────────────────────────────────────────────────

/// Promedia N lecturas de un pin analógico para reducir ruido ADC.
/// El ATmega2560 tiene un solo ADC multiplexado; cada lectura es ~104 µs.
macro_rules! adc_avg {
    ($pin:expr, $adc:expr, $n:expr) => {{
        let mut sum = 0u32;
        for _ in 0..$n {
            sum += $pin.analog_read(&mut $adc) as u32;
        }
        (sum / $n as u32) as u16
    }};
}

/// Aplica msm.drive al rover con dos modos de parada.
///
/// ## Modos
///
/// `hard` — Stop inmediato: resetea la rampa y llama a `rover.stop()`.
///   Usar en: obstáculo HC-SR04/VL53L0X, sobrecorriente OC_FAULT, stall de encoder.
///   El motor llega a 0 en el mismo tick — máximo pico de back-EMF, pero
///   obligatorio cuando hay peligro físico inmediato.
///
/// `soft` — Stop/start por rampa: interpola hacia el target del MSM a
///   RAMP_STEP_SOFT (10 %) por ciclo de 20 ms.
///   Usar en: comando GCS (STB/FLT), watchdog de comms, cap de safety Limit.
///   El motor llega a 0 en ≤ 200 ms — cumple RF-005 (≤ 500 ms).
///
/// En estado Limit, ambos modos recortan el target a LIMIT_SPEED_CAP antes
/// de pasarlo a la rampa (soft) o aplicarlo directo (hard ya para todo).
macro_rules! sync_drive {
    // ── Hard stop ──────────────────────────────────────────────────────────
    // brake_all() usa freno activo en BTS7960 y cae a stop() en L298N.
    (hard, $rover:expr, $msm:expr, $ramp:expr) => {
        $ramp.hard_stop();
        $rover.brake_all();
    };
    // ── Soft stop / start ──────────────────────────────────────────────────
    (soft, $rover:expr, $msm:expr, $ramp:expr) => {{
        let target_l = match $msm.state {
            RoverState::Fault | RoverState::Safe | RoverState::Standby => 0,
            _ => if $msm.safety == SafetyState::Limit {
                $msm.drive.left.clamp(-LIMIT_SPEED_CAP, LIMIT_SPEED_CAP)
            } else {
                $msm.drive.left
            },
        };
        let target_r = match $msm.state {
            RoverState::Fault | RoverState::Safe | RoverState::Standby => 0,
            _ => if $msm.safety == SafetyState::Limit {
                $msm.drive.right.clamp(-LIMIT_SPEED_CAP, LIMIT_SPEED_CAP)
            } else {
                $msm.drive.right
            },
        };
        let (l, r) = $ramp.step(target_l, target_r, RAMP_STEP_SOFT);
        if l == 0 && r == 0 {
            $rover.stop();
        } else {
            $rover.set_speeds(l, r);
        }
    }};
}

// ─── Causa de reset (MCUSR) ──────────────────────────────────────────────────
//
// El ATmega2560 expone en MCUSR los flags de la causa del último reset:
//   PORF  (bit 0) — Power-on reset
//   EXTRF (bit 1) — Reset externo (pin RESET o botón)
//   BORF  (bit 2) — Brownout (bajada de Vcc bajo el umbral del BOD)
//   WDRF  (bit 3) — Watchdog hardware disparó
//
// IMPORTANTE: hay que leer y limpiar MCUSR antes de cualquier otra init.
// Si se inicializa el WDT antes de limpiar MCUSR, el flag WDRF puede
// quedar activo y provocar un reset loop.
//
// Referencia: ATmega2560 Datasheet §11.4 (MCU Status Register).

/// Lee y limpia MCUSR. Retorna una cadena ASCII estática con la causa.
/// Si varios flags están activos (posible tras brownout + WDOG) reporta
/// el de mayor severidad: WDOG > BROWN > EXT > POWERON.
fn read_reset_cause() -> &'static str {
    let mcusr = unsafe {
        let val = (*avr_device::atmega2560::CPU::ptr()).mcusr().read().bits();
        (*avr_device::atmega2560::CPU::ptr()).mcusr().write(|w| w.bits(0));
        val
    };
    if mcusr & (1 << 3) != 0 { "RESET:WDOG"    }
    else if mcusr & (1 << 2) != 0 { "RESET:BROWN"  }
    else if mcusr & (1 << 1) != 0 { "RESET:EXT"    }
    else                           { "RESET:POWERON" }
}

// ─── Entry point ─────────────────────────────────────────────────────────────

#[arduino_hal::entry]
fn main() -> ! {
    let dp   = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // ── USART0: USB — modo test desde PC ─────────────────────────────────────
    // En producción usar USART3 (D14/D15): ver comentario en bloque ISR arriba.
    let serial_rpi = arduino_hal::default_serial!(dp, pins, 115200);
    let mut iface = CommandInterface::new(serial_rpi);

    // Habilitar interrupción USART0 RX Complete (RXCIE0 = bit 7 de UCSR0B).
    // La ISR USART0_RX copiará cada byte al RX_BUF antes de que el FIFO
    // hardware (3 bytes) se desborde durante delay_ms(20).
    // En producción (USART3): modificar UCSR3B con bit RXCIE3.
    //
    // IMPORTANTE: vaciar el FIFO hardware antes de habilitar la ISR.
    // El bootloader puede dejar bytes residuales en el FIFO que, de no
    // descartarse, la ISR los leería al ejecutar SEI y los almacenaría
    // en RX_BUF como basura, potencialmente formando comandos inválidos
    // (p.ej. "FLT") que pondrían la MSM en FAULT en el arranque.
    unsafe {
        let p = &(*avr_device::atmega2560::USART0::ptr());
        while p.ucsr0a().read().rxc0().bit_is_set() {
            let _ = p.udr0().read().bits(); // descartar byte residual
        }
        p.ucsr0b().modify(|_, w| w.rxcie0().set_bit());
    }

    // ── Timers PWM ───────────────────────────────────────────────────────────
    let mut timer2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    let mut timer3 = Timer3Pwm::new(dp.TC3, Prescaler::Prescale64);
    let mut timer4 = Timer4Pwm::new(dp.TC4, Prescaler::Prescale64);

    // ── 6 Motores ────────────────────────────────────────────────────────────
    // FR y FL: L298N por defecto/mixed; BTS7960 IBT-2 bajo feature "all-bts7960".
    //   Bajo all-bts7960:
    //     FR: RPWM=D9(OC2B/Timer2)  LPWM=D44(OC5C/Timer5) R_EN=D23 L_EN=D25
    //     FL: RPWM=D10(OC2A/Timer2) LPWM=D45(OC5B/Timer5) R_EN=D22 L_EN=D24
    //
    // CR/CL/RR/RL: L298N por defecto; BTS7960 IBT-2 bajo "mixed-drivers" o "all-bts7960".
    //   CR: RPWM=D5(OC3A/Timer3)  LPWM=D11(OC1A/Timer1) R_EN=D28 L_EN=D29
    //   CL: RPWM=D6(OC4A/Timer4)  LPWM=D12(OC1B/Timer1) R_EN=D30 L_EN=D31
    //   RR: RPWM=D7(OC4B/Timer4)  LPWM=D13(OC1C/Timer1) R_EN=D34 L_EN=D35
    //   RL: RPWM=D8(OC4C/Timer4)  LPWM=D4 (OC0B/Timer0) R_EN=D36 L_EN=D37
    #[cfg(any(feature = "mixed-drivers", feature = "all-bts7960"))]
    let mut timer1 = Timer1Pwm::new(dp.TC1, Prescaler::Prescale64);
    #[cfg(any(feature = "mixed-drivers", feature = "all-bts7960"))]
    let mut timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);
    #[cfg(feature = "all-bts7960")]
    let mut timer5 = Timer5Pwm::new(dp.TC5, Prescaler::Prescale64);

    // ── FR / FL ───────────────────────────────────────────────────────────────
    #[cfg(not(feature = "all-bts7960"))]
    let fr = L298NMotor::new(pins.d9.into_output().into_pwm(&mut timer2),  pins.d23.into_output(), pins.d25.into_output(), false);
    #[cfg(not(feature = "all-bts7960"))]
    let fl = L298NMotor::new(pins.d10.into_output().into_pwm(&mut timer2), pins.d22.into_output(), pins.d24.into_output(), false);

    // FR NO invertido: el bringup 2026-05-31 reportó inversión por error; en la práctica
    // FR gira igual que CR/RR (adelante con velocidad positiva, atrás con negativa).
    #[cfg(feature = "all-bts7960")]
    let fr = BTS7960Motor::new(pins.d9.into_output().into_pwm(&mut timer2),  pins.d44.into_output().into_pwm(&mut timer5), pins.d23.into_output(), pins.d25.into_output(), false);
    // FL invertido: M+/M- físicamente al revés (validado 2026-05-27, confirmado bringup 2026-05-31).
    #[cfg(feature = "all-bts7960")]
    let fl = BTS7960Motor::new(pins.d10.into_output().into_pwm(&mut timer2), pins.d45.into_output().into_pwm(&mut timer5), pins.d22.into_output(), pins.d24.into_output(), true);

    // ── CR / CL / RR / RL ─────────────────────────────────────────────────────
    #[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960")))]
    let cr = L298NMotor::new(pins.d5.into_output().into_pwm(&mut timer3), pins.d28.into_output(), pins.d29.into_output(), false);
    #[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960")))]
    let cl = L298NMotor::new(pins.d6.into_output().into_pwm(&mut timer4), pins.d30.into_output(), pins.d31.into_output(), false);
    #[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960")))]
    let rr = L298NMotor::new(pins.d7.into_output().into_pwm(&mut timer4), pins.d34.into_output(), pins.d35.into_output(), false);
    #[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960")))]
    let rl = L298NMotor::new(pins.d8.into_output().into_pwm(&mut timer4), pins.d36.into_output(), pins.d37.into_output(), false);

    // CR/CL: cableado REAL con LPWM y enables INTERCAMBIADOS respecto al diseño
    // original (descubierto en bringup 2026-05-31; RPWM D5/D6 sin cambio).
    //   CR real: RPWM=D5, LPWM=D12 (OC1B), R_EN=D30, L_EN=D31.
    //   CL real: RPWM=D6, LPWM=D11 (OC1A), R_EN=D28, L_EN=D29.
    #[cfg(any(feature = "mixed-drivers", feature = "all-bts7960"))]
    let cr = BTS7960Motor::new(pins.d5.into_output().into_pwm(&mut timer3), pins.d12.into_output().into_pwm(&mut timer1), pins.d30.into_output(), pins.d31.into_output(), false);
    // CL invertido (validado 2026-05-27, confirmado bringup 2026-05-31).
    #[cfg(any(feature = "mixed-drivers", feature = "all-bts7960"))]
    let cl = BTS7960Motor::new(pins.d6.into_output().into_pwm(&mut timer4), pins.d11.into_output().into_pwm(&mut timer1), pins.d28.into_output(), pins.d29.into_output(), true);
    #[cfg(any(feature = "mixed-drivers", feature = "all-bts7960"))]
    let rr = BTS7960Motor::new(pins.d7.into_output().into_pwm(&mut timer4), pins.d13.into_output().into_pwm(&mut timer1), pins.d34.into_output(), pins.d35.into_output(), false);
    // RL: BTS7960 por defecto en mixed/all-bts; L298N bajo all-bts7960+rl-l298n.
    // El BTS7960 RL resultó SANO en el bringup 2026-05-31 (no estaba quemado),
    // así que se vuelve a usar como BTS. Pines: RPWM=D8(T4), LPWM=D4(T0),
    // R_EN=D36, L_EN=D37. inverted=true (gira adelante con el cableado actual).
    #[cfg(all(any(feature = "mixed-drivers", feature = "all-bts7960"), not(all(feature = "all-bts7960", feature = "rl-l298n"))))]
    let rl = BTS7960Motor::new(pins.d8.into_output().into_pwm(&mut timer4), pins.d4.into_output().into_pwm(&mut timer0),  pins.d36.into_output(), pins.d37.into_output(), true);
    #[cfg(all(feature = "all-bts7960", feature = "rl-l298n"))]
    let rl = L298NMotor::new(pins.d8.into_output().into_pwm(&mut timer4), pins.d36.into_output(), pins.d37.into_output(), false);

    let mut rover = SixWheelRover::new(fr, fl, cr, cl, rr, rl);
    // enable_all() se difiere hasta después de calibrar los ACS712 (más abajo):
    // los drivers deben estar deshabilitados durante el muestreo del offset de cero.

    // ── Encoders fase B — pull-up interno (20–50 kΩ a 5V) ────────────────────
    // NFP-5840-31ZY-EN: encoder diferencial 6 cables. Se usan A y B;
    // A' y B' (señales complementarias) quedan sin conectar al Mega.
    //
    // FR/FL fase B:
    //   - default/mixed: D44(PL5) y D45(PL4) — Port L.
    //   - all-bts7960:   A13(PK5) y A14(PK6) — Port K (D44/D45 ocupados como Timer5 LPWM).
    // CR/CL/RR/RL fase B: D46..D49 (Port L) en todas las configuraciones.
    #[cfg(not(feature = "all-bts7960"))]
    let _enc_fr_b = pins.d44.into_pull_up_input(); // PL5 — FR fase B
    #[cfg(not(feature = "all-bts7960"))]
    let _enc_fl_b = pins.d45.into_pull_up_input(); // PL4 — FL fase B
    #[cfg(feature = "all-bts7960")]
    let _enc_fr_b = pins.a13.into_pull_up_input(); // PK5 — FR fase B (movido desde D44)
    #[cfg(feature = "all-bts7960")]
    let _enc_fl_b = pins.a14.into_pull_up_input(); // PK6 — FL fase B (movido desde D45)
    let _enc_cr_b = pins.d46.into_pull_up_input(); // PL3 — CR fase B
    let _enc_cl_b = pins.d47.into_pull_up_input(); // PL2 — CL fase B
    let _enc_rr_b = pins.d48.into_pull_up_input(); // PL1 — RR fase B
    let _enc_rl_b = pins.d49.into_pull_up_input(); // PL0 — RL fase B

    // ── Módulo relay 2 canales — D40(IN1/Bank2), D41(IN2/Bank3) ─────────────
    // Active LOW: LOW = relay energizado = banco habilitado.
    // Arranca con Bank 2 activo, Bank 3 en espera.
    // En FAULT/SAFE → emergency_off() pone ambos HIGH → motores sin potencia.
    let mut relay = RelayModule::new(
        pins.d40.into_output(),
        pins.d41.into_output(),
    );

    // ── HC-SR04 — D38(Trigger), D39(Echo) ───────────────────────────────────
    // with_timeout limita el bloqueo a ~1.75 ms (rango útil ~300 mm).
    // Solo usamos el sensor para detección de emergencia (< HC_EMERGENCY_MM),
    // no para navegación → no necesitamos el rango completo de 4 m.
    let mut hcsr04 = HCSR04::new(
        pins.d38.into_output(),
        pins.d39.into_floating_input().forget_imode(),
    ).with_timeout(HC_ECHO_TIMEOUT_US);

    // ── VL53L0X — D42(SDA/PL7), D43(SCL/PL6) vía soft I2C ──────────────────
    // Módulo GY-VL53L0XV2: incluye regulador 2.8V y level-shifter integrado.
    // VIN acepta 2.6–5.5V; I2C queda al nivel de VIN. No requiere pull-ups externos.
    // init() puede fallar si el sensor no responde; el sistema opera solo con HC-SR04.
    // Desactivable con `no-tof`: el rover usa el TF02 LiDAR para distancia, no este.
    #[cfg(not(feature = "no-tof"))]
    let mut tof = VL53L0X::new();
    #[cfg(not(feature = "no-tof"))]
    if tof.init() {
        tof.start_continuous();
        iface.log("INFO:TOF_OK");
    } else {
        iface.log("WARN:TOF_FAIL");
    }

    // ── INA3221 — D42(SDA/PL7), D43(SCL/PL6), dirección 0x40 ───────────────
    // Triple-canal en modo bus-only para los 3 bancos de batería. IN+n/IN-n
    // de cada canal van AMBOS al (+) del banco n → el chip NO ve la corriente
    // de los motores (shunt on-board en 0 V, sin disipación). Solo voltaje.
    // Comparte el bus soft I2C con VL53L0X (0x29) y MPU-6050 (0x68).
    // WARN si falla: batt_mv queda en 0 y la protección LLC-level de batería
    // baja (< 12 V) no opera; el HLC debe asumir el rol vía TLM.
    let mut ina = INA3221::new();
    if ina.init() {
        iface.log("INFO:INA3221_OK");
    } else {
        iface.log("WARN:INA3221_FAIL");
    }
    // I2C bus scan — diagnóstico (eliminar después de validar hardware)
    {
        use rover_low_level_controller::sensors::soft_i2c::SoftI2C;
        let scanner = SoftI2C::new();
        let found = scanner.scan();
        let mut any = false;
        for addr in 0x08u8..0x78 {
            if found[(addr / 8) as usize] & (1 << (addr % 8)) != 0 {
                let mut msg = [0u8; 16];
                let mut i = 0;
                for &b in b"I2C:0x" { msg[i] = b; i += 1; }
                let hi = addr >> 4; let lo = addr & 0xF;
                msg[i] = if hi < 10 { b'0'+hi } else { b'a'+hi-10 }; i += 1;
                msg[i] = if lo < 10 { b'0'+lo } else { b'a'+lo-10 }; i += 1;
                iface.log(core::str::from_utf8(&msg[..i]).unwrap_or("I2C:?"));
                any = true;
            }
        }
        if !any { iface.log("I2C:NONE"); }
    }
    // MPU-9250 — desactivable con `no-mpu` (firmware de operación flash-allbts).
    // La estimación de pose vive en el HLC: el LLC solo lee y transmite cuentas
    // crudas en la trama RAW. No hay EKF local.
    #[cfg(not(feature = "no-mpu"))]
    let mut mpu = MPU9250::new();
    #[cfg(not(feature = "no-mpu"))]
    {
        let who = mpu.init();
        if who != 0 {
            // INFO:MPU9250_OK:addr=0x68:id=0x71 (dirección e identificador por separado)
            let mut msg = [0u8; 48];
            let mut i = 0;
            let label: &[u8] = if mpu.is_expected_part() { b"INFO:MPU9250_OK:addr=" } else { b"WARN:IMU_UNEXPECTED_ID:addr=" };
            for &b in label { msg[i] = b; i += 1; }
            write_hex_u8(mpu.addr(), &mut msg, &mut i);
            for &b in b":id=" { msg[i] = b; i += 1; }
            write_hex_u8(who, &mut msg, &mut i);
            iface.log(core::str::from_utf8(&msg[..i]).unwrap_or("INFO:MPU9250_OK"));

            // Autocomprobación del giroscopio al arrancar. SOLO diagnóstico: el
            // promedio se registra y NO se resta de lo que se transmite. El sesgo
            // lo estima el HLC como estado del filtro (pseudo-medida en reposo).
            let mut sum = [0i32; 3];
            let mut n: i32 = 0;
            for _ in 0..50 {
                if let Some(r) = mpu.read_raw() {
                    sum[0] += r.4 as i32; sum[1] += r.5 as i32; sum[2] += r.6 as i32;
                    n += 1;
                }
                arduino_hal::delay_ms(10);
            }
            let mut msg = [0u8; 64];
            let mut i = 0;
            for &b in b"INFO:IMU_SELFTEST:n=" { msg[i] = b; i += 1; }
            write_i32(n, &mut msg, &mut i);
            if n > 0 {
                for (k, tag) in [b":gx=", b":gy=", b":gz="].iter().enumerate() {
                    for &b in tag.iter() { msg[i] = b; i += 1; }
                    write_i32(sum[k] / n, &mut msg, &mut i);
                }
            }
            iface.log(core::str::from_utf8(&msg[..i]).unwrap_or("INFO:IMU_SELFTEST"));
        } else {
            iface.log("WARN:MPU9250_FAIL");
        }
    }

    // ── USART2: TF02 LiDAR — D17(RX2) @ 115200 baud ─────────────────────────
    // TF02 transmite frames de 9 bytes a 100 Hz sin necesitar init ni comandos.
    // Solo conectamos RX2 (verde/TX del sensor). El blanco/RX del sensor queda libre.
    // Voltaje LVTTL 0-3.3 V: el Mega acepta 3.3 V como HIGH sin level-shifter.
    // USART1 ocupado por INT2/INT3 (encoders CR/CL). USART3 reservado para RPi5.
    // Baud 115200 a 16 MHz con U2X2=1: UBRR2 = 16 (error 2.1 %).
    unsafe {
        let p2 = &(*avr_device::atmega2560::USART2::ptr());
        p2.ubrr2().write(|w| w.bits(16));
        p2.ucsr2a().write(|w| w.u2x2().set_bit());
        p2.ucsr2b().write(|w| w.rxen2().set_bit().rxcie2().set_bit()); // RX + interrupción
        p2.ucsr2c().write(|w| w.ucsz2().chr8().usbs2().stop1()); // 8N1 explícito
        // Vaciar FIFO residual antes de que SEI habilite la ISR
        while p2.ucsr2a().read().rxc2().bit_is_set() { let _ = p2.udr2().read().bits(); }
    }
    let mut tf02 = TF02::new();
    iface.log("INFO:TF02_INIT");

    // ── ADC + sensores analógicos ─────────────────────────────────────────────
    // ACS712 (corriente):         A0=FR A1=FL A2=CR A3=CL A4=RR A5=RL
    // LM335  (temp. ambiente):    A6
    // NTC    (temp. baterías):    A7=B1a A8=B1b A9=B2a A10=B2b A11=B3a A12=B3b
    let mut adc     = arduino_hal::Adc::new(dp.ADC, Default::default());
    let acs_fr_pin  = pins.a0.into_analog_input(&mut adc);
    let acs_fl_pin  = pins.a1.into_analog_input(&mut adc);
    let acs_cr_pin  = pins.a2.into_analog_input(&mut adc);
    let acs_cl_pin  = pins.a3.into_analog_input(&mut adc);
    let acs_rr_pin  = pins.a4.into_analog_input(&mut adc);
    let acs_rl_pin  = pins.a5.into_analog_input(&mut adc);
    #[cfg(not(feature = "no-lm335"))]
    let lm335_pin   = pins.a6.into_analog_input(&mut adc);
    let ntc_b1a_pin = pins.a7.into_analog_input(&mut adc);   // Banco 1 — sensor A
    let ntc_b1b_pin = pins.a8.into_analog_input(&mut adc);   // Banco 1 — sensor B
    let ntc_b2a_pin = pins.a9.into_analog_input(&mut adc);   // Banco 2 — sensor A
    let ntc_b2b_pin = pins.a10.into_analog_input(&mut adc);  // Banco 2 — sensor B
    let ntc_b3a_pin = pins.a11.into_analog_input(&mut adc);  // Banco 3 — sensor A
    let ntc_b3b_pin = pins.a12.into_analog_input(&mut adc);  // Banco 3 — sensor B

    // ── Calibración automática del cero de los ACS712 ───────────────────────
    // Muestrea cada canal con los drivers aún deshabilitados (corriente ~0), por
    // lo que la lectura representa el offset físico del sensor (variación de
    // fábrica + errores de cableado VCC/GND). Sin esto, los chips con offset
    // grande (>500 mA) disparan OC_FAULT al arranque y dejan la MSM en Fault,
    // rechazando todo EXP del HLC con ERR:ESTOP.
    const ACS_CAL_SAMPLES: u8 = 32;
    let mut acs_zero_mv = [2500i32; 6]; // fallback al ideal VCC/2 si el muestreo falla
    {
        let mut cal_sum = [0u32; 6];
        for _ in 0..ACS_CAL_SAMPLES {
            cal_sum[0] += acs_fr_pin.analog_read(&mut adc) as u32;
            cal_sum[1] += acs_fl_pin.analog_read(&mut adc) as u32;
            cal_sum[2] += acs_cr_pin.analog_read(&mut adc) as u32;
            cal_sum[3] += acs_cl_pin.analog_read(&mut adc) as u32;
            cal_sum[4] += acs_rr_pin.analog_read(&mut adc) as u32;
            cal_sum[5] += acs_rl_pin.analog_read(&mut adc) as u32;
            arduino_hal::delay_ms(5);
        }
        for i in 0..6 {
            acs_zero_mv[i] = ((cal_sum[i] / ACS_CAL_SAMPLES as u32) * 5000 / 1023) as i32;
        }
    }

    // Instancias ACS712 por motor [FR, FL, CR, CL, RR, RL].
    // La variante (5A/20A/30A) se elige en compilación según el feature activo;
    // el cero de cada canal se ajusta en runtime con la medición anterior.
    #[cfg(feature = "mixed-drivers")]
    let acs: [ACS712; 6] = [
        ACS712::new_05a().calibrate_zero(acs_zero_mv[0]), ACS712::new_05a().calibrate_zero(acs_zero_mv[1]), // FR, FL → L298N
        ACS712::new_30a().calibrate_zero(acs_zero_mv[2]), ACS712::new_30a().calibrate_zero(acs_zero_mv[3]), // CR, CL → BTS7960
        ACS712::new_30a().calibrate_zero(acs_zero_mv[4]), ACS712::new_30a().calibrate_zero(acs_zero_mv[5]), // RR, RL → BTS7960
    ];
    // all-bts7960: 6× ACS712-20A físicos (auditoría 2026-05-27). El 20A da mejor
    // SNR que el 30A sin saturar (stall NFP-5840 ~6.5 A ≪ 20 A).
    #[cfg(feature = "all-bts7960")]
    let acs: [ACS712; 6] = [
        ACS712::new_20a().calibrate_zero(acs_zero_mv[0]), ACS712::new_20a().calibrate_zero(acs_zero_mv[1]),
        ACS712::new_20a().calibrate_zero(acs_zero_mv[2]), ACS712::new_20a().calibrate_zero(acs_zero_mv[3]),
        ACS712::new_20a().calibrate_zero(acs_zero_mv[4]), ACS712::new_20a().calibrate_zero(acs_zero_mv[5]),
    ];
    #[cfg(feature = "all-20a")]
    let acs: [ACS712; 6] = [
        ACS712::new_20a().calibrate_zero(acs_zero_mv[0]), ACS712::new_20a().calibrate_zero(acs_zero_mv[1]),
        ACS712::new_20a().calibrate_zero(acs_zero_mv[2]), ACS712::new_20a().calibrate_zero(acs_zero_mv[3]),
        ACS712::new_20a().calibrate_zero(acs_zero_mv[4]), ACS712::new_20a().calibrate_zero(acs_zero_mv[5]),
    ];
    #[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960", feature = "all-20a")))]
    let acs: [ACS712; 6] = [
        ACS712::new_05a().calibrate_zero(acs_zero_mv[0]), ACS712::new_05a().calibrate_zero(acs_zero_mv[1]),
        ACS712::new_05a().calibrate_zero(acs_zero_mv[2]), ACS712::new_05a().calibrate_zero(acs_zero_mv[3]),
        ACS712::new_05a().calibrate_zero(acs_zero_mv[4]), ACS712::new_05a().calibrate_zero(acs_zero_mv[5]),
    ]; // all-l298n (default)

    #[cfg(not(feature = "no-lm335"))]
    let lm335 = LM335::new(); // offset 0 K — ajustar con with_offset si es necesario

    // Instancias NTC por sensor de batería [B1a, B1b, B2a, B2b, B3a, B3b].
    // offset 0 — calibrar con .calibrate(offset_c) tras verificación en hardware.
    let ntc_batt: [NTCThermistor; 6] = [NTCThermistor::new(); 6];

    // ── Interrupciones externas INT0–INT5 (any edge) ─────────────────────────
    // QuadratureEncoder requiere ambos flancos de A para decodificación x2.
    // ISCn1=0, ISCn0=1 → any logical change (flanco de subida Y bajada).
    // EICRA 0x55 = 01010101: any-edge para INT0–INT3
    // EICRB 0x05 = 00000101: any-edge para INT4–INT5
    // EIMSK: habilita INT0–INT5 (bits 0–5)
    dp.EXINT.eicra().write(|w| unsafe { w.bits(0x55) });
    dp.EXINT.eicrb().write(|w| unsafe { w.bits(0x05) });
    dp.EXINT.eimsk().write(|w| unsafe { w.bits(0x3F) });
    unsafe { avr_device::interrupt::enable() };

    // ── Estado del loop ──────────────────────────────────────────────────────
    let mut msm              = MasterStateMachine::new();
    // Rampa de velocidad: interpola entre velocidad actual y objetivo del MSM
    // para evitar picos de back-EMF en frenadas bruscas. Ver src/ramp.rs.
    let mut ramp             = DriveRamp::new();
    let mut resp_buf         = [0u8; RESP_BUF];
    let mut tlm_counter:      u8 = 0;
    let mut hc_counter:       u8 = 0;
    let mut sen_counter:      u8 = 0;
    let mut sen_fast_counter: u8 = 0;
    let mut elapsed_ms:      u32 = 0; // timestamp relativo desde arranque (overflow ~49 días)
    let mut sensor_frame = SensorFrame::ZERO; // última lectura de ACS712 + LM335


    // Estado de stall por encoder (parallel al stall_mask de la MSM)
    let mut last_counts  = [0i32; 6];
    let mut stall_timers = [0u16; 6];
    // Ciclos consecutivos sin frame TF02 válido. Satura en 500 (~10 s); emite WARN una vez.
    let mut tf02_no_data: u16 = 0;
    // Bytes crudos recibidos de USART2 (satura en 65535). Logueado junto al WARN para diagnóstico.
    let mut tf02_raw_rx: u16 = 0;
    // Buffer de los primeros 18 bytes crudos de USART2 (2 frames) para hex dump en WARN.
    let mut tf02_raw_buf  = [0u8; 18];
    let mut tf02_raw_fill: u8 = 0;
    // Loguea SIG del primer frame válido para diagnosticar qué envía el sensor real.
    let mut tf02_sig_logged: bool = false;

    iface.log(read_reset_cause());
    #[cfg(all(feature = "mixed-drivers", not(feature = "all-bts7960")))]
    iface.log("=== ROVER OLYMPUS v2.20 — MSM + HC-SR04 + VL53L0X + TF02 + INA3221 + ENCODERS + ACS712 + LM335 + NTC + RELAY + CLB [mixed-drivers: FR/FL=L298N CR/CL/RR/RL=BTS7960] ===");
    #[cfg(feature = "all-bts7960")]
    iface.log("=== ROVER OLYMPUS v2.20 — MSM + HC-SR04 + VL53L0X + TF02 + INA3221 + ENCODERS + ACS712 + LM335 + NTC + RELAY + CLB [all-bts7960: 6x BTS7960 Timer5C/B en D44/D45] ===");
    #[cfg(not(any(feature = "mixed-drivers", feature = "all-bts7960")))]
    iface.log("=== ROVER OLYMPUS v2.20 — MSM + HC-SR04 + VL53L0X + TF02 + INA3221 + ENCODERS + ACS712 + LM335 + NTC + RELAY + CLB [all-l298n] ===");

    // Reporta el cero calibrado de cada ACS712 (mV). Útil para detectar un chip
    // con offset anómalo (p.ej. RR ≈ 1700 mV vs ~3000 normal → chip dañado).
    {
        let mut zb = [0u8; 64];
        let mut zi = 0;
        for &b in b"INFO:ACS_ZERO_MV" { zb[zi] = b; zi += 1; }
        for mv in &acs_zero_mv {
            zb[zi] = b':'; zi += 1;
            write_i32(*mv, &mut zb, &mut zi);
        }
        zb[zi] = b'\n'; zi += 1;
        iface.send_response(&zb[..zi]);
    }

    // Habilitar drivers ahora que los ACS712 ya calibraron su cero con duty=0.
    rover.enable_all(); // no-op para L298N; activa R_EN/L_EN en BTS7960

    // Estado de energía de los drivers (true tras enable_all() arriba).
    // Los BTS7960 se de-energizan (R_EN/L_EN→LOW, Hi-Z) en estados sin tracción
    // y se re-habilitan al volver a un estado con tracción (comportamiento por
    // defecto, ver sección 0 del loop).
    let mut motors_energized = true;

    // ── Bucle principal ───────────────────────────────────────────────────────
    loop {
        // 0. Gestión de energía del driver según el estado del MSM (DEFAULT).
        //    FIX definitivo "FR gira en FAULT": el debug_all_motors_bts NO gira
        //    en reposo porque termina con disable() (EN bajo, Hi-Z). El MSM, en
        //    cambio, dejaba R_EN/L_EN ALTOS para siempre tras enable_all(): en
        //    FAULT/Standby los IN quedan bajos pero EN alto = freno activo, y el
        //    FR (chip con fuga de high-side gateada por EN) seguía girando.
        //    Aquí de-energizamos el puente-H en Fault/Safe/Standby (corte real,
        //    no freno) y re-habilitamos antes de manejar. Latencia 1 ciclo (20 ms).
        {
            let want_energized = !matches!(
                msm.state,
                RoverState::Fault | RoverState::Safe | RoverState::Standby
            );
            if want_energized && !motors_energized {
                rover.enable_all();
                motors_energized = true;
            } else if !want_energized && motors_energized {
                rover.disable_all();
                motors_energized = false;
            }
        }

        // 1. Watchdog de comunicación
        // EXPERIMENTO de bisección (feature "no-watchdog"): si se desactiva, el
        // rover NUNCA entra en FAULT por pérdida de PING. Si con esto el FR deja
        // de girar solo, queda confirmado que la causa es el path de FAULT (la
        // rampa congelada), no un subsistema (EKF/I2C/encoders/etc.).
        #[cfg(not(feature = "no-watchdog"))]
        if let Some(wdog_resp) = msm.tick() {
            sync_drive!(soft, rover, msm, ramp); // watchdog: no hay peligro físico, rampa suave
            iface.send_response(format_response(wdog_resp, &mut resp_buf));
        }

        // 2. HC-SR04 + VL53L0X — lectura cada HC_READ_PERIOD ciclos (~100 ms)
        //    HC-SR04 es bloqueante (~30 ms max); VL53L0X no bloquea (modo continuo).
        hc_counter = hc_counter.wrapping_add(1);
        if hc_counter >= HC_READ_PERIOD {
            hc_counter = 0;
            #[cfg(not(feature = "no-hcsr04"))]
            {
                if let Ok(mm) = hcsr04.measure_mm() {
                    // Proximidad HC-SR04 → campo dist_mm del TLM (f[19]). Con no-tof
                    // el VL53L0X queda fuera y deja dist_mm en 0; aquí el HC-SR04 es
                    // el único que lo escribe (es la proximidad real que usa el rover).
                    sensor_frame.dist_mm = mm;
                    // FAULT por proximidad: desactivable con `hcsr04-no-fault` (el
                    // HC-SR04 es ruidoso y un pico espurio frenaba los motores). Con
                    // ese flag solo se telemetría dist_mm; la protección de proximidad
                    // la decide el HLC (retreat, gobernado por el nivel de seguridad).
                    #[cfg(not(feature = "hcsr04-no-fault"))]
                    {
                        let hc_thresh = if msm.state == RoverState::Climb { CLB_HC_EMERGENCY_MM } else { HC_EMERGENCY_MM };
                        if mm < hc_thresh {
                            let resp = msm.process(Command::Fault);
                            relay.emergency_off(); // corte hardware: obstáculo físico
                            sync_drive!(hard, rover, msm, ramp);
                            iface.send_response(format_response(resp, &mut resp_buf));
                        }
                    }
                }
            }
            // VL53L0X: lectura no bloqueante — NotReady si el sensor aún no tiene muestra.
            // Desactivado con `no-tof` (el rover no usa este sensor).
            #[cfg(not(feature = "no-tof"))]
            {
                let tof_thresh = if msm.state == RoverState::Climb { CLB_TOF_EMERGENCY_MM } else { TOF_EMERGENCY_MM };
                if tof.ready {
                    if let Ok(mm) = tof.read_mm() {
                        sensor_frame.dist_mm = mm;
                        if mm < tof_thresh {
                            let resp = msm.process(Command::Fault);
                            relay.emergency_off(); // corte hardware: ToF láser
                            sync_drive!(hard, rover, msm, ramp);
                            iface.send_response(format_response(resp, &mut resp_buf));
                        }
                    }
                }
            }
        }

        // 2b. TF02 — drenar bytes disponibles de USART2 (non-blocking)
        //     A 100 Hz × 9 bytes = 900 B/s → ~18 bytes por ciclo de 20 ms.
        //     Solo actualiza dist_far_mm; el HLC decide la evasión de obstáculos.
        let mut tf02_frame_this_cycle = false;
        while let Some(byte) = TF02_RX_BUF.pop() {
            tf02_raw_rx = tf02_raw_rx.saturating_add(1);
            if (tf02_raw_fill as usize) < tf02_raw_buf.len() {
                tf02_raw_buf[tf02_raw_fill as usize] = byte;
                tf02_raw_fill += 1;
            }
            if tf02.feed(byte) {
                sensor_frame.dist_far_mm = tf02.last_dist_mm;
                tf02_frame_this_cycle = true;
                if !tf02_sig_logged {
                    tf02_sig_logged = true;
                    let mut sig_buf = [0u8; 16];
                    let mut si = 0;
                    for &b in b"INFO:TF02_SIG:" { sig_buf[si] = b; si += 1; }
                    write_u32(tf02.last_sig as u32, &mut sig_buf, &mut si);
                    sig_buf[si] = b'\n'; si += 1;
                    iface.send_response(&sig_buf[..si]);
                }
            }
        }
        if tf02_frame_this_cycle {
            tf02_no_data = 0;
        } else if tf02_no_data < 500 {
            tf02_no_data += 1;
            if tf02_no_data == 500 {
                // Loguear bytes crudos: 0 = cable D17 desconectado; >0 = baud/protocolo incorrecto
                let mut warn_buf = [0u8; 32];
                let mut wi = 0;
                for &b in b"WARN:TF02_NO_DATA:RX=" { warn_buf[wi] = b; wi += 1; }
                write_u32(tf02_raw_rx as u32, &mut warn_buf, &mut wi);
                warn_buf[wi] = b'\n'; wi += 1;
                iface.send_response(&warn_buf[..wi]);
                // Hex dump de los primeros bytes capturados: "TF02_RAW:59 59 03 ..."
                if tf02_raw_fill > 0 {
                    // Buffer fijo: "TF02_RAW:" + hasta 18 bytes × "XX " + "\n" = 9 + 54 + 1 = 64
                    let mut raw_buf = [0u8; 64];
                    let mut ri = 0;
                    for &b in b"TF02_RAW:" { raw_buf[ri] = b; ri += 1; }
                    const HEX: &[u8] = b"0123456789ABCDEF";
                    for idx in 0..(tf02_raw_fill as usize) {
                        let byte = tf02_raw_buf[idx];
                        raw_buf[ri] = HEX[(byte >> 4) as usize]; ri += 1;
                        raw_buf[ri] = HEX[(byte & 0xF) as usize]; ri += 1;
                        if idx + 1 < tf02_raw_fill as usize { raw_buf[ri] = b' '; ri += 1; }
                    }
                    raw_buf[ri] = b'\n'; ri += 1;
                    iface.send_response(&raw_buf[..ri]);
                }
            }
        }

        // 3. Stall detection via encoders
        // Lee los 6 contadores y compara con el ciclo anterior.
        {
            // Enclavamiento conjunto: una sola lectura del IMU por ciclo, seguida
            // de inmediato por los seis contadores, para que la trama RAW
            // corresponda a un único instante de muestreo (desfase ~µs en lugar
            // de ~2 ms). Esa simultaneidad es la razón de tener el IMU en el LLC.
            #[cfg(not(feature = "no-mpu"))]
            let imu_sample = mpu.read_raw();
            let counts = [
                ENCODER_FR.get_counts(), ENCODER_FL.get_counts(),
                ENCODER_CR.get_counts(), ENCODER_CL.get_counts(),
                ENCODER_RR.get_counts(), ENCODER_RL.get_counts(),
            ];

            // --- Telemetría RAW para estimación de pose en el HLC (v3.0) ---
            // Formato: RAW:tick:ax:ay:az:gx:gy:gz:encL:encR:temp:imu_ok
            // Los dos campos nuevos van al FINAL para no mover las columnas
            // existentes. La trama sale SIEMPRE: si el IMU no respondió, los
            // campos inerciales van en 0 e imu_ok = 0, de modo que el HLC
            // distingue "sensor caído" de "trama perdida". Cuentas crudas, sin
            // restar sesgo (lo estima el HLC).
            #[cfg(not(feature = "no-mpu"))]
            {
                let (ax, ay, az, temp, gx, gy, gz, imu_ok) = match imu_sample {
                    Some(r) => (r.0, r.1, r.2, r.3, r.4, r.5, r.6, 1i32),
                    None    => (0, 0, 0, 0, 0, 0, 0, 0i32),
                };
                let mut raw_buf = [0u8; 100];
                let mut r_i = 0;
                for &b in b"RAW:" { raw_buf[r_i] = b; r_i += 1; }
                write_u32(elapsed_ms, &mut raw_buf, &mut r_i);
                for val in &[ax, ay, az, gx, gy, gz] {
                    raw_buf[r_i] = b':'; r_i += 1;
                    write_i32(*val as i32, &mut raw_buf, &mut r_i);
                }
                raw_buf[r_i] = b':'; r_i += 1;
                write_i32(counts[1].wrapping_add(counts[3]).wrapping_add(counts[5]), &mut raw_buf, &mut r_i); // Total Left
                raw_buf[r_i] = b':'; r_i += 1;
                write_i32(counts[0].wrapping_add(counts[2]).wrapping_add(counts[4]), &mut raw_buf, &mut r_i); // Total Right
                raw_buf[r_i] = b':'; r_i += 1;
                write_i32(temp as i32, &mut raw_buf, &mut r_i);
                raw_buf[r_i] = b':'; r_i += 1;
                write_i32(imu_ok, &mut raw_buf, &mut r_i);
                raw_buf[r_i] = b'\n'; r_i += 1;
                iface.send_response(&raw_buf[..r_i]);
            }

            // Velocidad por motor: [FR, FL, CR, CL, RR, RL]
            // FR/CR/RR → lado derecho, FL/CL/RL → lado izquierdo
            let speeds = [
                msm.drive.right, msm.drive.left,   // FR, FL
                msm.drive.right, msm.drive.left,   // CR, CL
                msm.drive.right, msm.drive.left,   // RR, RL
            ];
            let stall_thresh = if msm.state == RoverState::Climb { CLB_STALL_THRESHOLD } else { STALL_THRESHOLD };
            let mut stall_mask: u8 = 0;
            for i in 0..6usize {
                if speeds[i].abs() > STALL_SPEED_MIN && counts[i] == last_counts[i] {
                    stall_timers[i] = stall_timers[i].saturating_add(1);
                } else {
                    stall_timers[i] = 0;
                }
                last_counts[i] = counts[i];
                if stall_timers[i] > stall_thresh {
                    stall_mask |= 1 << i;
                }
            }
            // Acumuladores de odometría: izquierdo = FL(1)+CL(3)+RL(5), derecho = FR(0)+CR(2)+RR(4)
            sensor_frame.enc_left  = counts[1].wrapping_add(counts[3]).wrapping_add(counts[5]);
            sensor_frame.enc_right = counts[0].wrapping_add(counts[2]).wrapping_add(counts[4]);
            if stall_mask != 0 {
                msm.update_safety(stall_mask);
                relay.emergency_off(); // corte hardware: stall encoder detectado
                sync_drive!(hard, rover, msm, ramp);
                iface.send_response(format_response(msm.telemetry(stall_mask, sensor_frame), &mut resp_buf));
            }
        }

        // 4. Comando entrante desde RPi5 (interrupt-driven via RX_BUF)
        if iface.poll_from_ring(&RX_BUF) {
            // Copiar a buffer local para liberar el borrow de iface
            let mut cmd_buf = [0u8; 32];
            let cmd_len = {
                let raw = iface.get_command();
                let len = raw.len().min(32);
                cmd_buf[..len].copy_from_slice(&raw[..len]);
                len
            };
            let cmd_bytes = &cmd_buf[..cmd_len];
            let response = match parse_command(cmd_bytes) {
                Some(cmd) => msm.process(cmd),
                None      => Response::ErrUnknown,
            };
            // Aplicar efectos secundarios de relay según la respuesta.
            match response {
                Response::BankChange(mode) => relay.set_mode(mode),
                Response::Ack(RoverState::Fault) | Response::Ack(RoverState::Safe) => {
                    relay.emergency_off(); // FLT/SAFE por comando GCS
                }
                Response::Ack(RoverState::Standby) if msm.state == RoverState::Standby => {
                    // RST regresa a Standby: restaurar Bank 2 (relay vuelve a normal)
                    relay.reset_normal();
                }
                _ => {}
            }
            sync_drive!(soft, rover, msm, ramp);
            iface.send_response(format_response(response, &mut resp_buf));
        }

        // 5a. Chequeo rápido de sobrecorriente — cada SEN_FAST_PERIOD ciclos (~60 ms).
        //     Solo 2 muestras por canal (~1.25 ms bloqueantes): detecta Fault únicamente.
        //     No actualiza sensor_frame (usa las últimas lecturas lentas para TLM).
        sen_fast_counter = sen_fast_counter.wrapping_add(1);
        if sen_fast_counter >= SEN_FAST_PERIOD && msm.state != RoverState::Fault {
            sen_fast_counter = 0;
            let fast_raw = [
                adc_avg!(acs_fr_pin, adc, SEN_FAST_SAMPLES),
                adc_avg!(acs_fl_pin, adc, SEN_FAST_SAMPLES),
                adc_avg!(acs_cr_pin, adc, SEN_FAST_SAMPLES),
                adc_avg!(acs_cl_pin, adc, SEN_FAST_SAMPLES),
                adc_avg!(acs_rr_pin, adc, SEN_FAST_SAMPLES),
                adc_avg!(acs_rl_pin, adc, SEN_FAST_SAMPLES),
            ];
            let mut fault_mask: u8 = 0;
            for i in 0..6usize {
                if acs[i].read_ma(fast_raw[i]).abs() > OC_FAULT[i] {
                    fault_mask |= 1 << i;
                }
            }
            if fault_mask != 0 {
                msm.update_overcurrent(SafetyState::FaultStall);
                relay.emergency_off(); // corte hardware: sobrecorriente OC_FAULT rápida
                sync_drive!(hard, rover, msm, ramp);
                iface.send_response(format_response(
                    msm.telemetry(fault_mask, sensor_frame), &mut resp_buf));
            }
        }

        // 5b. Sensores analógicos — corriente y temperatura, cada SEN_READ_PERIOD ciclos (~500 ms).
        //     8 muestras: lectura precisa para Warn/Limit + actualiza sensor_frame para TLM.
        sen_counter = sen_counter.wrapping_add(1);
        if sen_counter >= SEN_READ_PERIOD {
            sen_counter = 0;

            let raw_i = [
                adc_avg!(acs_fr_pin, adc, SEN_SAMPLES),
                adc_avg!(acs_fl_pin, adc, SEN_SAMPLES),
                adc_avg!(acs_cr_pin, adc, SEN_SAMPLES),
                adc_avg!(acs_cl_pin, adc, SEN_SAMPLES),
                adc_avg!(acs_rr_pin, adc, SEN_SAMPLES),
                adc_avg!(acs_rl_pin, adc, SEN_SAMPLES),
            ];

            // Clasificar el peor nivel de corriente entre los 6 motores.
            // Cada motor usa su propia instancia ACS712 y sus umbrales OC_*[i].
            let mut worst = SafetyState::Normal;
            for i in 0..6usize {
                let current_ma = acs[i].read_ma(raw_i[i]);
                sensor_frame.currents[i] = current_ma;
                let abs_ma = current_ma.abs();
                let level = if abs_ma > OC_FAULT[i] {
                    SafetyState::FaultStall
                } else if abs_ma > OC_LIMIT[i] {
                    SafetyState::Limit
                } else if abs_ma > OC_WARN[i] {
                    SafetyState::Warn
                } else {
                    SafetyState::Normal
                };
                if level > worst { worst = level; }
            }

            #[cfg(not(feature = "no-lm335"))]
            {
                let raw_t = adc_avg!(lm335_pin, adc, SEN_SAMPLES);
                let t_amb = lm335.read_celsius(raw_t);
                match check_temp_c(t_amb, AMBIENT_TEMP_MIN_C, AMBIENT_TEMP_MAX_C) {
                    Some(t_valid) => {
                        sensor_frame.temp_c = t_valid;
                        if t_valid >= AMBIENT_SAFE_C
                            && msm.state != RoverState::Safe
                            && msm.state != RoverState::Fault
                        {
                            msm.process(Command::Safe);
                            relay.emergency_off();
                            sync_drive!(hard, rover, msm, ramp);
                            iface.log("ALARM:AMBIENT_OVERHEAT_SAFE_MODE");
                        }
                    }
                    None => iface.log("WARN:LM335_OOR"),
                }
            }

            // INA3221 — voltaje de los 3 bancos (modo bus-only, sin corriente).
            // batt_mv reporta el mínimo de los 3 para que la protección de batería
            // baja dispare si cualquier banco está descargado.
            if ina.ready {
                let [v_b1, v_b2, v_b3] = ina.read_all_mv();
                sensor_frame.batt_mv = v_b1.min(v_b2).min(v_b3);
                sensor_frame.batt_ma = 0; // modo bus-only: no medimos corriente
            // Protección de batería baja (EPS-REQ-002 / SYS-FUN-040a):
            // Si el INA226 lee < 12 V (con lectura válida > 5 V para descartar
            // el arranque en frío), el LLC entra en Safe Mode directamente sin
            // esperar al HLC. El HLC también detecta esta condición vía TLM y
            // emite Command::Safe, pero la protección en LLC es la última línea.
            if sensor_frame.batt_mv > 5000 && sensor_frame.batt_mv < 12000 {
                if msm.state != RoverState::Safe && msm.state != RoverState::Fault {
                    msm.process(Command::Safe);
                    relay.emergency_off(); // corte hardware: batería crítica Bank 1
                    sync_drive!(hard, rover, msm, ramp);
                    iface.log("ALARM:LOW_BATTERY_SAFE_MODE");
                }
            }
            }

            // Leer NTC de batería y clasificar el peor nivel térmico.
            // ntc4: solo hay 4 NTC físicos (A7–A10). A11/A12 quedan al aire y
            // leerían ~50–56 °C basura → falso LIMIT. Con ntc4 no se clasifican.
            // no-ntc: omite por completo la lectura y clasificación térmica de los
            // termistores de batería. Un NTC desconectado lee -20/100 °C (extremos
            // del rango válido) → dispararía un FAULT de sobre-temperatura FALSO.
            // Con no-ntc, batt_temps queda en 0 y nunca contribuye a `worst`.
            #[cfg(not(feature = "no-ntc"))]
            {
                const NTC_N: usize = if cfg!(feature = "ntc4") { 4 } else { 6 };
                let raw_batt = [
                    adc_avg!(ntc_b1a_pin, adc, SEN_SAMPLES),
                    adc_avg!(ntc_b1b_pin, adc, SEN_SAMPLES),
                    adc_avg!(ntc_b2a_pin, adc, SEN_SAMPLES),
                    adc_avg!(ntc_b2b_pin, adc, SEN_SAMPLES),
                    adc_avg!(ntc_b3a_pin, adc, SEN_SAMPLES),
                    adc_avg!(ntc_b3b_pin, adc, SEN_SAMPLES),
                ];
                for i in 0..NTC_N {
                    let t_raw = ntc_batt[i].read_celsius(raw_batt[i]);
                    // Validar plausibilidad antes de usar en clasificación de seguridad.
                    // NTC desconectado (ADC flotante) puede devolver -20 °C o 100 °C,
                    // valores que están en los extremos del rango válido. Si el ADC
                    // devuelve un valor imposible (fuera de -20..100) es basura → ignorar.
                    let t = match check_temp_c(t_raw, BATT_TEMP_MIN_C, BATT_TEMP_MAX_C) {
                        Some(t_valid) => t_valid,
                        None => {
                            iface.log("WARN:NTC_OOR");
                            continue; // no actualizar sensor_frame ni clasificar nivel
                        }
                    };
                    sensor_frame.batt_temps[i] = t;
                    let level = if t > BATT_FAULT_C {
                        SafetyState::FaultStall
                    } else if t > BATT_LIMIT_C {
                        SafetyState::Limit
                    } else if t > BATT_WARN_C {
                        SafetyState::Warn
                    } else {
                        SafetyState::Normal
                    };
                    if level > worst { worst = level; }
                }
            }

            let prev_safety = msm.safety;
            let faulted = msm.update_overcurrent(worst);

            // sync_drive si: hay fault, safety cambió (cap aplicado/eliminado), o sigue en Warn/Limit.
            // hard si hay fault real (sobrecorriente superó OC_FAULT en la lectura lenta);
            // soft si solo cambió el nivel Warn/Limit (aplicar o levantar el speed cap).
            if faulted {
                sync_drive!(hard, rover, msm, ramp); // sobrecorriente confirmada: stop inmediato
            } else if msm.safety != prev_safety || msm.safety != SafetyState::Normal {
                sync_drive!(soft, rover, msm, ramp); // cap de velocidad: rampa hacia nuevo límite
            }
            // TLM inmediato si el estado no es Normal
            if faulted || msm.safety != SafetyState::Normal {
                iface.send_response(format_response(
                    msm.telemetry(0, sensor_frame), &mut resp_buf));
            }
        }

        // 6. Telemetría periódica (~1 s) — stall_mask siempre 0 aquí porque
        //    si hubiera stall ya se reportó en el bloque de encoders arriba.
        tlm_counter = tlm_counter.wrapping_add(1);
        if tlm_counter >= TLM_PERIOD {
            tlm_counter = 0;
            iface.send_response(format_response(msm.telemetry(0, sensor_frame), &mut resp_buf));
        }

        // 7. Convergencia de rampa garantizada en estados SIN tracción.
        //    FIX bug "FR gira solo en FAULT": `sync_drive!` es event-driven; el
        //    watchdog dispara la transición a Fault UNA sola vez (msm.tick()
        //    retorna Some solo en el flanco), por lo que la rampa solo avanza
        //    UN paso (p.ej. 30→20%) y luego se CONGELA — nada vuelve a llamar
        //    sync_drive! sin PING/obstáculo/stall/OC. Los 6 motores quedan con
        //    PWM residual; la rueda de menor fricción (FR) es la única que lo
        //    vence y sigue girando. Aquí forzamos que la rampa llegue a 0 en
        //    ≤200 ms (RAMP_STEP_SOFT) en Fault/Safe/Standby y se quede ahí.
        //    Solo actúa en estados de reposo y solo mientras la rampa ≠ 0 → no
        //    interfiere con la rampa event-driven de Explore/Climb (no doble paso).
        //    DEFAULT: garantiza además que ramp.actual quede en 0 al desactivar
        //    los drivers, para un re-arranque limpio (no salta desde un residual).
        {
            match msm.state {
                RoverState::Fault | RoverState::Safe | RoverState::Standby => {
                    if ramp.actual_l != 0 || ramp.actual_r != 0 {
                        sync_drive!(soft, rover, msm, ramp);
                    }
                }
                _ => {}
            }
        }

        // Actualizar timestamp relativo antes del delay para que el próximo
        // ciclo (y el TLM periódico) reflejen el tiempo acumulado real.
        elapsed_ms = elapsed_ms.wrapping_add(LOOP_MS);
        sensor_frame.tick_ms = elapsed_ms;

        // delay_ms restaurado: la ISR USART0_RX garantiza que ningún byte
        // se pierde durante el bloqueo. Ver docs/debug_usart_overflow.md.
        arduino_hal::delay_ms(LOOP_MS);
    }
}

fn write_u32(val: u32, buf: &mut [u8], i: &mut usize) {
    if val == 0 {
        buf[*i] = b'0'; *i += 1; return;
    }
    let mut tmp = [0u8; 10];
    let mut len = 0;
    let mut v = val;
    while v > 0 {
        tmp[len] = b'0' + (v % 10) as u8;
        v /= 10;
        len += 1;
    }
    for k in (0..len).rev() {
        buf[*i] = tmp[k]; *i += 1;
    }
}

/// Escribe un byte como "0xHH" (minúsculas).
#[cfg(not(feature = "no-mpu"))]
fn write_hex_u8(val: u8, buf: &mut [u8], i: &mut usize) {
    let hex = |n: u8| if n < 10 { b'0' + n } else { b'a' + n - 10 };
    buf[*i] = b'0'; *i += 1;
    buf[*i] = b'x'; *i += 1;
    buf[*i] = hex(val >> 4); *i += 1;
    buf[*i] = hex(val & 0xF); *i += 1;
}

fn write_i32(val: i32, buf: &mut [u8], i: &mut usize) {
    if val < 0 {
        buf[*i] = b'-'; *i += 1;
        write_u32(val.wrapping_neg() as u32, buf, i);
    } else {
        write_u32(val as u32, buf, i);
    }
}
