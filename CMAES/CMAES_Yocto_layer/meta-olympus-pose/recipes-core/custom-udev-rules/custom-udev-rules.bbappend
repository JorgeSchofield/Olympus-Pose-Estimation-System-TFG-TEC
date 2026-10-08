# Arduino Mega real port renamed to /dev/arduino_mega_hw: llcmux owns it and
# publishes /dev/arduino_mega as a pty for olympus_hlc (agents/DESIGN.md §6, D13).
# Only 99-arduino.rules is replaced (files/ here is searched first); 99-i2c.rules
# still comes from meta-olympus.
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
