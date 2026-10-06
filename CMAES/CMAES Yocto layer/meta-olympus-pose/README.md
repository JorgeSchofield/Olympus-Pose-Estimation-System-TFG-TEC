# meta-olympus-pose

Yocto layer adding the **CMAES pthreads backend** to the Olympus rover's
existing HLC image (TFG, Electronic Engineering, TEC — SETEC Lab, ELANaV).

Targets **Yocto scarthgap (5.0 LTS)**, matching the rover's current build
(`Alonso11/olympus-hlc-rpi5`, `LAYERSERIES_COMPAT_meta-olympus = "scarthgap"`,
`DISTRO = "poky"`, `MACHINE = "raspberrypi5"`, `PREFERRED_VERSION_linux-raspberrypi = "6.12%"`).

## Why the name

The rover layer's directory is `layers/meta-olympus` and it registers the
collection `meta-olympus`. This layer must not reuse either. Keep the
directory named `meta-olympus-pose`; the collection is `olympus-pose`.

## Contents

```
conf/layer.conf                                  layer metadata
recipes-cmaes/cmaes/cmaes_0.1.0.bb               library + demos
recipes-cmaes/cmaes/files/*.service              systemd units (inert on sysvinit)
recipes-core/images/olympus-image.bbappend       adds cmaes-demos to the rover image
```

## Install

```bash
cd olympus-hlc-rpi5
git clone <this layer> layers/meta-olympus-pose
source layers/poky/oe-init-build-env build
bitbake-layers add-layer ../layers/meta-olympus-pose
bitbake cmaes            # recipe alone first
bitbake olympus-image
```

Nothing in `local.conf` needs to change. `enable_uart=1` and
`dtparam=i2c_arm=1` are already in `RPI_EXTRA_CONFIG`.

## Packages produced

| Package | Contents |
| --- | --- |
| `cmaes` | empty (the library is static) |
| `cmaes-staticdev` | `libcmaes_pthreads.a` |
| `cmaes-dev` | `${includedir}/cmaes/CMAES.h` |
| `cmaes-demos` | both demo binaries, plus units if the image uses systemd |

## Caveats

1. **Source changes.** `CMAES/CMakeLists.txt` has a broken
   `add_subdirectory` path; the recipe patches it with a `sed` in
   `do_configure:prepend`. See `CHANGES.md` — change 1 is required, the rest
   remove workarounds carried here.
2. **The rover image is sysvinit, not systemd.** `build/conf/local.conf`
   sets no `INIT_MANAGER` and `DISTRO = "poky"` defaults to sysvinit, so the
   `.service` files are installed but never run. Verify on the board with
   `ps -p 1 -o comm=`. Run the demos from the shell instead, or set
   `INIT_MANAGER = "systemd"` — which is a rebuild of the whole image and a
   change to the rover's baseline, so not something to do casually.
3. **`SCHED_FIFO` needs privilege.** Without `CAP_SYS_NICE` the port prints
   one warning and falls back to `SCHED_OTHER` with advisory priorities.
   Root has it.
4. **The Pi kernel is not `PREEMPT_RT`.** Priorities are honoured, worst-case
   latency is not bounded. Relevant to the < 100 ms indicator: measure it.
5. **Static library.** `cmaes-staticdev`, not a `.so`. The agent application
   links it in; there is no runtime package to install on the rover.
