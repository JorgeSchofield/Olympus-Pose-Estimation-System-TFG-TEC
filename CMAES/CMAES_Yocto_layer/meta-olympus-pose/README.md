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

The layer lives inside the Pose Estimation repository, at
`CMAES/CMAES_Yocto_layer/meta-olympus-pose`. Clone that repository next to
the rover's build tree and add the layer by its path. The path must not
contain spaces: `BBLAYERS` is a space-separated list, which is why the
folder is `CMAES_Yocto_layer` and not `CMAES Yocto layer`.

```bash
cd olympus-hlc-rpi5
git clone https://github.com/JorgeSchofield/Olympus-Pose-Estimation-System-TFG-TEC.git layers/olympus-pose-repo
source layers/poky/oe-init-build-env build
bitbake-layers add-layer ../layers/olympus-pose-repo/CMAES/CMAES_Yocto_layer/meta-olympus-pose
bitbake cmaes            # recipe alone first
bitbake olympus-image
```

The clone only provides the layer metadata. The `cmaes` recipe fetches the
sources itself from GitHub at the pinned `SRCREV` on branch `main`, so a
library change reaches the image only after it is merged into `main` and
`SRCREV` is bumped to that commit.

Nothing in `local.conf` needs to change. `enable_uart=1` and
`dtparam=i2c_arm=1` are already in `RPI_EXTRA_CONFIG`.

## Packages produced

| Package | Contents |
| --- | --- |
| `cmaes` | empty (the library is static) |
| `cmaes-staticdev` | `libcmaes_pthreads.a` |
| `cmaes-dev` | `${includedir}/cmaes/CMAES.h` |
| `cmaes-demos` | the three demo binaries (`sender_receiver`, `rock_paper_scissors`, `rps_stress`), plus units if the image uses systemd |

## Caveats

1. **Source status.** The recipe builds the upstream CMake project as is, with
   no `sed`, no hand-written `do_install` and no `-Wno-error` flags. Since
   `671bf4a` the library builds as strict C99 without warnings, so it is
   ready for GCC 14 as well as scarthgap's GCC 13.
2. **Init system: probably sysvinit, to be confirmed on the board.**
   `build/conf/local.conf` sets no `INIT_MANAGER` and `DISTRO = "poky"`
   defaults to sysvinit in scarthgap, so the `.service` files would be
   installed but never run. The rover's own decision log says the image
   uses systemd, so check with `ps -p 1 -o comm=`. On sysvinit, run the demos
   from the shell. Do not change `INIT_MANAGER` for this: it rebuilds the
   whole image and changes the rover's baseline.
3. **`SCHED_FIFO` needs privilege.** Without `CAP_SYS_NICE` the port prints
   one warning and falls back to `SCHED_OTHER` with advisory priorities.
   Root has it.
4. **The Pi kernel is not `PREEMPT_RT`.** Priorities are honoured, worst-case
   latency is not bounded. Relevant to the < 100 ms indicator: measure it.
5. **Static library.** `cmaes-staticdev`, not a `.so`. The agent application
   links it in; there is no runtime package to install on the rover.
