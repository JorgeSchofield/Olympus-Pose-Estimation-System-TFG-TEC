SUMMARY = "CMAES multi-agent framework — POSIX/pthreads backend"
DESCRIPTION = "Native POSIX/pthreads port of CMAES (Conejo Boza, TEC), the C \
implementation of the FreeMAES multi-agent paradigm. Provides the static \
library used by the Olympus rover Pose Estimation agents on the HLC, plus \
the two upstream smoke-test demos (sender_receiver, rock_paper_scissors)."
HOMEPAGE = "https://github.com/JorgeSchofield/Olympus-Pose-Estimation-System-TFG-TEC"
SECTION = "libs"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=ba0b40e97387b5107ffd67be88d54f98"

SRC_URI = "git://github.com/JorgeSchofield/Olympus-Pose-Estimation-System-TFG-TEC.git;protocol=https;branch=main \
           file://cmaes-demo-sender-receiver.service \
           file://cmaes-demo-rock-paper-scissors.service \
"

# Pin the revision. Bump this on every port change so sstate invalidates
# correctly — never AUTOREV in a build that has to be reproducible.
# 6358206: install() rules after add_library (main at a56a5e8 fails to
# configure) and the ConstructorUSER_DEF_COND prototype. The fetcher requires
# SRCREV to be reachable from branch=main, so merge the master→main pull
# request that carries this commit before running bitbake.
SRCREV = "63582061683a3605a02ea673ddb63fafb084f353"
PV = "0.1.0+git"

# scarthgap: the git fetcher unpacks to ${WORKDIR}/git and file:// entries
# land directly in ${WORKDIR}. Both change in Yocto 5.2 (UNPACKDIR); revisit
# if this layer is ever moved off scarthgap.
S = "${WORKDIR}/git"

# The repository root holds LICENSE, README and document/; the buildable
# tree is one level down.
OECMAKE_SOURCEPATH = "${S}/CMAES"

inherit cmake systemd

# The port still has ~37 void/void* function-pointer mismatches in the
# constructors. GCC 13 (scarthgap) only warns about them; GCC 14+ rejects them.
# Kept as insurance until those assignments are fixed in the library.
CFLAGS:append = " -Wno-error=incompatible-pointer-types"

# The library, header and demos are installed by the upstream install() rules
# (default cmake do_install). Only the demo units are added here. They are
# inert on a sysvinit image and installed only when DISTRO_FEATURES has systemd.
do_install:append() {
    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}${systemd_system_unitdir}
        install -m 0644 ${WORKDIR}/cmaes-demo-sender-receiver.service \
            ${D}${systemd_system_unitdir}/
        install -m 0644 ${WORKDIR}/cmaes-demo-rock-paper-scissors.service \
            ${D}${systemd_system_unitdir}/
    fi
}

PACKAGES =+ "${PN}-demos"

FILES:${PN}-demos = "${bindir}/cmaes_sender_receiver_demo \
                     ${bindir}/cmaes_rock_paper_scissors_demo \
                     ${systemd_system_unitdir}/cmaes-demo-*.service"

SYSTEMD_PACKAGES = "${PN}-demos"
SYSTEMD_SERVICE:${PN}-demos = "cmaes-demo-sender-receiver.service \
                               cmaes-demo-rock-paper-scissors.service"
SYSTEMD_AUTO_ENABLE:${PN}-demos = "disable"

# Static-only library: ${PN} itself carries no files.
ALLOW_EMPTY:${PN} = "1"
