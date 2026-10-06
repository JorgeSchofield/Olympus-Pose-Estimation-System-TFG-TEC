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
SRCREV = "ea4ce447549f3c852372d9e29c3a75feeedbdd99"
PV = "0.1.0+git"

# scarthgap: the git fetcher unpacks to ${WORKDIR}/git and file:// entries
# land directly in ${WORKDIR}. Both change in Yocto 5.2 (UNPACKDIR); revisit
# if this layer is ever moved off scarthgap.
S = "${WORKDIR}/git"

# The repository root holds LICENSE, README and document/; the buildable
# tree is one level down.
OECMAKE_SOURCEPATH = "${S}/CMAES"

inherit cmake systemd

# Upstream defect: CMAES/CMakeLists.txt still refers to the pre-reorganization
# path "CMAES/libCMAES_pthreads". After commit ea4ce44 that directory is a
# direct child, so cmake fails with "not an existing directory". Delete this
# prepend once the one-line fix lands upstream (change 1 in CHANGES.md).
do_configure:prepend() {
    sed -i 's|add_subdirectory(CMAES/libCMAES_pthreads)|add_subdirectory(libCMAES_pthreads)|' \
        ${OECMAKE_SOURCEPATH}/CMakeLists.txt
}

# Inert on scarthgap (GCC 13 only warns about these), kept as insurance: the
# port has ~40 void/void* constructor mismatches, one NULL-to-pthread_t
# assignment and one implicit function declaration, all of which GCC 14+
# rejects outright. See changes 4 and 5 in CHANGES.md for the real fix.
CFLAGS:append = " -Wno-error=incompatible-pointer-types \
                  -Wno-error=int-conversion \
                  -Wno-error=implicit-function-declaration"

# Upstream CMakeLists.txt declares no install() rules, so install by hand.
# Drop this whole function once change 3 in CHANGES.md is applied.
do_install() {
    install -d ${D}${libdir}
    install -m 0644 ${B}/libCMAES_pthreads/libcmaes_pthreads.a ${D}${libdir}/

    install -d ${D}${includedir}/cmaes
    install -m 0644 ${OECMAKE_SOURCEPATH}/libCMAES_pthreads/include/CMAES.h \
        ${D}${includedir}/cmaes/

    install -d ${D}${bindir}
    install -m 0755 ${B}/linux_demo/sender_receiver/cmaes_sender_receiver_demo \
        ${D}${bindir}/
    install -m 0755 ${B}/linux_demo/rock_paper_scissors/cmaes_rock_paper_scissors_demo \
        ${D}${bindir}/

    # The rover image is currently sysvinit (no INIT_MANAGER in local.conf,
    # DISTRO = "poky"), so this branch does nothing today. It costs nothing
    # and means the recipe is already correct if the image moves to systemd.
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
