SUMMARY = "Olympus pose estimation: CMAES agents and LLC link multiplexer"
DESCRIPTION = "olympus-pose: the CMAES multi-agent application that estimates \
the rover's planar pose on the HLC (acquisition, fusion, estimation, \
communication and GPS agents; EKF core generated with MATLAB Coder). llcmux: \
shares the Arduino Mega link between olympus_hlc and olympus-pose. See \
agents/DESIGN.md (ARQ-PE-003) in the repository."
HOMEPAGE = "https://github.com/JorgeSchofield/Olympus-Pose-Estimation-System-TFG-TEC"
SECTION = "apps"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=ba0b40e97387b5107ffd67be88d54f98"

SRC_URI = "git://github.com/JorgeSchofield/Olympus-Pose-Estimation-System-TFG-TEC.git;protocol=https;branch=main \
           file://llcmux.init \
           file://olympus-pose.init \
           file://llcmux.default \
"

# a5e816d: GPS viewer with constant cost per refresh; 7896ee5: model/app parity
# (pipeline.c, est.* exported by the model); ec12765: GPS
# viewer (gps_view.py, OGP1 datagrams); before that firmware v3.0 frame
# (temp, imu_ok, est.dt_source), llcmux whole-line
# framing for olympus_hlc, FR = 2.47 x FL, odom_calib.py.
# The CMAES library is built from the SAME commit (agents/CMakeLists.txt adds ../CMAES), so the app
# and the library can never drift apart. The fetcher requires SRCREV to be on
# branch=main: merge the master->main pull request before running bitbake.
SRCREV = "a5e816d21628cbf747b89b943465a95a76fcedc2"
PV = "0.1.0+git"

S = "${WORKDIR}/git"
OECMAKE_SOURCEPATH = "${S}/agents"

inherit cmake update-rc.d

# The rover image is sysvinit (confirmed on the board, 2026-10-07).
do_install:append() {
    install -d ${D}${sysconfdir}/init.d ${D}${sysconfdir}/default
    install -m 0755 ${WORKDIR}/llcmux.init       ${D}${sysconfdir}/init.d/llcmux
    install -m 0755 ${WORKDIR}/olympus-pose.init ${D}${sysconfdir}/init.d/olympus-pose
    install -m 0644 ${WORKDIR}/llcmux.default    ${D}${sysconfdir}/default/llcmux
}

PACKAGES =+ "${PN}-llcmux"

FILES:${PN}-llcmux = "${bindir}/llcmux \
                      ${sysconfdir}/init.d/llcmux \
                      ${sysconfdir}/default/llcmux"
CONFFILES:${PN}-llcmux = "${sysconfdir}/default/llcmux"
CONFFILES:${PN} = "${sysconfdir}/olympus-pose/pose.conf"

# llcmux first (S90): /dev/arduino_mega must exist before olympus_hlc or
# olympus-pose start. Both stop early at shutdown so the logs are synced.
INITSCRIPT_PACKAGES = "${PN}-llcmux ${PN}"
INITSCRIPT_NAME:${PN}-llcmux = "llcmux"
INITSCRIPT_PARAMS:${PN}-llcmux = "start 90 2 3 4 5 . stop 10 0 1 6 ."
INITSCRIPT_NAME:${PN} = "olympus-pose"
INITSCRIPT_PARAMS:${PN} = "start 91 2 3 4 5 . stop 09 0 1 6 ."

# The Python test tools (pose_listen.py, pose_ctl.py, llc_emulator.py, capture_to_replay.py,
# odom_calib.py, gps_view.py: web server + JSON, gps_emulator.py).
RDEPENDS:${PN} += "python3-core python3-csv python3-terminal python3-netserver python3-json"
