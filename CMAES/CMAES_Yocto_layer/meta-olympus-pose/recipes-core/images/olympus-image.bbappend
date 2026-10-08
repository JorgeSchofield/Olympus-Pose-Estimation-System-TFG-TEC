# Adds the pose-estimation subsystem to the rover image:
#   olympus-pose         the CMAES agents, /etc/olympus-pose/pose.conf, tools
#   olympus-pose-llcmux  the LLC link multiplexer (starts at boot, S90)
#
# The CMAES demos (cmaes-demos) are no longer installed: the OE3 indicator was
# closed on 2026-10-07. Add " cmaes-demos" back to repeat that test.
#
# If you would rather not modify the rover image recipe, delete this file and
# put the same line in build/conf/local.conf instead.

IMAGE_INSTALL:append = " olympus-pose olympus-pose-llcmux"
