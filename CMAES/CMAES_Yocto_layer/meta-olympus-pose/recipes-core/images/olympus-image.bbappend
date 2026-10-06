# Adds the CMAES demos to the rover image for bring-up.
#
# Bring-up only. Once the Pose Estimation agent application exists, replace
# "cmaes-demos" with the application package and drop the demos: they run
# forever and print continuously, which is not something the flight image
# should ship.
#
# If you would rather not modify the rover image at all, delete this file and
# put the same line in build/conf/local.conf instead — the effect is
# identical and it leaves the rover layer untouched.

IMAGE_INSTALL:append = " cmaes-demos"
