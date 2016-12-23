Udev rules
==========

These udev rules are provided as a reference for some sensors typically installed on the Husky. If you are using an
Clearpath-provided image to setup your husky PC, they should be installed automatically.

To install them manually, execute:

sudo cp $(rospack find husky_bringup)/udev/* /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
