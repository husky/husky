HUSKY_MAG_CONFIG=$(catkin_find --etc --first-only husky_bringup mag_config.yaml 2>/dev/null)
if [ -z "$HUSKY_MAG_CONFIG" ]; then
  HUSKY_MAG_CONFIG=$(catkin_find --share --first-only husky_bringup config/mag_config_default.yaml 2>/dev/null)
fi

export HUSKY_MAG_CONFIG
