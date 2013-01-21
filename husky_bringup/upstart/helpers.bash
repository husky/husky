
function install_udev_rules {
  cp `rospack find ${robot}_bringup`/udev/* /etc/udev/rules.d/
  cp `rospack find ${robot}_bringup`/bin/clearpath-name /usr/sbin/
  service udev restart
}

function do_subs {
  source_file=$1
  dest_file=$2
  interface=$3
  portnum=$4
  robot=$5
  job=$6
  release=$7
  user=$8
  cp $source_file $dest_file
  sed -i "s/interface0/$interface/g" $2
  sed -i "s/portnum/$portnum/g" $2
  sed -i "s/robot/$robot/g" $2
  sed -i "s/job/$job/g" $2
  sed -i "s/release/$release/g" $2 
  sed -i "s/user/$user/g" $2 
}

function install_job {
  job=$1
  interface=$2
  portnum=$3

  echo "Installing $robot-$job using network interface $interface, port $portnum."
  
  cp $stackPath/mklaunch /usr/sbin/mklaunch
  do_subs $stackPath/start /usr/sbin/$robot-$job-start $interface $portnum $robot $job $release $user
  chmod +x /usr/sbin/$robot-$job-start

  do_subs $stackPath/stop /usr/sbin/$robot-$job-stop $interface $portnum $robot $job $release $user
  chmod +x /usr/sbin/$robot-$job-stop

  do_subs $stackPath/job.conf /etc/init/$robot-$job.conf $interface $portnum $robot $job $release $user

  # Copy launch files into /etc/ros/
  launch_path=/etc/ros/$release/$robot/$job.d
  mkdir -p $launch_path 
  cp `rospack find ${robot}_bringup`/launch/$job/* $launch_path
}


