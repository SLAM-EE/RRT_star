#!/bin/bash 
xhost +local:root

docker run --privileged -it \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/home/$USER:/home/$USER" \
--workdir="/home/ORB_SLAM2" \
--volume="/home/$USER/expansion_drive/kitti/dataset:/home/dataset" \
--volume="/home/aswin/expansion_drive:/home/custom_dataset" \
--volume="/home/$USER/xcode/magellan/mapper_orb_slam2:/home/ORB_SLAM2" \
--volume="/etc/group:/etc/group:ro" \
--volume="/etc/passwd:/etc/passwd:ro" \
--volume="/etc/shadow:/etc/shadow:ro" \
--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
--volume="$HOME/host_docker:/home/user/host_docker" \
--device /dev/dri:/dev/dri \
--network=host \
-e LOCAL_USER_ID=`id -u $USER` \
-e LOCAL_GROUP_ID=`id -g $USER` \
-e LOCAL_GROUP_NAME=`id -gn $USER` \
    youyu/orb_slam2:ubuntu18

xhost -local:root

