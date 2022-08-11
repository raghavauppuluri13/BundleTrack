BUNDLETRACK_DIR="/home/ruppulur/catkin_ws/src/BundleTrack"
NOCS_DIR="/home/ruppulur/catkin_ws/src/nocs_ours_results"
YCBINEOAT_DIR="/home/ruppulur/catkin_ws/src/ycb"
echo "BUNDLETRACK_DIR $BUNDLETRACK_DIR"
echo "NOCS_DIR $NOCS_DIR"
echo "YCBINEOAT_DIR $YCBINEOAT_DIR"

docker run --gpus all -it --network=host --name bundletrack  -m  16000m --cap-add=SYS_PTRACE --security-opt seccomp=unconfined  -v $BUNDLETRACK_DIR:$BUNDLETRACK_DIR:rw -v $NOCS_DIR:$NOCS_DIR -v $YCBINEOAT_DIR:$YCBINEOAT_DIR -v /tmp:/tmp  --ipc=host -e DISPLAY=${DISPLAY} -e GIT_INDEX_FILE mars-bundletrack-ros:latest bash
