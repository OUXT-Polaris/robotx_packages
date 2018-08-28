echo "Warning ! Please install tensorflow for Jetson TX2 before running this script!!"
read -p "Are You Ready? (y/n)" ans
case $ans in
    [Yy] | [Yy][Ee][Ss] )
    rosdep install -i -r -y --from-paths . --rosdistro kinetic
esac
cd /home/nvidia/catkin_ws/ catkin_make --pkg robotx_driver robotx_msgs robotx_navigation robotx_recognition wamv_description