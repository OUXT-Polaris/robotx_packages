echo "Warning ! Please install tensorflow for Jetson TX2 before running this script!! Please read http://robotx.osaka/2018/01/05/compile-tensorflow-on-jetson-tx2/"
read -p "Are You Ready? (y/n)" ans
case $ans in
    [Yy] | [Yy][Ee][Ss] )
    rosdep install -i -r -y --from-paths . --rosdistro kinetic
esac
cd /home/nvidia/catkin_ws/
catkin_make --pkg robotx_msgs robotx_navigation robotx_recognition wamv_description