roscd robotx_msgs
rm -rf debian
rm -rf obj-x86_64-linux-gnu
bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
fakeroot debian/rules binary

cd ../
mv *.deb build/amd64
cd build