export DEB_BUILD_OPTIONS='parallel=8'

for package in robotx_driver robotx_msgs robotx_navigation robotx_recognition wamv_description
do
    roscd $package
    bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic --skip-package-names ignore.txt
    dh_auto_configure --parallel
    fakeroot debian/rules binary
    rm -rf debian
    rm -rf obj-x86_64-linux-gnu
done

cd ../
mv *amd64.deb ./build/amd64