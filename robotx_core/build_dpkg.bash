for package in robotx_msgs robotx_driver robotx_navigation robotx_recognition wamv_description
do
    roscd $package
    bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic --skip-package-names ignore.txt
    dh_auto_configure
    fakeroot debian/rules binary
    rm -rf debian
    rm -rf obj-x86_64-linux-gnu
done

cd ../
mv *.deb ./build/amd64