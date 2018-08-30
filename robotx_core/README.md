# robotx_core packages
robotx_core packages are ROS packages for Jetson TX2 or other embeded boards which is on our wam-v.

# How to cross-compile
1.update bloom  
See also https://qiita.com/musubi05/items/8d36f96122ef31145915.  
```
git clone https://github.com/OUXT-Polaris/bloom
cd bloom
sudo python setup.py install

cd <robotx_core dir>
sudo apt update
sudo apt install fakeroot dpkg-dev debhelper
source build_dpkg.bash
``` 