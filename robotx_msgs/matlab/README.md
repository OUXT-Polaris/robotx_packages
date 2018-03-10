#How to build costom ros messages for MATLAB/Simulink

##Install matlab
Install matlab and robotics system toolbox.

##Install robotics addons
Read this [documentation](https://jp.mathworks.com/help/robotics/custom-message-support.html).

execute this command in matlab command line

```matlab
roboticsAddons
```

##Build and install custom messages

open Linux terminal and execute these commands.

```bash
cd (path_to_this_directory)
sudo matlab install_custom_messages.m
```
run this command in matlab command lines

```
install_custom_messages
```
