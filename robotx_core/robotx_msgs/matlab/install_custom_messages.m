path = pwd;
ros_directory_path = strcat(path,'/../../');
rosgenmsg(ros_directory_path)
addpath(strcat(ros_directory_path,'/matlab_gen/msggen'))
savepath