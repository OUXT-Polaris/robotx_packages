#ifndef CUDA_DIAGNOSTIC_H_INCLUDED
#define CUDA_DIAGNOSTIC_H_INCLUDED

//headers in ROS
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>

//headers in CUDA
#include <cuda_runtime.h>

/**
 * @brief cuda diagnostic class
 * it publish status of cuda devices
 */
class cuda_diagnostic
{
    public:
        /**
         * @brief parameters for cuda_diagnostic class
         * 
         */
        struct parameters
        {
            /**
             * @brief device id (use in diagnostic_updater)
             * 
             */
            std::string device_id;
            int num_cuda_devices;
            parameters()
            {
                ros::param::param<std::string>(ros::this_node::getName()+"/device_id", device_id, "");
                cudaError_t error_id = cudaGetDeviceCount(&num_cuda_devices);
                if(error_id != cudaSuccess)
                {
                    ROS_ERROR_STREAM("No cuda device detected. Abort.");
                    std::exit(0);
                }
            }
        };
        cuda_diagnostic();
        ~cuda_diagnostic();
    private:
        ros::NodeHandle nh_;
        diagnostic_updater::Updater updater_;
};
#endif  //CUDA_DIAGNOSTIC_H_INCLUDED