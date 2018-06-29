#include <cuda_diagnostic.h>

cuda_diagnostic::cuda_diagnostic() : params_()
{
    updater_.setHardwareID(params_.device_id); 
    updater_.add("cuda-memory", this, &cuda_diagnostic::update_memory_usage_);
}

cuda_diagnostic::~cuda_diagnostic()
{
    
}

void cuda_diagnostic::run()
{
    ros::Rate rate(params_.update_frequency);
    while(ros::ok())
    {
        updater_.update();
        rate.sleep();
    }
}

void cuda_diagnostic::update_memory_usage_(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    size_t free;
    size_t total;
    cudaError_t result = cudaMemGetInfo(&free,&total);
    if(result != cudaSuccess)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "cudaMemGetInfo failed");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "cudaMemGetInfo succeed");
    }
    stat.add("cuda memory", 0);
}