#include <cuda_diagnostic.h>

cuda_diagnostic::cuda_diagnostic() : params_()
{
    updater_.setHardwareID(params_.device_id); 
    updater_.add("gpu-memory-usage", this, &cuda_diagnostic::update_memory_usage_);
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
        stat.add("gpu-memory-usage", "NaN");
    }
    else
    {
        if((double)free/(double)total < params_.low_memory_usage_threshold)
        {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Low free memory");
        }
        else
        {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enough free memory");
        }
        stat.add("gpu-memory-usage", std::to_string((double)free/(double)total*100)+"%");
    }
}