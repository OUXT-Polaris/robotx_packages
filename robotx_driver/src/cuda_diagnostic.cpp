#include <cuda_diagnostic.h>

cuda_diagnostic::cuda_diagnostic() : params_()
{
    updater_.setHardwareID(params_.device_id); 
    updater_.add("gpu-memory-usage", this, &cuda_diagnostic::update_memory_usage_);
    updater_.add("gpu-temperature", this, &cuda_diagnostic::update_temperature_);
}

cuda_diagnostic::~cuda_diagnostic()
{
    
}

bool cuda_diagnostic::exec_shell_cmd(const char* cmd, std::string& stdOut, int& exitCode)
{
	std::shared_ptr<FILE> pipe(popen(cmd, "r"), [&](FILE* p) {exitCode = pclose(p); });
	if (!pipe)
    {
		return false;
	}
	std::array<char, 256> buf;
	while (!feof(pipe.get()))
    {
		if (fgets(buf.data(), buf.size(), pipe.get()) != nullptr)
        {
			stdOut += buf.data();
		}
	}
	return true;
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

void cuda_diagnostic::update_temperature_(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    std::string device_id_query_cmd = "cat /sys/devices/virtual/thermal/thermal_zone*/type";
    std::string device_id_query_result;
    int device_id_query_return_code;
    if(exec_shell_cmd(device_id_query_cmd.c_str(),device_id_query_result,device_id_query_return_code))
    {
        //ROS_ERROR_STREAM(device_id_query_result);
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "failed to execute cat /sys/devices/virtual/thermal/thermal_zone*/type");
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