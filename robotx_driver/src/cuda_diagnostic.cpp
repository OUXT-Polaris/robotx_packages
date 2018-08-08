#include <cuda_diagnostic.h>

cuda_diagnostic::cuda_diagnostic() : params_() {
  updater_.setHardwareID(params_.device_id);
  updater_.add("gpu-memory-usage", this, &cuda_diagnostic::update_memory_usage_);
  updater_.add("gpu-temperature", this, &cuda_diagnostic::update_temperature_);
}

cuda_diagnostic::~cuda_diagnostic() {}

bool cuda_diagnostic::exec_shell_cmd(const char *cmd, std::string &stdOut, int &exitCode) {
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), [&](FILE *p) { exitCode = pclose(p); });
  if (!pipe) {
    return false;
  }
  std::array<char, 256> buf;
  while (!feof(pipe.get())) {
    if (fgets(buf.data(), buf.size(), pipe.get()) != nullptr) {
      stdOut += buf.data();
    }
  }
  return true;
}

void cuda_diagnostic::run() {
  ros::Rate rate(params_.update_frequency);
  while (ros::ok()) {
    updater_.update();
    rate.sleep();
  }
}

void cuda_diagnostic::update_temperature_(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  std::string device_id_query_cmd = "cat /sys/devices/virtual/thermal/thermal_zone*/type";
  std::string device_id_query_result;
  int device_id_query_return_code;
  if (exec_shell_cmd(device_id_query_cmd.c_str(), device_id_query_result, device_id_query_return_code)) {
    std::istringstream stream(device_id_query_result);
    std::string field;
    int target_index = -1;
    int i = 0;
    while (std::getline(stream, field)) {
      if (std::equal(field.begin(), field.end(), params_.gpu_device_name.c_str())) {
        target_index = i;
      }
      i++;
    }
    if (target_index == -1) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "does not match any device name in executing cat "
                   "/sys/devices/virtual/thermal/thermal_zone*/type");
      stat.add("gpu-temperature", "NaN");
    } else {
      std::string get_temperature_cmd = "cat /sys/devices/virtual/thermal/thermal_zone*/temp";
      std::string get_temperature_result;
      int get_temperature_return_code;
      if (exec_shell_cmd(get_temperature_cmd.c_str(), get_temperature_result, get_temperature_return_code)) {
        int m = 0;
        std::istringstream temperature_stream(get_temperature_result);
        std::string temperature_field;
        while (std::getline(temperature_stream, temperature_field)) {
          if (m == target_index) {
            double temperature = std::stod(temperature_field) / (double)1000;
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Get GPU temperature succeed");
            stat.add("gpu-temperature", std::to_string(temperature) + " C");
          }
          m++;
        }
      }
    }
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                 "failed to execute cat "
                 "/sys/devices/virtual/thermal/thermal_zone*/type");
    stat.add("gpu-temperature", "NaN");
  }
}

void cuda_diagnostic::update_memory_usage_(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  size_t free;
  size_t total;
  cudaError_t result = cudaMemGetInfo(&free, &total);
  if (result != cudaSuccess) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "cudaMemGetInfo failed");
    stat.add("gpu-memory-usage", "NaN");
  } else {
    if ((double)free / (double)total < params_.low_memory_usage_threshold) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Low free memory");
    } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enough free memory");
    }
    stat.add("gpu-memory-usage", std::to_string((double)free / (double)total * 100) + "%");
  }
}