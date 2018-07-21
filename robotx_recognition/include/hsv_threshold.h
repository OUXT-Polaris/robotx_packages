#ifndef HSV_THRESHOLD_H_INCLUDED
#define HSV_THRESHOLD_H_INCLUDED

#include <array>

class hsv_threshold {
 public:
  hsv_threshold(std::string name,
                double min_h,
                double max_h,
                double min_s,
                double max_s,
                double min_v,
                double max_v,
                double min_area,
                double max_area)
      : target_buoy_name(name) {
    this->min_h = min_h;
    this->max_h = max_h;
    this->min_s = min_s;
    this->max_s = max_s;
    this->min_v = min_v;
    this->max_v = max_v;
    this->min_area = min_area;
    this->max_area = max_area;
  }
  void get_threshold(
      double& min_h, double& max_h, double& min_s, double& max_s, double& min_v, double& max_v) {
    min_h = this->min_h;
    max_h = this->max_h;
    min_s = this->min_s;
    max_s = this->max_s;
    min_v = this->min_v;
    max_v = this->max_v;
  }
  void get_area_threshold(double& min_area, double& max_area) {
    min_area = this->min_area;
    max_area = this->max_area;
  }
  const std::string target_buoy_name;

 private:
  double min_h;
  double max_h;
  double min_s;
  double max_s;
  double min_v;
  double max_v;
  double min_area;
  double max_area;
};

#endif
