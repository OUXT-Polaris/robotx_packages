#include <disparity_image.h>

#include <ros/ros.h>

disparity_image::disparity_image(parameters params)
{
    recieved_right_image_ = false;
    recieved_left_image_ = false;
    params_ = params;

    ssgbm_ = cv::StereoSGBM::create(
        params_.minDisparity, params_.numDisparities, params_.blockSize, params_.P1, params_.P2, 
        params_.disp12MaxDiff, params_.preFilterCap, params_.uniquenessRatio, params_.speckleWindowSize , 
        params_.speckleRange, cv::StereoSGBM::MODE_SGBM);
}

disparity_image::disparity_image()
{
    recieved_right_image_ = false;
    recieved_left_image_ = false;
    params_ = parameters();

    ssgbm_ = cv::StereoSGBM::create(
        params_.minDisparity, params_.numDisparities, params_.blockSize, params_.P1, params_.P2, 
        params_.disp12MaxDiff, params_.preFilterCap, params_.uniquenessRatio, params_.speckleWindowSize , 
        params_.speckleRange, cv::StereoSGBM::MODE_SGBM);
}

disparity_image::~disparity_image()
{

}

void disparity_image::set_parameters(parameters params)
{
    params_ = params;
    ssgbm_ = cv::StereoSGBM::create(
        params_.minDisparity, params_.numDisparities, params_.blockSize, params_.P1, params_.P2, 
        params_.disp12MaxDiff, params_.preFilterCap, params_.uniquenessRatio, params_.speckleWindowSize , 
        params_.speckleRange, cv::StereoSGBM::MODE_SGBM);
}

void disparity_image::set_left_image(cv::Mat left_image)
{
    recieved_left_image_ = true;
    left_image_ = left_image;
}

void disparity_image::set_right_image(cv::Mat right_image)
{
    recieved_right_image_ = true;
    right_image_ = right_image;
}

bool disparity_image::get_disparity_image(cv::Mat& disparity)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if(recieved_right_image_ == false || recieved_left_image_ == false)
    {
        return false;
    }
    ssgbm_->compute(left_image_ ,right_image_ , disparity);
    return true;

}