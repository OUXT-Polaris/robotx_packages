#include <disparity_image.h>

disparity_image::disparity_image(parameters params)
{
    recieved_right_image_ = false;
    recieved_left_image_ = false;
    params_ = params;

    ssgbm_ = cv::StereoSGBM::create(
    params.minDisparity, params.numDisparities, params.blockSize, params.P1, params.P2, 
    params.disp12MaxDiff, params.preFilterCap, params.uniquenessRatio, params.speckleWindowSize , 
    params.speckleRange, cv::StereoSGBM::MODE_SGBM);
}

disparity_image::~disparity_image()
{

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