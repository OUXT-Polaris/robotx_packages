#ifndef DISPARITY_IMAGE_H_INCLUDED
#define DISPARITY_IMAGE_H_INCLUDED

//headers in opencv
#include <opencv2/opencv.hpp>

//headers in STL
#include <mutex>

class disparity_image
{
    struct parameters
    {
        int window_size = 3;
        int minDisparity = 0;
        int numDisparities = 32;
        int blockSize = 3;
        int P1 = 8 * 3 * window_size * window_size;
        int P2 =  32 * 3 * window_size * window_size;
        int disp12MaxDiff = 1;
        int preFilterCap = 0;
        int uniquenessRatio = 10;
        int speckleWindowSize = 100;
        int speckleRange = 32;
    };
public:
    disparity_image(parameters paras);
    ~disparity_image();
    void set_left_image(cv::Mat left_image);
    void set_right_image(cv::Mat right_image);
    bool get_disparity_image(cv::Mat& disparity);
private:
    parameters params_;
    std::mutex mtx_;
    volatile bool recieved_left_image_;
    volatile bool recieved_right_image_;
    cv::Mat left_image_;
    cv::Mat right_image_;
    cv::Ptr<cv::StereoSGBM> ssgbm_;
};

#endif  //DISPARITY_IMAGE_H_INCLUDED