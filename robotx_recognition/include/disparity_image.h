#ifndef DISPARITY_IMAGE_H_INCLUDED
#define DISPARITY_IMAGE_H_INCLUDED

//headers in opencv
#include <opencv2/opencv.hpp>

//headers in STL
#include <mutex>

class disparity_image
{
public:
    struct parameters
    {
        int window_size;
        int minDisparity;
        int numDisparities;
        int blockSize;
        int P1;
        int P2;
        int disp12MaxDiff;
        int preFilterCap;
        int uniquenessRatio;
        int speckleWindowSize;
        int speckleRange;
        int mode;
        parameters()
        {
            window_size = 3;
            minDisparity = 0;
            numDisparities = 32;
            blockSize = 3;
            P1 = 8 * 3 * window_size * window_size;
            P2 =  32 * 3 * window_size * window_size;
            disp12MaxDiff = 0;
            preFilterCap = 0;
            uniquenessRatio = 10;
            speckleWindowSize = 0;
            speckleRange = 0;
            mode = cv::StereoSGBM::MODE_SGBM;
        }
    };
    disparity_image(parameters paras);
    disparity_image();
    ~disparity_image();
    void set_left_image(cv::Mat left_image);
    void set_right_image(cv::Mat right_image);
    void set_parameters(parameters params);
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