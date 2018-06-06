#ifndef OBJECTNESS_EVALUATOR_H_INCLUDED
#define OBJECTNESS_EVALUATOR_H_INCLUDED

//headers in opencv
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>

//headers in STL
#include <vector>

class objectness_evaluator
{
public:
    objectness_evaluator();
    ~objectness_evaluator();
    std::vector<double> evaluate(cv::Mat disparity_image,cv::Mat left_image,cv::Mat right_image,std::vector<cv::Rect> rois);
};

#endif  //OBJECTNESS_EVALUATOR_H_INCLUDED