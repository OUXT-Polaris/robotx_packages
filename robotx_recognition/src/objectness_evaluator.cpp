#include <objectness_evaluator.h>

objectness_evaluator::objectness_evaluator()
{

}

objectness_evaluator::~objectness_evaluator()
{

}

std::vector<double> objectness_evaluator::evaluate(cv::Mat disparity_image,cv::Mat left_image,cv::Mat right_image,std::vector<cv::Rect> rois)
{
    std::vector<double> result(rois.size());
    for(auto roi_itr = rois.begin(); roi_itr != rois.end(); ++roi_itr)
    {
        cv::Rect roi_rect = *roi_itr;
        cv::Mat disparity_roi = disparity_image(roi_rect);
        cv::MatND hist;
        float range[] = {0,256};
        const float* histrange = range;
        int histsize = 24;
        /*
        cv::calcHist(&disparity_roi, 1, 0, cv::Mat(), hist, 1, &histsize, &histrange);
        cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();
        em_model->setClustersNumber(2);
        em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_SPHERICAL);
        cv::Mat labels;
        em_model->trainEM(hist, cv::noArray(), labels, cv::noArray());
        cv::Mat means = em_model->getMeans();
        //cv::Mat covs = em_model->get_covs();
        */
    }
    return result;
}