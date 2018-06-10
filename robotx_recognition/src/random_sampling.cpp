#include <random_sampling.h>
#include <random>

random_sampling::random_sampling()
{
    params_ = parameters();
}

random_sampling::random_sampling(parameters params)
{
    params_ = params;
}

random_sampling::~random_sampling()
{
    
}

std::vector<cv::Rect> random_sampling::get_rois()
{
    std::vector<cv::Rect> rects(params_.num_roi);
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> x_rand;
    std::uniform_int_distribution<int> y_rand;
    x_rand = std::uniform_int_distribution<int>(0,params_.image_width-params_.roi_width.min);
    y_rand = std::uniform_int_distribution<int>(0,params_.image_height-params_.roi_height.min);
    std::uniform_int_distribution<int> w_rand;
    std::uniform_int_distribution<int> h_rand;
    w_rand = std::uniform_int_distribution<int>(params_.roi_width.min,params_.roi_width.max);
    h_rand = std::uniform_int_distribution<int>(params_.roi_height.min,params_.roi_height.max);
    for(int i=0; i< rects.size(); i++)
    {
        int x,y,w,h;
        do
        {
            x = x_rand(mt);
            y = y_rand(mt);
            w = w_rand(mt);
            h = h_rand(mt);
            if(x+w>params_.image_width)
                w = params_.image_width - x;
            if(y+h>params_.image_width)
                h = params_.image_height - y;
        }
        while((double)h/(double)w < params_.roi_aspect_ratio.max && (double)h/(double)w > params_.roi_aspect_ratio.min);
        cv::Rect rect{x,y,w,h};
        rects.push_back(rect);
    }
    return rects;
}