#ifndef PASSTHROUGH_FILTER_H_INCLUDED
#define PASSTHROUGH_FILTER_H_INCLUDED

/**
 * @brief definition of passthrough filter class
 * 
 * @file passthrough_filter.h
 * @author Masaya Kataoka
 * @date 2018-06-10
 */

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

/**
 * @brief passthrought filter class
 * 
 */
class passthough_filter
{
public:
    /**
     * @brief parameters for passthough_filter class
     * 
     */
    struct parameters
    {
        int mode;
        double x_size;
        double y_size;
        double z_size;
        /**
         * @brief default parameters
         * 
         */
        parameters()
        {
            mode = remain;
            x_size = 0;
            y_size = 0;
            z_size = 0;
        }
    };
    /**
     * @brief Construct a new passthough filter object
     * 
     */
    passthough_filter();
    /**
     * @brief Construct a new passthough filter object
     * 
     * @param param
     * @sa passthough_filter::parameters
     */
    passthough_filter(passthough_filter::parameters& param);
    /**
     * @brief Destroy the passthough filter object
     * 
     */
    ~passthough_filter();
    const parameters param_;
private:
    /**
     * @brief operating mode
     * 
     * remain : remain pointclouds in the box
     * remove : remove pointclouds in the box
     */
    enum modes{remain,remove};
};

#endif  //PASSTHROUGH_FILTER_H_INCLUDED