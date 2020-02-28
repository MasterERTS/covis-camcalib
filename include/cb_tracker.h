#ifndef CBTRACKER_H
#define CBTRACKER_H

#include <opencv2/calib3d/calib3d.hpp>

namespace covis
{

class CBTracker
{
public:

    cv::Size size_;
    std::vector<cv::Point> cog_prev_;

    CBTracker(unsigned int r, unsigned int c);

    bool detect(cv::Mat &_im, std::vector<cv::Point> &_cog);
    bool track(cv::Mat &_im, std::vector<cv::Point> &_cog);

private:
    inline double sqdist(cv::Point &p1, cv::Point &p2)
    {
        return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
    }
};

}

#endif // CBTRACKER_H
