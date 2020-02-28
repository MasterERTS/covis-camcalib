#ifndef FIND_DOTS_H
#define FIND_DOTS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace std;
using namespace cv;

struct dot
{
    double x_, y_, a_;
    dot(double _x, double _y, double _a)
    {
        x_ = _x;
        y_ = _y;
        a_ = _a;
    }
    double Dist(const double &_x, const double &_y, const double &_a)
    {
        //cout << "area ratio: " << a_/_a << endl;
        if(a_/_a < 0.5)
            return -1;
        else if(a_/_a > 1.1)
            return 0;

        return (_x-x_)*(_x-x_) + (_y-y_)*(_y-y_) + 0*(_a-a_)*(_a-a_);
    }
};


// This function takes an image and returns the position (cog) of the 36 ellipses of interest
bool FindDots(cv::Mat _im, std::vector<cv::Point> &_cog)
{

    // to grey level + blur
    cv::Mat img;
    cv::cvtColor(_im, img, cv::COLOR_BGR2GRAY);

    // cv::GaussianBlur(img, img, Size(7, 7), 1, 1);
    cv::Mat imth;
    cv::Canny(img, imth, 20, 80, 3);


    // contours
    vector<vector<Point> > contours_tmp, contours;
    vector<Vec4i> hierarchy;
    //  cv::namedWindow("Canny");
    // cv::drawContours(imth, contours_tmp, -1, cv::Scalar(255), 2);
    cv::findContours(imth, contours_tmp, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    //   cv::imshow("Canny", imth);

    // cv::waitKey(0);
    cout << "  - found " << contours_tmp.size() << " contours" << endl;
    // hierarchy to detect doublons
    unsigned int count = 0;
    vector<dot> dots;
    Moments mu;
    cv::RotatedRect rect;
    unsigned int i;

    for (i=0; i<contours_tmp.size(); i++)
        if (hierarchy[i][3] >= 0)   //has parent, inner (hole) contour of a closed edge (looks good)
        {
            mu = moments(contours_tmp[i], false);
            rect = cv::minAreaRect(contours_tmp[i]);
            //cout << "Ellipse ratio " << abs(mu.m00*4/(M_PI*rect.size.area())-1) << endl;
            if(mu.m00 > 0 && abs(mu.m00*4/(M_PI*rect.size.area())-1) < 0.1)
            {
                stringstream ss;
                ss << dots.size();
                dots.push_back(dot(mu.m10/mu.m00, mu.m01/mu.m00, mu.m00));
                contours.push_back(contours_tmp[i]);
                cv::putText(_im, ss.str(), cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00), 0, 0.5, cv::Scalar(0,0,255), 2);
                cv::drawContours(_im, contours_tmp, i, Scalar(255, 0, 0), 1, 8);
                count++;
            }
        }
    cout << "  - eliminating outer/non-ellipsoids -> " << count << endl;
    cv::Mat imbkp;_im.copyTo(imbkp);

    // 36 largest for first guess
    vector<unsigned int> candidates, others;
    unsigned int idx_small = 0;
    for(i=0;i<contours.size();++i)
    {
        if(i < 36)
        {
            candidates.push_back(i);
            if(dots[i].a_ < dots[idx_small].a_)
                idx_small = i;
        }
        else
        {
            if(dots[i].a_ > dots[idx_small].a_)
            {
                others.push_back(idx_small);
                *(std::find(candidates.begin(), candidates.end(),idx_small)) = i;
                idx_small = i;
                for(unsigned int j=0;j<candidates.size();++j)
                {
                    if(dots[candidates[j]].a_ < dots[idx_small].a_)
                        idx_small = candidates[j];
                }
            }
        }
        /*
        if(dots[i].a_ > smallest)

        if(i < 36)

        else
            others.push_back(i);*/
    }
    if(candidates.size()<36)
    {
        cout << "  -> Not enough candidates to find grid (" << candidates.size() << ")" << endl;
        return true;
    }
    else if(contours.size() >=36)   // too much contours, have to check nearest ones
    {
        bool update = true;

        while(update)
        {
            imbkp.copyTo(_im);
            update = false;
            // mean of current candidates
            double x=0, y=0, a=0;
            for(i=0;i<36;++i)
            {
                cv::drawContours(_im, contours, candidates[i], Scalar(0, 255, 0), 1, 8);
                x += dots[candidates[i]].x_;
                y += dots[candidates[i]].y_;
                a += dots[candidates[i]].a_;
            }
            x *= 1./36;
            y *= 1./36;
            a *= 1./36;
            // max candidate distance
            double d = 0, d_max = 0;
            unsigned int idx_max;
            for(i=0;i<36;++i)
            {
                d = dots[candidates[i]].Dist(x,y,a);
                //   cout << "Dist for " << candidates[i] << ": " << d << endl;
                if(d > d_max && d != -1)
                {
                    d_max = d;
                    idx_max = i;
                }
            }
            //    cout << "Max candidate distance: " << d_max << " for " << candidates[idx_max] << endl;
            // min other distance
            unsigned int idx_min;
            for(i=0;i<others.size();++i)
            {
                d = dots[others[i]].Dist(x,y,a);
                if(d < d_max && d != -1)
                {
                    d_max = d;
                    idx_min = i;
                    update = true;
                }
            }
            //   cout << "Min other distance: " << d_max << " for " << others[idx_min] << endl;

            // switch
            if(update)
            {
                //     cout << "Replacing " << candidates[idx_max] << " with " << others[idx_min] << endl;
                cv::drawContours(_im, contours, candidates[idx_max], Scalar(0, 0, 255), 2, 8);
                cv::drawContours(_im, contours, others[idx_min], Scalar(0, 255, 0), 2, 8);
                i = candidates[idx_max];
                candidates[idx_max] = others[idx_min];
                others[idx_min] = i;
            }
            cv::circle(_im, cv::Point2f(x,y), int(sqrt(a/M_PI)), cv::Scalar(0,255,0), 2);
            // cv::imshow("raw", _im);
            // cv::waitKey(0);
        }
    }
    cout << "  - kept only 36" << endl;

    // keep only candidates
    _cog.clear();
    for(i=0;i<36;++i)
        _cog.push_back(cv::Point(int(dots[candidates[i]].x_), int(dots[candidates[i]].y_)));

    return false;
}

// utility function
float EuclideanDist(const cv::Point& _p, const cv::Point& _q) {
    cv::Point diff = _p - _q;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}

// utility function to draw current dot sequence
void DrawSeq(const std::string &_window, cv::Mat &_im, const std::vector<cv::Point> &_X, std::vector<unsigned int> _idx = vector<unsigned int>())
{
    cv::Mat imdraw;
    _im.copyTo(imdraw);
    _idx.push_back(0);

    cv::circle(imdraw, _X[_idx[0]], 2, cv::Scalar(0,255,0),2);

    for(unsigned int i=1;i<_X.size();++i)
    {
        _idx.push_back(i);
        cv::circle(imdraw, _X[_idx[i]], 2, cv::Scalar(0,255,0),2);
        cv::line(imdraw, _X[_idx[i-1]], _X[_idx[i]], cv::Scalar(0,0,255), 2);
    }

    cv::imshow(_window, imdraw);
    //cv::waitKey(0);
}



#endif // FIND_DOTS_H
