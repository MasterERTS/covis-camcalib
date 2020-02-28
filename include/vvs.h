#ifndef VVS_H
#define VVS_H

#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpExponentialMap.h>
#include <generic_camera.h>

namespace covis
{

// structure used to regroup data
struct Pattern
{
    cv::Mat im;                     // OpenCV image
    std::vector<cv::Point> point;   // extracted points
    std::string window;             // window to display this image
};

class VVS
{
public:
    // This constructor builds the 3D points belonging to the calibration grid
    VVS(GenericCamera &_cam, const double &_d, const unsigned int r = 6, const unsigned int c = 6)
    {
        r_ = r;
        c_ = c;
        X_.clear();
        // build 3D points
        for(unsigned int j=0;j<c;++j)
            for(unsigned int i=0;i<r;++i)
                X_.push_back(vpPoint((i-(r/2.-0.5))*_d,(j-(c/2.-0.5))*_d,0));

        // grid borders
        unsigned int nbcontour = (r+c+2)*2+1;
        B_.resize(nbcontour);
        B_[0].setWorldCoordinates(-(r/2.+0.5)*_d, -(c/2.+0.5)*_d, 0);
        double dx, dy;
        for(unsigned int i=1;i<nbcontour-1;++i)
        {
            // update direction
            if(sgn(fabs(B_[i-1].get_oX())-(r/2.+0.5)*_d, _d/2) == 0 &&
                    sgn(fabs(B_[i-1].get_oY())-(c/2.+0.5)*_d, _d/2) == 0)    // corner
            {
                dx = -_d*sgn(sgn(B_[i-1].get_oX())+sgn(B_[i-1].get_oY()),0.5);
                dy = _d*sgn(sgn(B_[i-1].get_oX())-sgn(B_[i-1].get_oY()),0.5);
            }
            B_[i].setWorldCoordinates(B_[i-1].get_oX()+dx, B_[i-1].get_oY()+dy, 0);
        }
        B_[nbcontour-1] = B_[0];
        Bpx_.resize(nbcontour);

        // XYZ frame
        F_.resize(4);
        F_[0].setWorldCoordinates(0, 0, 0);
        F_[1].setWorldCoordinates(-3*_d, 0, 0);
        F_[2].setWorldCoordinates(0, 3*_d, 0);
        F_[3].setWorldCoordinates(0, 0, -3*_d);

        //lambda_ = vpAdaptiveGain(2, 0.1,0.1);
        lambda_ = 0.5;
        display_ = false;

        // set the camera
        cam_ = &_cam;
    }


    // display projected grid vs real one, assumes points have been projected in _s
    void display(const std::vector<Pattern> &_pat, const std::vector<vpHomogeneousMatrix> &_M, const vpColVector &_s, const unsigned int &_wait = 0)
    {
        unsigned int row, n=0;
        Pattern t;
        for(auto &pat : _pat)
        {
            pat.im.copyTo(imtp_);
            // display all points (desired, current estimation and line between the two)
            for(unsigned int i=0;i<X_.size();++i)
            {
                row = 2*i+2*r_*c_*n;
                cv::circle(imtp_, cv::Point(int(_s[row]), int(_s[row+1])), 1, cv::Scalar(0,0,255), 2);
                cv::circle(imtp_, pat.point[i], 1, cv::Scalar(0,255,0), 2);
                cv::line(imtp_, cv::Point(int(_s[row]), int(_s[row+1])),
                        pat.point[i], cv::Scalar(0,255,0), 1);
            }

            // projection of borders
            B_[0].track(_M[n]);
            cam_->project(B_[0], Bpx_[0].x, Bpx_[0].y);
            for(unsigned int i=1;i<(r_+c_+2)*2+1;++i)
            {
                B_[i].track(_M[n]);
                cam_->project(B_[i], Bpx_[i].x, Bpx_[i].y);
                cv::circle(imtp_, cv::Point(int(Bpx_[i].x), int(Bpx_[i].y)), 1, cv::Scalar(0,0,255), 2);
                cv::line(imtp_,
                         cv::Point(int(Bpx_[i].x), int(Bpx_[i].y)),
                         cv::Point(int(Bpx_[i-1].x), int(Bpx_[i-1].y)),
                        cv::Scalar(0,0,255), 2);
            }

            // projection of XYZ frame
            for(unsigned int i=0;i<4;++i)
                F_[i].track(_M[n]);
            double u,v;
            cam_->project(F_[0], u,v);
            cv::Point F0 = cv::Point(int(u), int(v));
            cam_->project(F_[1], u,v); cv::line(imtp_,F0,cv::Point(int(u),int(v)),cv::Scalar(0,0,255),2);            // X
            cam_->project(F_[2], u,v); cv::line(imtp_,F0,cv::Point(int(u),int(v)),cv::Scalar(0,255,0),2);            // Y
            cam_->project(F_[3], u,v); cv::line(imtp_,F0,cv::Point(int(u),int(v)),cv::Scalar(255,0,0),2);            // Z

            // display
            cv::imshow(pat.window, imtp_);
            n++;
        }
        cv::waitKey(_wait);
    }

    // display function for only one image
    inline void display(const Pattern &_pat, const vpHomogeneousMatrix &_M, const vpColVector &_s, const unsigned int &_wait = 0)
    {
        std::vector<Pattern> pat = {_pat};
        std::vector<vpHomogeneousMatrix> M = {_M};
        display(pat, M, _s, _wait);
    }

    void calibrate(std::vector<Pattern> &_pat);
    void computePose(Pattern &_pat, vpHomogeneousMatrix &_M, const bool &_reset = false);


protected:
    // get the sign of a value
    inline double sgn(const double x, const double approx = 0)
    {
        if(fabs(x) < approx) return 0;
        return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
    }

    // put a sub-matrix inside a given matrix
    void putAt(vpMatrix &_M, const vpMatrix &_Msub, const unsigned int &_r, const unsigned int &_c)
    {
        if(_Msub.getRows() + _r > _M.getRows())
        {
            std::cout << "VVS::putAt: rows do not fit" << std::endl;
        }
        else if(_Msub.getCols() + _c > _M.getCols())
        {
            std::cout << "VVS::putAt: columns do not fit" << std::endl;
        }
        else
        {
            vpSubMatrix m(_M, _r, _c, _Msub.getRows(), _Msub.getCols());
            m = _Msub;
        }
    }

    // update an homogeneous matrix with a given velocity screw
    inline void updatePosition(vpHomogeneousMatrix &_M, const vpColVector &_v)
    {
        if(_v.getRows() != 6)
            std::cout << "VVS::updatePosition: wrong number of rows for v (" << _v.getRows() << ")" << std::endl;
        else
            _M = vpExponentialMap::direct(_v).inverse() * _M;
    }



    // 3D points + borders + frame
    unsigned int r_, c_;
    std::vector<vpPoint> X_, B_, F_;
    std::vector<cv::Point2d> Bpx_;

    // used camera and markers
    GenericCamera* cam_;
    double d_;

    // if we display
    bool display_;
    cv::Mat imtp_;

    // optimization variables
    double lambda_;
    vpHomogeneousMatrix M0_;

    // history for plots
    vpMatrix xi_hist_, err_hist_;
};

}

#endif // VVS_H
