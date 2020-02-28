#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpPoint.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpIoTools.h>

#include <cam_models.h>
#include <grid_tracker.h>
#include <vvs.h>

using namespace std;
using namespace cv;


void OnMouseSelect(int evt, int x, int y, int flags, void* click) {
    if(evt == cv::EVENT_LBUTTONDBLCLK)
    {
        int *c = (int*) click;
        *c = evt;
    }
}

int main()
{
    cv::VideoCapture cap(0);
    vector<cv::Mat> im(1);
    cv::Mat imtmp;
    vector<string> window(1);
    window[0] = "Camera";
    vector<vector<cv::Point> >cog(1);

    // load image for warping
    cv::Mat im_ecn = cv::imread("../ecn.jpg");

    vector<cv::Point2f> warp1(4), warp0;
    warp0.push_back(cv::Point(0,0));
    warp0.push_back(cv::Point(im_ecn.cols,0));
    warp0.push_back(cv::Point(im_ecn.cols,im_ecn.rows));
    warp0.push_back(cv::Point(0,im_ecn.rows));

    vector<vpPoint> W(4);
    W[2].setWorldCoordinates(-0.105, -0.105, 0);
    W[3].setWorldCoordinates(0.105, -0.105, 0);
    W[0].setWorldCoordinates(0.105,0.105,  0);
    W[1].setWorldCoordinates(-0.105, 0.105, 0);

    // minim...
    // HP laptop cam + VVS
    DistortionCamera cam(646.7696914,  643.7703183 , 305.4480097, 242.7912928 , 0.00594527338 , 1.001051828, true);
    //PerspectiveCamera cam(650.9187166,  648.3355392,  309.6989075,  242.0794704, true);
    VVS vvs(0.0322);
    vvs.cam(cam);

    // display
    vvs.SetDisplay(window, im, true);
    cv::namedWindow(window[0]);
    int clic_evt = cv::EVENT_FLAG_ALTKEY;
    cv::setMouseCallback(window[0], OnMouseSelect, (void*)&clic_evt);

    bool first = true;

    clic_evt = cv::EVENT_FLAG_ALTKEY;
    // tracker
    GridTracker tracker;

    unsigned int iter = 0;
    bool reset = false;
    vpHomogeneousMatrix M0;
    vpPoseVector pose;    
    while(cap.isOpened() && clic_evt == cv::EVENT_FLAG_ALTKEY)
    {
        cap.read(im[0]);
        im[0].copyTo(imtmp);
        cv::imshow(window[0], im[0]);

        if(first)
        {
            tracker.Detect(im[0], cog[0]);
            vvs.Calib(cog, reset);
            M0 = vvs.M_[0];
            first = false;
        }
        else if(tracker.Track(im[0], cog[0]))
        {
            vvs.Calib(cog, reset);
            pose.buildFrom(M0.inverse()*vvs.M_[0]);
            reset = vvs.display_ = (pose.getTranslationVector().euclideanNorm() > 0.01);
            M0 = vvs.M_[0];
        }

        cout << "Reset: " << reset << endl;

        if(!reset)
        {

        // trying warp
        double u,v;
        cv::Mat im_warp, im_warp_g, mask, mask_inv, im_bg, warp_fg, im_all;
        for(unsigned int i=0;i<4;++i)
        {
            W[i].track(vvs.M_[0]);
            cam.Project(W[i], u,v);
            warp1[i] = cv::Point2f(u,v);
        }
        cv::warpPerspective(im_ecn, im_warp, cv::getPerspectiveTransform(warp0, warp1),cv::Size(im[0].cols, im[0].rows));
        //cv::imshow("Warp", im_warp);
        cv::cvtColor(im_warp, im_warp_g, cv::COLOR_BGR2GRAY);
        cv::threshold(im_warp_g, mask, 10, 255, cv::THRESH_BINARY);
        cv::bitwise_not(mask, mask_inv);
        cv::bitwise_and(im[0], im[0],im_bg, mask_inv);
        cv::bitwise_and(im_warp, im_warp,warp_fg, mask);
        cv::add(im_bg, im_warp, im_all);

        /*
        for(unsigned int i=0;i<im[0].rows;++i)
        {
            for(unsigned int j=0;j<im[0].cols;++j)
            {

                cout << im_warp_g.at<unsigned int>(i,j) << endl;
                if(im_warp_g.at<unsigned int>(i,j))
                    im[0].at<cv::Vec3b>(i,j) = im_warp.at<cv::Vec3b>(i,j);
            }
        }*/        
        cv::imshow("Warp", im_all);
        }

        vvs.Display(1);
    }
}


