#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpPoint.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpIoTools.h>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>

#include <vvs.h>
#include <grid_tracker.h>
#include <perspective_camera.h>
#include <distortion_camera.h>
#include <cb_tracker.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;
using cv::waitKey;
using namespace covis;

int main()
{
    // load calibration images from hard drive
    const string base = "../images/";
    const string prefix = "img";

    // init empty vector of detected patterns
    vector<Pattern> patterns;
    patterns.clear();
    patterns.reserve(36);



    //GridTracker tracker;      // this tracker detects a 6x6 grid of points
    CBTracker tracker(8,6);     // this one is to be given the chessboard dimension (8x6)

    vpHomogeneousMatrix M;  // camera pose

    while(true)
    {
        stringstream ss;
        ss << prefix << patterns.size() << ".jpg";
        std::ifstream testfile(base + ss.str());
        if(testfile.good())
        {
            testfile.close();
            Pattern pat;
            pat.im =  cv::imread(base + ss.str());
            tracker.detect(pat.im, pat.point);
            pat.window = ss.str();
            // draw extraction results
            drawSeq(pat.window, pat.im, pat.point);
            patterns.push_back(pat);
            waitKey(0);
        }
        else
            break;
    }
    cout << "Found " << patterns.size() << " images" << endl;

    PerspectiveCamera cam(544.6583991, 546.1633797, 319.7809807, 235.3760285);

    VVS vvs(cam, 0.03, 8, 6);

    // find pose of camera
    for(int i = 0; i < 9; ++i) {
        stringstream ss;
        ss << prefix << i << ".jpg";
        std::ifstream testfile(base + ss.str());
        if(testfile.good()) {
            testfile.close();
            Pattern pat;
            pat.im = cv::imread(base + ss.str());
            tracker.detect(pat.im, pat.point);
            pat.window = ss.str();

            // draw extraction results
            drawSeq(pat.window, pat.im, pat.point);
            waitKey(0);

            // calibrate from this single image
            if(i == 0){vvs.computePose(pat, M, true);} else{vvs.computePose(pat, M);}

        }
    }

    // this will wait for a key pressed to stop the program
    waitKey(0);
}
