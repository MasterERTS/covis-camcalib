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

int main()
{
    string base = "../images/hp";

    unsigned int n_im = 6;
    // load images
    vector<cv::Mat> im(n_im);
    vector<string> window(n_im);
    vector<vector<cv::Point> > cog(n_im);
    unsigned int k=0;
    GridTracker tracker;
    for(unsigned int i=0;i<n_im;++i)
    {
        //     if(i == 2)  // to load only a given image
        {
            cv::Mat im_k;
            vector<cv::Point> cog_k;
            std::stringstream ss;
            ss << base << i << ".jpg";
            im_k = cv::imread(ss.str());

            tracker.Detect(im_k, cog_k);

            DrawSeq(ss.str(), im_k, cog_k);
            cv::waitKey(0);
            im_k.copyTo(im[k]);
            cog[k] = cog_k;
            window[k] = ss.str();
            k++;
        }
    }

    // print + update number of success
    cout << k << "/" << n_im << " images successfully loaded" << endl;
    n_im = k;

    // initiate tracker / calibrator
    const bool calibrated = false;
    VVS vvs(0.0322);

    // minim...
    // camera model with default parameters
    const double pxy = 0.5*(im[0].rows+im[0].cols);
     PerspectiveCamera cam(pxy, pxy, 0.5*im[0].cols, 0.5*im[0].rows);
    //DistortionCamera cam(pxy, pxy, 0.5*im[0].cols, 0.5*im[0].rows, 0, 1);
    /*if(calibrated)
        cam = DistortionCamera(922.344, 916.79, 311.504, 245.911, 0.117207, 4.00827, calibrated);*/
    vvs.cam(cam);

    // display during minimization
    vvs.SetDisplay(window, im, true);

    // calib from all images
    vvs.Calib(cog);

    // display error image
    cv::Mat imerr(im[0].rows, im[0].cols, im[0].type(), cv::Scalar(255,255,255));
    const unsigned int L = 50;
    int hue,col;
    cv::Scalar cmap;
    unsigned int u,v,ud,vd, row;

    for(unsigned int n=0;n<n_im;++n)
    {
        // cv::line(imtp, cog[n][0], cog[n][5], cv::Scalar(0,255,0), 2);
        for(unsigned int i=0;i<36;++i)
        {
            // vvs.X_[i].track(vvs.M_[n]);
            // corresponding pixels
            row = 2*i+72*n;
            // cam.Project(vvs.X_[i], vvs.s_[row], vvs.sd_[row+1]);

            u = int(vvs.s_[row]);
            ud = int(vvs.sd_[row]);
            v = int(vvs.s_[row+1]);
            vd = int(vvs.sd_[row+1]);

            hue = int(2*(1-sqrt(vpMath::sqr(u-ud) +vpMath::sqr(v-vd))));
            col = 1-abs(hue%2-1);
            if(hue < 1)
                cmap = cv::Scalar(0,int(col*255), 255);
            else
                cmap = cv::Scalar(0,255, int(col*255));
            cv::circle(imerr, cv::Point(ud,vd),1, cmap, 2);
        }
        cv::imshow("error", imerr);
    }


    // print best results
    cout << "Intrinsic: " << cam.xi_.t() << endl;
    //cout << "Max reprojection error: " << err_best << " pixels" << endl;



    // save to files
    vpIoTools::setBaseDir("../results/");

    // intrinsic parameters
    if(cam.NbParam() == 4)
    {
        vpMatrix::saveMatrixYAML("../results/p_intrinsic.txt", vvs.xi_hist_, "dataType: iteration-based\nlegend: ['p_x', 'p_y', 'u_0', 'v_0']\n");
        // pixel error
        vpMatrix::saveMatrixYAML("../results/p_error.txt", vvs.err_hist_, "dataType: iteration-based\n");
    }
    else
    {
        vpMatrix::saveMatrixYAML("../results/d_intrinsic.txt", vvs.xi_hist_, "dataType: iteration-based\nlegend: ['p_x', 'p_y', 'u_0', 'v_0', '\\alpha', '\\beta']\n");
        // pixel error
        vpMatrix::saveMatrixYAML("../results/d_error.txt", vvs.err_hist_, "dataType: iteration-based\n");
    }

    // display results
    vvs.Display();




}
