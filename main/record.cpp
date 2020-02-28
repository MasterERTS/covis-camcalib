#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;

const int EVT_SAVE = cv::EVENT_LBUTTONDOWN;
const int EVT_EXIT = cv::EVENT_RBUTTONDOWN;


void OnMouseSelect(int evt, int x, int y, int flags, void* click)
{
    if(evt == EVT_SAVE || evt == EVT_EXIT)
    {
        int *c = (int*) click;
        *c = evt;
    }
}

int main()
{
    cv::VideoCapture cap(0);
    cv::Mat im;
    string window = "Camera";
    string prefix = "img";

    // display
    cv::namedWindow(window);
    int clic_evt = cv::EVENT_FLAG_ALTKEY;
    cv::setMouseCallback(window, OnMouseSelect, (void*)&clic_evt);

    // get starting number
    bool found = true;
    int idx = -1;
    while(found)
    {
        idx++;
        stringstream ss;
        ss << "../images/" << prefix << idx << ".jpg";
        std::ifstream testfile(ss.str());
        found = testfile.good();
    }
    cout << "Begin at " << idx << endl;

    std::cout << "Left click to save image, right click to exit" << std::endl;

    // double click saves image, right click exits
    while(cap.isOpened() && clic_evt != EVT_EXIT)
    {
        cap >> im;
        cv::imshow(window, im);

        if(clic_evt == EVT_SAVE)
        {
            stringstream ss;
            ss << "../images/" << prefix << idx++ << ".jpg";
            cv::imwrite(ss.str(), im);
            cout << "Saving " << ss.str() << endl;
            clic_evt = cv::EVENT_FLAG_ALTKEY;
        }

        cv::waitKey(1);
    }
}
