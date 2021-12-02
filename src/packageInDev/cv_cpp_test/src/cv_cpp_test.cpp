#include "opencv2/opencv.hpp"
#include <iostream>

int main()
{
    cv::Mat frame;

    cv::VideoCapture cap;
    cap.open("/dev/video0");    // запускает стриминг с камеры под индексом 0
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FPS, 60);
    
    while(true)
    {
        cap.read(frame);

        cv::imshow("this is you, smile! :)", frame);

        if( cv::waitKey(10) == 27 ) 
            break; // stop capturing by pressing ESC 
    }
    return 0;
}