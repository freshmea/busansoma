#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int main()
{
    Mat img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    cout << "Hello, world!\n"
         << endl;
    rectangle(img, Point(100, 100), Point(400, 400), Scalar(255, 255, 255), 5);
    imshow("Image", img);
    waitKey(0);
    return 0;
}