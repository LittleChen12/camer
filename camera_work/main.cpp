// #include <opencv2/core.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>

// #include "include/CCalibration.hpp"
// using namespace cv;
// using namespace std;
int main(int, char**)
{
    // CCalibration calib(cv::Size(13,10),1,10);
 
	// //从相机中获取图像标定
	// //calib.calibrateFromCamera();
 
	// //从已有图像中标定
	// calib.calibrateFromFile();
	
	//运用标定结果显示修正后图像
	//calib.display();
    std::cout << "Press Enter to continue...";
    std::cin.get();

    return 0;
}