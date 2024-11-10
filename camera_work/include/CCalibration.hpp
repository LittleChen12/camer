#ifndef CCALIBRATION_HPP
#define CCALIBRATION_HPP

#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <vector>


class CCalibration
{
public:
	CCalibration(cv::Size _board_sz, double _board_dt, int _n_boards = 15);
	~CCalibration(){};
 
public:
    bool doCalibrate(const cv::Mat& image_points, const cv::Mat& object_points, const cv::Mat& point_counts, const cv::Size& size);
	bool calibrateFromCamera();
	bool calibrateFromFile();
	void display();
 
protected:
 
private:
	cv::Size board_sz; //标定板信息
	int n_boards;    //视场总数
	double board_dt; //相邻视场间的获取时间间隔
 
private:
    cv::Mat intrinsic_matrix = cv::Mat::eye(3, 3, CV_32F);//内参数矩阵
    cv::Mat distortion_coeffs = cv::Mat::zeros(5, 1, CV_32F);//畸变矩阵
 
};

#endif