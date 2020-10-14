#pragma once
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <k4a/k4a.hpp>

#include <fstream>
#include <experimental/filesystem>
#include <vector>
#include <Windows.h>

using namespace std;
typedef std::vector <std::string> stringmat;

class SensorData
{
public:

	SensorData();
	SensorData(const cv::Mat& rgb, const cv::Mat & depth, int id = 0, double stamp = 0.0);

	cv::Mat _rgb;
	cv::Mat _depth;
	int SeqID;

};

class ImageReader
{
	public:
		int i = 0;
		stringstream num;
		string colorpath;
		string depthpath;
		stringmat v;

		cv::Mat bgrCV;
		cv::Mat depthCV;

		void init();
		SensorData ImageRead();

	private:
};

