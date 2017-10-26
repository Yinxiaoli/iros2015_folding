#ifndef __IMAGE_UTIL_H__
#define __IMAGE_UTIL_H__


#include <bluetail/image/imageiohdr.h>
#include <bluetail/image/image.h>
#include <iostream>
#include "datatypes.h"
#include <string>
#include <opencv2/opencv.hpp>

enum GarmentType
{
	SWEATER,
	PANTS,
	TOWEL
};

class ImageUtil 
{
public:
	ImageUtil();
	static SImage<double, 1> ConvertToSImage(char* file_name);
	static SImage<double, 1> ConvertToSImage(cv::Mat input);

	static void ImageShow(char* file_name);
	static void ImageShow(cv::Mat img);
	static void ImageShow(cv::Mat img, int sec);


	static void rotate_image_90n(cv::Mat &src, cv::Mat &dst, int angle);
};


#endif