// A class that implements some supervised image segmentation algoirthm
// to segment the garment from the background.

#ifndef __WATERSHED_SEGMENTER_H__
#define __WATERSHED_SEGMENTER_H__

#include <opencv2/opencv.hpp>
#include "ImageUtil.h"

class ObjectSegmenter{

public:
	ObjectSegmenter();
	cv::Mat ProcessGrabCut(cv::Mat &image, GarmentType type);
	cv::Mat ProcessWatershed(cv::Mat &image, GarmentType type);

	void CreateGrabCutMarker(GarmentType type);
	void CreateWatershedMarker(GarmentType type);
	void SetSrcImage(cv::Mat& src); 
		
private:
	void CreateGrabCutMarkerSweater();
	void CreateGrabCutMarkerPants();
	void CreateWatershedMarkerSweater();
	void CreateWatershedMarkerPants();

	cv::Mat src_image;
	cv::Mat markers;
};

#endif