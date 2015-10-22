// A class that implements functions for image pre-processing.

#ifndef __IMAGE_PREPROCESSOR_H__
#define __IMAGE_PREPROCESSOR_H__

#include "ImageUtil.h"
#include "ObjectSegmenter.h"
#include <vector>
#include <stdio.h>

#include <string>
#include <opencv2/opencv.hpp>

struct keyPoint  
{
	int id;
	int x;
	int y;
};

class ImagePreprocessor
{
public:
	ImagePreprocessor();

    void GenerateGarmentMask(char* file_name, GarmentType type);
	cv::Mat GetGarmentMask();
	cv::Rect GetROI();
	void RescalePoints(const SCurve* ptId, const SVar& ptPos);

	void TestPoint(int x, int y);
	void TestPoints(std::vector<keyPoint> ptList);
	vector<keyPoint>* GetPointList();

private:
	void CreateMarker(cv::Mat& marker);
	void CropRectROI(char* fileName);
	void SegmentObject(GarmentType type);
	cv::Mat CreateSquareGarmentMask();

	void WritePointToFile();

	
	int maskSize;

	cv::Mat marker;
	cv::Mat img_cropped;
	cv::Rect roi;
	cv::Mat seg_result;
	cv::Mat garment_mask;

	cv::Mat img_load;
	vector<keyPoint> point_list;

	ObjectSegmenter* mObjectSegmenter;
};


#endif