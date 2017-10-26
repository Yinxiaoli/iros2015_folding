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

    void GenerateGarmentMask(char* fileName, GarmentType type);
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
	cv::Mat SegResult;
	cv::Mat garmentMask;

	cv::Mat img_load;
	vector<keyPoint> pointList;

	ObjectSegmenter* mObjectSegmenter;
};


#endif