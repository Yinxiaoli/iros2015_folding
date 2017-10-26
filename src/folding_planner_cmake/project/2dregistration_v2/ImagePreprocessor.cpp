#include "ImagePreprocessor.h"

using namespace std;

ImagePreprocessor::ImagePreprocessor()
{
	// Initialized roi with fixed camera pose.
	this->roi = cv::Rect(0, 100, 880, 430);
	maskSize = 1000;

	garmentMask.create(cv::Size(maskSize, maskSize), CV_8U);
	garmentMask.setTo(cv::Scalar(255));

	mObjectSegmenter = new ObjectSegmenter();
}

void ImagePreprocessor::CropRectROI(char* fileName)
{
	img_load = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);
	cv::Size size(960, 540);
	cv::resize(img_load, img_load, size);
	//ImageUtil::ImageShow(img_load);
	img_cropped = img_load(this->roi);
	//ImageUtil::ImageShow(img_cropped);
}

 void ImagePreprocessor::SegmentObject(GarmentType type)
 {
	mObjectSegmenter->SetSrcImage(img_cropped);

	//ImageUtil::ImageShow(img_cropped);
	SegResult = mObjectSegmenter->ProcessWatershed(img_cropped, type);
	 
	 //printf("I am after segmentation.");
	 //ImageUtil::ImageShow(SegResult);
 }

 void ImagePreprocessor::GenerateGarmentMask(char* fileName, GarmentType type)
 {
	 // Crop original image.
	 CropRectROI(fileName);

	 // Segment the garment from the background using Watershed segmentation.
	 SegmentObject(type);

	 // Generate binary garment mask.
	 cv::threshold(SegResult, SegResult, 200, 255, cv::THRESH_BINARY_INV);
	 //ImageUtil::ImageShow(SegResult);
	 cv::dilate(SegResult, SegResult, cv::Mat(), cv::Point(-1, -1), 9, 1, 1);
	 //ImageUtil::ImageShow(SegResult);

	 // Generate a 1000x1000 mask.
     garmentMask = CreateSquareGarmentMask();
 }

 cv::Mat ImagePreprocessor::CreateSquareGarmentMask()
 {
	 int topLeft_x = maskSize/2 - SegResult.cols/2;
	 int topLeft_y = maskSize/2 - SegResult.rows/2;
	 cv::Rect roi = cv::Rect(topLeft_x, topLeft_y, SegResult.cols, SegResult.rows);
	 
	 SegResult.copyTo(garmentMask(roi));
	 //ImageUtil::ImageShow(garmentMask);

	 return garmentMask;
 }

 cv::Mat ImagePreprocessor::GetGarmentMask()
 {
	 return this->garmentMask;
 }

 cv::Rect ImagePreprocessor::GetROI()
 {
	 return this->roi;
 }

 void ImagePreprocessor::RescalePoints(const SCurve* ptId, const SVar& ptPos)
 {
	 float x;
	 float y;

	 for(int i=0; i<ptId->nVertices; i++)
	 {
		 if(ptId->vertexIDs(i) >= 0)
		 {
			 x = ptPos.pos.col(i).x();
			 y = ptPos.pos.col(i).y();
			 printf("xxxx, yyyy, %f, %f\n", x, y);
			 printf("roi_y %f \n", (float)this->roi.y);
			 x = x / 2.0f * (float)(maskSize/2) + (float)this->roi.width/2 + (float)this->roi.x;
			 y = (float)(this->roi.height/2) - y / 2.0f * (float)(maskSize/2) + (float)this->roi.y;
			 printf("test x y... x = %f, y = %f \n", x, y);

			 keyPoint keyPt;
			 keyPt.id = ptId->vertexIDs(i);
			 keyPt.x = (int)x;
			 keyPt.y = (int)y;
			 
			 pointList.push_back(keyPt);
		 }
	 }
	 for (int i = 0; i < pointList.size(); i ++)
	 {
		 printf("rescaled point: id= %d;  %d, %d \n", pointList[i].id, pointList[i].x, pointList[i].y);
	 }

	 // Draw points on the original image.
	 TestPoints(pointList);
	 WritePointToFile();
 }

 void ImagePreprocessor::WritePointToFile()
 {
	 //Remove existing keypoints file.
	 remove("keypoints.txt");
	 
	 FILE *fp = fopen("keypoints.txt", "w");

	 if (fp == NULL)
	 {
		 printf("Error opening file!\n");
		 exit(1);
	 }
	 for (int i = 0; i < pointList.size(); i++)
	 {
		 fprintf(fp, "%d %d %d \n", pointList[i].id, pointList[i].x, pointList[i].y);
	 }

	 fclose(fp);
 }


 void ImagePreprocessor::TestPoint(int x, int y)
 {
	 for (int i = x-3; i < x+3; i++)
		 for (int j = y-3; j < y+3; j ++)
		 {
			 img_load.at<cv::Vec3b>(i,j) = cv::Vec3b(0, 255, 255);
		 }
		 ImageUtil::ImageShow(img_load, 3);
 }

 void ImagePreprocessor::TestPoints(std::vector<keyPoint> ptList)
 {
	 for (int k = 0; k < ptList.size(); k++)
	 {
		 // Switch x, y for OpenCV drawing.
		 int x = ptList[k].y;
		 int y = ptList[k].x;
		 for (int i = x-6; i < x+6; i++)
			 for (int j = y-6; j < y+6; j ++)
			 {
				 img_load.at<cv::Vec3b>(i,j) = cv::Vec3b(0, 0, 255);
			 }
	 }
	 ImageUtil::ImageShow(img_load, 3);
 }

 vector<keyPoint>* ImagePreprocessor::GetPointList()
 {
	 printf("Get function----The size is %d\n", pointList.size());
	 return &pointList;
 }