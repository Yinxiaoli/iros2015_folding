#include "ImagePreprocessor.h"

using namespace std;

ImagePreprocessor::ImagePreprocessor()
{
	// Initialized roi with fixed camera pose.
	this->roi = cv::Rect(0, 100, 600, 300);
	maskSize = 800;

	garment_mask.create(cv::Size(maskSize, maskSize), CV_8U);
	garment_mask.setTo(cv::Scalar(255));

	mObjectSegmenter = new ObjectSegmenter();
}

void ImagePreprocessor::CropRectROI(char* file_name)
{
	img_load = cv::imread(file_name, CV_LOAD_IMAGE_COLOR);
	//ImageUtil::ImageShow(img_load);
	img_cropped = img_load(this->roi);
}

 void ImagePreprocessor::SegmentObject(GarmentType type)
 {
	mObjectSegmenter->SetSrcImage(img_cropped);

	//ImageUtil::ImageShow(img_cropped);
	seg_result = mObjectSegmenter->ProcessWatershed(img_cropped, type);
	 
	 //printf("I am after segmentation.");
	 //ImageUtil::ImageShow(SegResult);
 }

 void ImagePreprocessor::GenerateGarmentMask(char* file_name, GarmentType type)
 {
	 // Crop original image.
	 CropRectROI(file_name);

	 // Segment the garment from the background using Watershed segmentation.
	 SegmentObject(type);

	 // Generate binary garment mask.
	 cv::threshold(seg_result, seg_result, 200, 255, cv::THRESH_BINARY_INV);
	 //ImageUtil::ImageShow(SegResult);
	 //cv::dilate(SegResult, SegResult, cv::Mat(), cv::Point(-1, -1), 9, 1, 1);
	 //ImageUtil::ImageShow(SegResult);

	 // Generate a 800x800 mask.
     garment_mask = CreateSquareGarmentMask();
 }

 cv::Mat ImagePreprocessor::CreateSquareGarmentMask()
 {
	 int topLeft_x = maskSize/2 - seg_result.cols/2;
	 int topLeft_y = maskSize/2 - seg_result.rows/2;
	 cv::Rect roi = cv::Rect(topLeft_x, topLeft_y, seg_result.cols, seg_result.rows);
	 
	 seg_result.copyTo(garment_mask(roi));
	 //ImageUtil::ImageShow(garmentMask);

	 return garment_mask;
 }

 cv::Mat ImagePreprocessor::GetGarmentMask()
 {
	 return this->garment_mask;
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
			 x = x / 2.0f * (float)(maskSize/2) + (float)this->roi.width/2;
			 y = (float)(this->roi.height/2) - y / 2.0f * (float)(maskSize/2) + (float)this->roi.y;
			 printf("test x y... x = %f, y = %f \n", x, y);

			 keyPoint keyPt;
			 keyPt.id = ptId->vertexIDs(i);
			 keyPt.x = (int)x;
			 keyPt.y = (int)y;
			 
			 point_list.push_back(keyPt);
		 }
	 }
	 for (int i = 0; i < point_list.size(); i ++)
	 {
		 printf("rescaled point: id= %d;  %d, %d \n", point_list[i].id, point_list[i].x, point_list[i].y);
	 }

	 // Draw points on the original image.
	 TestPoints(point_list);
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
	 for (int i = 0; i < point_list.size(); i++)
	 {
		 fprintf(fp, "%d %d %d \n", point_list[i].id, point_list[i].x, point_list[i].y);
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

 void ImagePreprocessor::TestPoints(std::vector<keyPoint> pt_list)
 {
	 for (int k = 0; k < pt_list.size(); k++)
	 {
		 // Switch x, y for OpenCV drawing.
		 int x = pt_list[k].y;
		 int y = pt_list[k].x;
		 for (int i = x-4; i < x+4; i++)
			 for (int j = y-4; j < y+4; j ++)
			 {
				 img_load.at<cv::Vec3b>(i,j) = cv::Vec3b(0, 0, 255);
			 }
	 }
	 ImageUtil::ImageShow(img_load, 3);
 }

 vector<keyPoint>* ImagePreprocessor::GetPointList()
 {
	 printf("Get function----The size is %d\n", point_list.size());
	 return &point_list;
 }