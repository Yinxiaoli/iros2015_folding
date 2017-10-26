#include "ObjectSegmenter.h"


ObjectSegmenter::ObjectSegmenter()
{
	
}

cv::Mat ObjectSegmenter::ProcessGrabCut(cv::Mat& image, GarmentType type)
{
	CreateGrabCutMarker(type);
	//ImageUtil::ImageShow(markers);
	cv::grabCut(image, markers, cv::Rect(), cv::Mat(), cv::Mat(), 10, cv::GC_INIT_WITH_MASK);
	markers &= 1;
	markers *= 255;
	//ImageUtil::ImageShow(markers);
	markers.convertTo(markers, CV_8U);
	return markers;
}

cv::Mat ObjectSegmenter::ProcessWatershed(cv::Mat& image, GarmentType type)
{
	CreateWatershedMarker(type);
	markers.convertTo(markers, CV_32S);
	cv::watershed(image, markers);
	markers.convertTo(markers,CV_8U);
	return markers;
}

void ObjectSegmenter::SetSrcImage(cv::Mat& src)
{
	this->src_image = src;
}

void ObjectSegmenter::CreateGrabCutMarker(GarmentType type)
{
	if (type == GarmentType::SWEATER)
	{
		CreateGrabCutMarkerSweater();
	}
	else if (type == GarmentType::PANTS)
	{
		CreateGrabCutMarkerPants();
	}
	else
	{
		CreateGrabCutMarkerPants();
	}
}

void ObjectSegmenter::CreateGrabCutMarkerSweater()
{
	markers.create(src_image.size(), CV_8U);
	//markers.setTo(cv::Scalar(0));

	markers.setTo(cv::GC_PR_FGD);

	// Set four corners as background.
	markers(cv::Rect(0, 0, markers.cols-1, 5)) = cv::GC_BGD;
	markers(cv::Rect(markers.cols-6, 0, 5, markers.rows-1)) = cv::GC_BGD;
	markers(cv::Rect(0, markers.rows-6, markers.cols-1, 5)) = cv::GC_BGD;
	markers(cv::Rect(0, 0, 5, markers.rows-1)) = cv::GC_BGD;

	// Set foreground.
	// Assumption: the center of the image falls into the garment area.
	markers(cv::Rect(markers.cols/2-2, markers.rows/2-2, 4, 4)) = cv::GC_FGD;
	markers(cv::Rect(markers.cols/2-2, markers.rows/3-2, 4, 4)) = cv::GC_FGD;
	markers(cv::Rect(markers.cols/2-2, markers.rows/3*2-2, 4, 4)) = cv::GC_FGD;
}


void ObjectSegmenter::CreateGrabCutMarkerPants()
{
	markers.create(src_image.size(), CV_8U);
	markers.setTo(cv::GC_PR_FGD);

	// Set four corners as background.
	// 4 edges
	markers(cv::Rect(0, 0, markers.cols-1, 5)) = cv::GC_BGD;
	markers(cv::Rect(markers.cols-6, 0, 5, markers.rows-1)) = cv::GC_BGD;
	markers(cv::Rect(0, markers.rows-6, markers.cols-1, 5)) = cv::GC_BGD;
	markers(cv::Rect(0, 0, 5, markers.rows-1)) = cv::GC_BGD;
	// crotch
	markers(cv::Rect(0, markers.rows/2-4, markers.cols/2, markers.rows/2+4)) = cv::GC_BGD;

	// Set foreground.
	// Assumption: the center of the image falls into the garment area.
// 	markers(cv::Rect(markers.cols*3/4-2, markers.rows/2-2, 4, 4)) = cv::GC_FGD;
// 	markers(cv::Rect(markers.cols*3/4-2, markers.rows/3-2, 4, 4)) = cv::GC_FGD;
// 	markers(cv::Rect(markers.cols*3/4-2, markers.rows/3*2-2, 4, 4)) = cv::GC_FGD;

	markers(cv::Rect(markers.cols/3-2, markers.rows/4-2, 4, 4)) = cv::GC_FGD;
	markers(cv::Rect(markers.cols/3-2, markers.rows*3/4-2, 4, 4)) = cv::GC_FGD;
	markers(cv::Rect(markers.cols*3/4-5, markers.rows/2-20, 10, 40)) = cv::GC_FGD;
}


void ObjectSegmenter::CreateWatershedMarker(GarmentType type)
{
	if (type == GarmentType::SWEATER)
	{
		CreateWatershedMarkerSweater();
	}
	else if (type == GarmentType::PANTS)
	{
		CreateWatershedMarkerPants();
	}
	else
	{
		// Use the same marker as sweater.
		CreateWatershedMarkerSweater();
	}
}

void ObjectSegmenter::CreateWatershedMarkerSweater()
{
	markers.create(src_image.size(), CV_8U);
	markers.setTo(cv::Scalar(0));

	// Set four corners as background.
	markers(cv::Rect(0, 0, 5, 5)) = cv::Scalar(100);
	markers(cv::Rect(markers.cols-5, markers.rows-5, 5, 5)) = cv::Scalar(100);
	markers(cv::Rect(0, markers.rows-5, 5, 5)) = cv::Scalar(100);
	markers(cv::Rect(markers.cols-5, 0, 5, 5)) = cv::Scalar(100);

	// Set foreground.
	// Assumption: the center of the image falls into the garment area.
	markers(cv::Rect(markers.cols/2-2, markers.rows/2-2, 4, 4)) = cv::Scalar(250);
}

void ObjectSegmenter::CreateWatershedMarkerPants()
{
	markers.create(src_image.size(), CV_8U);
	markers.setTo(cv::Scalar(0));

	// Set four corners as background.
	markers(cv::Rect(0, 0, 5, 5)) = cv::Scalar(100);
	markers(cv::Rect(markers.cols-5, markers.rows-5, 5, 5)) = cv::Scalar(100);
	markers(cv::Rect(0, markers.rows-5, 5, 5)) = cv::Scalar(100);
	markers(cv::Rect(markers.cols-5, 0, 5, 5)) = cv::Scalar(100);

	// Set foreground.
	// Assumption: the center of the image falls into the garment area.
	//markers(cv::Rect(markers.cols/3-2, markers.rows/4-2, 4, 4)) = cv::Scalar(250);
	//markers(cv::Rect(markers.cols/3-2, markers.rows*3/4-2, 4, 4)) = cv::Scalar(250);
	markers(cv::Rect(markers.cols*3/4-5, markers.rows/2-20, 10, 40)) = cv::Scalar(250);
}