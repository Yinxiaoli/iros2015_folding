#include <vector>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ImageUtil.h"
#include "ImagePreprocessor.h"

#include <opencv2/opencv.hpp>

class FoldPlanner
{
public:
	FoldPlanner();
	void MappingTrajectory(vector<keyPoint>* pointList, GarmentType garment);

private:
	void SweaterPlanner(vector<keyPoint>* pointList);
	void PantsPlanner(vector<keyPoint>* pointList);
	void TowelPlanner(vector<keyPoint>* pointList);

	void LoadTrajectory(char* fileName);
	void Interpolate3D(cv::Point2f startPos, cv::Point2f endPos);
	void WriteMappedTrajToFile();

	vector<keyPoint>* pointList;
	vector<cv::Point2f> trajLoad;
	vector<cv::Point3f> mappedTraj;

	cv::Point2f startPos; 
	cv::Point2f endPos;
};