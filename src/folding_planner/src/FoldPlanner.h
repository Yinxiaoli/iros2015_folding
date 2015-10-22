// A class implements the folding plan generation for sweater, pants, and towel.
// The function reads the key point list from the previous detection step. 
// Then mapping the key points into a folding plan.

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
	void MappingTrajectory(vector<keyPoint>* point_list, GarmentType garment);

private:
	void SweaterPlanner(vector<keyPoint>* point_list);
	void PantsPlanner(vector<keyPoint>* point_list);
	void TowelPlanner(vector<keyPoint>* point_list);

	void LoadTrajectory(char* file_name);
	void Interpolate3D(cv::Point2f start_pos, cv::Point2f end_pos);
	void WriteMappedTrajToFile();

	vector<keyPoint>* point_list;
	vector<cv::Point2f> traj_load;
	vector<cv::Point3f> mapped_traj;

	cv::Point2f start_pos; 
	cv::Point2f end_pos;
};