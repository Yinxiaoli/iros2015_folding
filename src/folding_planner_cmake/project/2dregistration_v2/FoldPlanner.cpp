#include "FoldPlanner.h"

FoldPlanner::FoldPlanner()
{

}


void FoldPlanner::MappingTrajectory(vector<keyPoint>* pointList, GarmentType garment)
{
	switch (garment)
	{
	case SWEATER:
		SweaterPlanner(pointList);
		break;
	case PANTS:
		PantsPlanner(pointList);
		break;
	case TOWEL:
		TowelPlanner(pointList);
		break;
	default:
		SWEATER;
	}
}


void FoldPlanner::SweaterPlanner(vector<keyPoint>* pointList)
{
	// From 2&3 to 7
	LoadTrajectory("maya_trajectory_sweater/trajectory_left_arm_fold.txt");
	startPos = cv::Point2f((pointList->at(2).x + pointList->at(3).x)/2.0f, 
		(pointList->at(2).y + pointList->at(3).y)/2.0f);
	endPos = cv::Point2f(pointList->at(7).x, pointList->at(7).y);
	Interpolate3D(startPos, endPos);

	// From 9&8 to 4
	LoadTrajectory("maya_trajectory_sweater/trajectory_right_arm_fold.txt");
	startPos = cv::Point2f((pointList->at(8).x + pointList->at(9).x)/2.0f, 
		(pointList->at(8).y + pointList->at(9).y)/2.0f);
	endPos = cv::Point2f(pointList->at(4).x, pointList->at(4).y);
	Interpolate3D(startPos, endPos);

	// Bottom up: 5->1
	LoadTrajectory("maya_trajectory_sweater/trajectory_left_arm_bottom_up.txt");
	startPos = cv::Point2f(pointList->at(5).x, pointList->at(5).y);
	endPos   = cv::Point2f(pointList->at(1).x, pointList->at(1).y);
	Interpolate3D(startPos, endPos);

	// Bottom up: 6->10
	LoadTrajectory("maya_trajectory_sweater/trajectory_right_arm_bottom_up.txt");
	startPos = cv::Point2f(pointList->at(6).x, pointList->at(6).y);
	endPos   = cv::Point2f(pointList->at(10).x, pointList->at(10).y);
	Interpolate3D(startPos, endPos);

	// Write all converted trajectories to file.
	WriteMappedTrajToFile();
}


void FoldPlanner::PantsPlanner(vector<keyPoint>* pointList)
{
	// Bottom up: 5->2
	LoadTrajectory("maya_trajectory_pants/trajectory_left_arm_bottom_up.txt");
	startPos = cv::Point2f(pointList->at(5).x, pointList->at(5).y);
	endPos   = cv::Point2f(pointList->at(2).x, pointList->at(2).y);
	Interpolate3D(startPos, endPos);
	
	// Bottom up: 4->3
	LoadTrajectory("maya_trajectory_pants/trajectory_right_arm_bottom_up.txt");
	startPos = cv::Point2f(pointList->at(4).x, pointList->at(4).y);
	endPos   = cv::Point2f(pointList->at(3).x, pointList->at(3).y);
	Interpolate3D(startPos, endPos);
	
	// From 1&2 to (3+4)/2&3
	LoadTrajectory("maya_trajectory_pants/trajectory_left_arm_fold.txt");
	startPos = cv::Point2f((pointList->at(1).x + pointList->at(2).x)/2.0f, 
		(pointList->at(1).y + pointList->at(2).y)/2.0f);
	endPos   = cv::Point2f(((pointList->at(3).x + pointList->at(4).x)/2.0f + pointList->at(3).x)/2, 
		((pointList->at(3).y + pointList->at(4).y)/2.0f + pointList->at(3).y)/2.0f);
	Interpolate3D(startPos, endPos);

	// Write all converted trajectories to file.
	WriteMappedTrajToFile();
}


void FoldPlanner::TowelPlanner(vector<keyPoint>* pointList)
{
	// Bottom up 2->3
	LoadTrajectory("maya_trajectory_towel/trajectory_right_arm_bottom_up.txt");
	startPos = cv::Point2f(pointList->at(2).x, pointList->at(2).y);
	endPos   = cv::Point2f(pointList->at(3).x, pointList->at(3).y);
	Interpolate3D(startPos, endPos);

	// Bottom up 0->1
	LoadTrajectory("maya_trajectory_towel/trajectory_left_arm_bottom_up.txt");
	startPos = cv::Point2f(pointList->at(1).x, pointList->at(1).y);
	endPos   = cv::Point2f(pointList->at(0).x, pointList->at(0).y);
	Interpolate3D(startPos, endPos);

	// Left arm fold
	LoadTrajectory("maya_trajectory_towel/trajectory_left_arm_fold.txt");
	startPos = cv::Point2f(((pointList->at(2).x + pointList->at(3).x)/2.0f + pointList->at(3).x)/2.0f, 
		((pointList->at(2).y + pointList->at(3).y)/2.0f + pointList->at(3).y)/2.0f);
	endPos   = cv::Point2f(((pointList->at(0).x + pointList->at(1).x)/2.0f + pointList->at(0).x)/2.0f, 
		((pointList->at(0).y + pointList->at(1).y)/2.0f + pointList->at(0).y)/2.0f);
	Interpolate3D(startPos, endPos);

	// Write all converted trajectories to file.
	WriteMappedTrajToFile();
}


void FoldPlanner::Interpolate3D(cv::Point2f startPos, cv::Point2f endPos)
{
	float xOffset = endPos.x - startPos.x;
	float yOffset = endPos.y - startPos.y;
	float dist = sqrt((startPos.x - endPos.x)*(startPos.x - endPos.x) + (startPos.y - endPos.y)*(startPos.y - endPos.y));

	for (int i = 0; i < trajLoad.size(); ++i)
	{
		printf("Loaded trajectory %f %f \n", trajLoad[i].x, trajLoad[i].y);
		float x = startPos.x + trajLoad[i].x * xOffset; 
		float y = startPos.y + trajLoad[i].x * yOffset;
		float z = trajLoad[i].y * dist;
		mappedTraj.push_back(cv::Point3f(x, y, z));
	}

	// Add a mark for each trajectory.
	mappedTraj.push_back(cv::Point3f(-1.0f, -1.0f, -1.0f));
}


void FoldPlanner::LoadTrajectory(char* fileName)
{
	// Clear previous loaded trajectory.
	trajLoad.clear();
	// Read trajectory
	float f1, f2;
	string str;
	FILE* f;
	f = fopen(fileName, "r+");
	while (!feof(f))
	{
		fscanf(f, "%f %f\n", &f1, &f2);
		trajLoad.push_back(cv::Point2f(f1, f2));
	}
}


void FoldPlanner::WriteMappedTrajToFile()
{
	// Write to file.
	remove("mapped_keypoints.txt");
	FILE *fp = fopen("mapped_keypoints.txt", "w");

	if (fp == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}
	for (int i = 0; i < mappedTraj.size(); i++)
	{
		if (mappedTraj[i].x == -1.0f)
		{
			fprintf(fp, "\n");
			continue;
		}
		fprintf(fp, "%f %f %f \n", mappedTraj[i].x, mappedTraj[i].y, mappedTraj[i].z);
	}
	fclose(fp);

	/*
	printf("xOffset = %f \n", xOffset);
	printf("yOffset = %f \n", yOffset);
	printf("dist = %f \n", dist);
	printf("start pos: %f, %f \n", startPos.x, startPos.y);
	printf("end pos: %f, %f \n", endPos.x, endPos.y);
	*/

	printf("The size is %d\n", mappedTraj.size());
	printf("I am after. \n");
	for (int i = 0; i < mappedTraj.size(); i ++)
	{
		printf("Mapped trajectory points: %f  %f  %f\n", mappedTraj[i].x, mappedTraj[i].y, mappedTraj[i].z);
	}
}