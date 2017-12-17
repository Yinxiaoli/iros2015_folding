#include "FoldPlanner.h"

// Constructor.
FoldPlanner::FoldPlanner()
{

}

void FoldPlanner::MappingTrajectory(vector<keyPoint>* point_list, GarmentType garment)
{
	switch (garment)
	{
	case SWEATER:
		SweaterPlanner(point_list);
		break;
	case PANTS:
		PantsPlanner(point_list);
		break;
	case TOWEL:
		TowelPlanner(point_list);
		break;
	default:
		SWEATER;
	}
}

void FoldPlanner::SweaterPlanner(vector<keyPoint>* point_list)
{
	// From 2&3 to 7
	LoadTrajectory("maya_trajectory_sweater/trajectory_left_arm_fold.txt");
	start_pos = cv::Point2f((point_list->at(2).x + point_list->at(3).x)/2.0f, 
		(point_list->at(2).y + point_list->at(3).y)/2.0f);
	end_pos = cv::Point2f(point_list->at(7).x, point_list->at(7).y);
	Interpolate3D(start_pos, end_pos);

	// From 9&8 to 4
	LoadTrajectory("maya_trajectory_sweater/trajectory_right_arm_fold.txt");
	start_pos = cv::Point2f((point_list->at(8).x + point_list->at(9).x)/2.0f, 
		(point_list->at(8).y + point_list->at(9).y)/2.0f);
	end_pos = cv::Point2f(point_list->at(4).x, point_list->at(4).y);
	Interpolate3D(start_pos, end_pos);

	// Bottom up: 5->1
	LoadTrajectory("maya_trajectory_sweater/trajectory_left_arm_bottom_up.txt");
	start_pos = cv::Point2f(point_list->at(5).x, point_list->at(5).y);
	end_pos   = cv::Point2f(point_list->at(1).x, point_list->at(1).y);
	Interpolate3D(start_pos, end_pos);

	// Bottom up: 6->10
	LoadTrajectory("maya_trajectory_sweater/trajectory_right_arm_bottom_up.txt");
	start_pos = cv::Point2f(point_list->at(6).x, point_list->at(6).y);
	end_pos   = cv::Point2f(point_list->at(10).x, point_list->at(10).y);
	Interpolate3D(start_pos, end_pos);

	// Write all converted trajectories to file.
	WriteMappedTrajToFile();
}

void FoldPlanner::PantsPlanner(vector<keyPoint>* point_list)
{
	// Bottom up: 5->2
	LoadTrajectory("maya_trajectory_pants/trajectory_left_arm_bottom_up.txt");
	start_pos = cv::Point2f(point_list->at(5).x, point_list->at(5).y);
	end_pos   = cv::Point2f(point_list->at(2).x, point_list->at(2).y);
	Interpolate3D(start_pos, end_pos);
	
	// Bottom up: 4->3
	LoadTrajectory("maya_trajectory_pants/trajectory_right_arm_bottom_up.txt");
	start_pos = cv::Point2f(point_list->at(4).x, point_list->at(4).y);
	end_pos   = cv::Point2f(point_list->at(3).x, point_list->at(3).y);
	Interpolate3D(start_pos, end_pos);
	
	// From 1&2 to (3+4)/2&3
	LoadTrajectory("maya_trajectory_pants/trajectory_left_arm_fold.txt");
	start_pos = cv::Point2f((point_list->at(1).x + point_list->at(2).x)/2.0f, 
		(point_list->at(1).y + point_list->at(2).y)/2.0f);
	end_pos   = cv::Point2f(((point_list->at(3).x + point_list->at(4).x)/2.0f + point_list->at(3).x)/2, 
		((point_list->at(3).y + point_list->at(4).y)/2.0f + point_list->at(3).y)/2.0f);
	Interpolate3D(start_pos, end_pos);

	// Write all converted trajectories to file.
	WriteMappedTrajToFile();
}

void FoldPlanner::TowelPlanner(vector<keyPoint>* point_list)
{
	// Bottom up 2->3
	LoadTrajectory("maya_trajectory_towel/trajectory_right_arm_bottom_up.txt");
	start_pos = cv::Point2f(point_list->at(2).x, point_list->at(2).y);
	end_pos   = cv::Point2f(point_list->at(3).x, point_list->at(3).y);
	Interpolate3D(start_pos, end_pos);

	// Bottom up 0->1
	LoadTrajectory("maya_trajectory_towel/trajectory_left_arm_bottom_up.txt");
	start_pos = cv::Point2f(point_list->at(1).x, point_list->at(1).y);
	end_pos   = cv::Point2f(point_list->at(0).x, point_list->at(0).y);
	Interpolate3D(start_pos, end_pos);

	// Left arm fold
	LoadTrajectory("maya_trajectory_towel/trajectory_left_arm_fold.txt");
	start_pos = cv::Point2f(((point_list->at(2).x + point_list->at(3).x)/2.0f 
		+ point_list->at(3).x)/2.0f, 
		((point_list->at(2).y + point_list->at(3).y)/2.0f + point_list->at(3).y)/2.0f);
	end_pos   = cv::Point2f(((point_list->at(0).x + point_list->at(1).x)/2.0f 
		+ point_list->at(0).x)/2.0f, 
		((point_list->at(0).y + point_list->at(1).y)/2.0f + point_list->at(0).y)/2.0f);
	Interpolate3D(start_pos, end_pos);

	// Write all converted trajectories to file.
	WriteMappedTrajToFile();
}

void FoldPlanner::Interpolate3D(cv::Point2f startPos, cv::Point2f endPos)
{
	float xOffset = endPos.x - startPos.x;
	float yOffset = endPos.y - startPos.y;
	float dist = sqrt((startPos.x - endPos.x)*(startPos.x - endPos.x) 
		+ (startPos.y - endPos.y)*(startPos.y - endPos.y));

	for (int i = 0; i < traj_load.size(); ++i)
	{
		printf("Loaded trajectory %f %f \n", traj_load[i].x, traj_load[i].y);
		float x = startPos.x + traj_load[i].x * xOffset; 
		float y = startPos.y + traj_load[i].x * yOffset;
		float z = traj_load[i].y * dist;
		mapped_traj.push_back(cv::Point3f(x, y, z));
	}

	// Add a mark for each trajectory.
	mapped_traj.push_back(cv::Point3f(-1.0f, -1.0f, -1.0f));
}

void FoldPlanner::LoadTrajectory(char* fileName)
{
	// Clear previous loaded trajectory.
	traj_load.clear();
	// Read trajectory
	float f1, f2;
	string str;
	FILE* f;
	f = fopen(fileName, "r+");
	while (!feof(f))
	{
		fscanf(f, "%f %f\n", &f1, &f2);
		traj_load.push_back(cv::Point2f(f1, f2));
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
	for (int i = 0; i < mapped_traj.size(); i++)
	{
		if (mapped_traj[i].x == -1.0f)
		{
			fprintf(fp, "\n");
			continue;
		}
		fprintf(fp, "%f %f %f \n", mapped_traj[i].x, mapped_traj[i].y, mapped_traj[i].z);
	}
	fclose(fp);

	/*
	printf("xOffset = %f \n", xOffset);
	printf("yOffset = %f \n", yOffset);
	printf("dist = %f \n", dist);
	printf("start pos: %f, %f \n", startPos.x, startPos.y);
	printf("end pos: %f, %f \n", endPos.x, endPos.y);
	*/

	printf("The size is %d\n", mapped_traj.size());
	printf("I am after. \n");
	for (int i = 0; i < mapped_traj.size(); i ++)
	{
		printf("Mapped trajectory points: %f  %f  %f\n", 
			mapped_traj[i].x, mapped_traj[i].y, mapped_traj[i].z);
	}
}
