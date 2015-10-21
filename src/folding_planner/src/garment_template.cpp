#include "garment_template.h"
#include "registration.h"

void initGarmentTemplate(SCurve* io_curve, SVar& io_vars, GarmentType type)
{
	if (type == GarmentType::SWEATER)
	{
		initSweaterTemplate(io_curve, io_vars);
	}
	else if (type == GarmentType::PANTS)
	{
		initPantsTemplate(io_curve, io_vars);
	}
	else
	{
		initTowelTemplate(io_curve, io_vars);
	}
}

void initSweaterTemplate(SCurve* io_curve, SVar& io_vars)
{
	io_curve->closed = true;
	io_curve->nVertices = 12;
	io_curve->restAngles.resize(io_curve->nVertices);
	io_curve->restLengths.resize(io_curve->nVertices);
	io_curve->vertexIDs.resize(io_curve->nVertices);

	resize(io_vars, io_curve->nVertices);

	/*
	// Old garment template
	io_vars.pos(0, 0) = -0.15;
	io_vars.pos(1, 0) =  0.7;
	io_vars.pos(0, 1) = -0.4;
	io_vars.pos(1, 1) =  0.9;
	io_vars.pos(0, 2) = -1.8;
	io_vars.pos(1, 2) =  0.1;
	io_vars.pos(0, 3) = -1.7;
	io_vars.pos(1, 3) = -0.4;
	io_vars.pos(0, 4) = -0.6;
	io_vars.pos(1, 4) =  0.0;
	io_vars.pos(0, 5) = -0.6;
	io_vars.pos(1, 5) = -1.2;
	*/


	io_vars.pos(0, 0) = -0.2;
	io_vars.pos(1, 0) =  0.9;
	io_vars.pos(0, 1) = -0.5;
	io_vars.pos(1, 1) =  1.0;
	io_vars.pos(0, 2) = -1.8;
	io_vars.pos(1, 2) =  0.4;
	io_vars.pos(0, 3) = -1.7;
	io_vars.pos(1, 3) = -0.4;
	io_vars.pos(0, 4) = -0.6;
	io_vars.pos(1, 4) =  0.0;
	io_vars.pos(0, 5) = -0.6;
	io_vars.pos(1, 5) = -1.0;
	io_vars.pos(0, 6) = -io_vars.pos(0, 5);
	io_vars.pos(1, 6) =  io_vars.pos(1, 5);
	io_vars.pos(0, 7) = -io_vars.pos(0, 4);
	io_vars.pos(1, 7) =  io_vars.pos(1, 4);
	io_vars.pos(0, 8) = -io_vars.pos(0, 3);
	io_vars.pos(1, 8) =  io_vars.pos(1, 3);
	io_vars.pos(0, 9) = -io_vars.pos(0, 2);
	io_vars.pos(1, 9) =  io_vars.pos(1, 2);
	io_vars.pos(0,10) = -io_vars.pos(0, 1);
	io_vars.pos(1,10) =  io_vars.pos(1, 1);
	io_vars.pos(0,11) = -io_vars.pos(0, 0);
	io_vars.pos(1,11) =  io_vars.pos(1, 0);

	for(int i=0; i<io_curve->nVertices; i++)
	{
		io_vars.conf(i) = 0.0;
		io_curve->vertexIDs(i) = i;
	}

	int nSegs = io_curve->nVertices;

	for(int i=0; i<nSegs; i++)
	{
		int im = (i+nSegs-1) % nSegs;
		int ip = (i+1) % nSegs;

		const Eigen::Vector2d e_im = io_vars.pos.col(i) - io_vars.pos.col(im);
		const Eigen::Vector2d e_i = io_vars.pos.col(ip) - io_vars.pos.col(i);

		io_curve->restAngles(i) = ComputeAngle(e_im, e_i);//theta;
		io_curve->restLengths(i) = (e_i).norm();
	}
}


void initPantsTemplate(SCurve* io_curve, SVar& io_vars)
{
	io_curve->closed = true;
	io_curve->nVertices = 7;
	io_curve->restAngles.resize(io_curve->nVertices);
	io_curve->restLengths.resize(io_curve->nVertices);
	io_curve->vertexIDs.resize(io_curve->nVertices);

	resize(io_vars, io_curve->nVertices);

	io_vars.pos(0, 0) =  0.2;
	io_vars.pos(1, 0) =  0.0;
	io_vars.pos(0, 1) = -1.2;
	io_vars.pos(1, 1) =  0.2;
	io_vars.pos(0, 2) = -1.2;
	io_vars.pos(1, 2) =  0.9;
	io_vars.pos(0, 3) =  1.3;
	io_vars.pos(1, 3) =  0.7;
	io_vars.pos(0, 4) =  io_vars.pos(0, 3);
	io_vars.pos(1, 4) = -io_vars.pos(1, 3);
	io_vars.pos(0, 5) =  io_vars.pos(0, 2);
	io_vars.pos(1, 5) = -io_vars.pos(1, 2);
	io_vars.pos(0, 6) =  io_vars.pos(0, 1);
	io_vars.pos(1, 6) = -io_vars.pos(1, 1);

	for(int i=0; i<io_curve->nVertices; i++)
	{
		io_vars.conf(i) = 0.0;
		io_curve->vertexIDs(i) = i;
	}

	int nSegs = io_curve->nVertices;

	for(int i=0; i<nSegs; i++)
	{
		int im = (i+nSegs-1) % nSegs;
		int ip = (i+1) % nSegs;

		const Eigen::Vector2d e_im = io_vars.pos.col(i) - io_vars.pos.col(im);
		const Eigen::Vector2d e_i = io_vars.pos.col(ip) - io_vars.pos.col(i);

		io_curve->restAngles(i) = ComputeAngle(e_im, e_i);//theta;
		io_curve->restLengths(i) = (e_i).norm();
	}
}

void initTowelTemplate(SCurve* io_curve, SVar& io_vars)
{
	io_curve->closed = true;
	io_curve->nVertices = 4;
	io_curve->restAngles.resize(io_curve->nVertices);
	io_curve->restLengths.resize(io_curve->nVertices);
	io_curve->vertexIDs.resize(io_curve->nVertices);

	resize(io_vars, io_curve->nVertices);

	io_vars.pos(0, 0) =  1.5;
	io_vars.pos(1, 0) =  1.0;
	io_vars.pos(0, 1) =  io_vars.pos(0, 0);
	io_vars.pos(1, 1) = -io_vars.pos(1, 0);
	io_vars.pos(0, 2) = -io_vars.pos(0, 0);;
	io_vars.pos(1, 2) = -io_vars.pos(1, 0);;
	io_vars.pos(0, 3) = -io_vars.pos(0, 0);
	io_vars.pos(1, 3) =  io_vars.pos(1, 0);;

	for(int i=0; i<io_curve->nVertices; i++)
	{
		io_vars.conf(i) = 0.0;
		io_curve->vertexIDs(i) = i;
	}

	int nSegs = io_curve->nVertices;

	for(int i=0; i<nSegs; i++)
	{
		int im = (i+nSegs-1) % nSegs;
		int ip = (i+1) % nSegs;

		const Eigen::Vector2d e_im = io_vars.pos.col(i) - io_vars.pos.col(im);
		const Eigen::Vector2d e_i = io_vars.pos.col(ip) - io_vars.pos.col(i);

		io_curve->restAngles(i) = ComputeAngle(e_im, e_i);//theta;
		io_curve->restLengths(i) = (e_i).norm();
	}
}