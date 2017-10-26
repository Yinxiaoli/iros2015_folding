#include "garment_template.h"
#include "registration.h"

void initGarmentTemplate(SCurve* io_Curve, SVar& io_Vars, GarmentType type)
{
	if (type == GarmentType::SWEATER)
	{
		initSweaterTemplate(io_Curve, io_Vars);
	}
	else if (type == GarmentType::PANTS)
	{
		initPantsTemplate(io_Curve, io_Vars);
	}
	else
	{
		initTowelTemplate(io_Curve, io_Vars);
	}
}

void initSweaterTemplate(SCurve* io_Curve, SVar& io_Vars)
{
	io_Curve->closed = true;
	io_Curve->nVertices = 12;
	io_Curve->restAngles.resize(io_Curve->nVertices);
	io_Curve->restLengths.resize(io_Curve->nVertices);
	io_Curve->vertexIDs.resize(io_Curve->nVertices);

	resize(io_Vars, io_Curve->nVertices);

	/*
	// Old garment template
	io_Vars.pos(0, 0) = -0.15;
	io_Vars.pos(1, 0) =  0.7;
	io_Vars.pos(0, 1) = -0.4;
	io_Vars.pos(1, 1) =  0.9;
	io_Vars.pos(0, 2) = -1.8;
	io_Vars.pos(1, 2) =  0.1;
	io_Vars.pos(0, 3) = -1.7;
	io_Vars.pos(1, 3) = -0.4;
	io_Vars.pos(0, 4) = -0.6;
	io_Vars.pos(1, 4) =  0.0;
	io_Vars.pos(0, 5) = -0.6;
	io_Vars.pos(1, 5) = -1.2;
	*/


	io_Vars.pos(0, 0) = -0.2;
	io_Vars.pos(1, 0) =  0.9;
	io_Vars.pos(0, 1) = -0.5;
	io_Vars.pos(1, 1) =  1.0;
	io_Vars.pos(0, 2) = -1.8;
	io_Vars.pos(1, 2) =  0.4;
	io_Vars.pos(0, 3) = -1.7;
	io_Vars.pos(1, 3) = -0.4;
	io_Vars.pos(0, 4) = -0.6;
	io_Vars.pos(1, 4) =  0.0;
	io_Vars.pos(0, 5) = -0.6;
	io_Vars.pos(1, 5) = -1.0;
	io_Vars.pos(0, 6) = -io_Vars.pos(0, 5);
	io_Vars.pos(1, 6) =  io_Vars.pos(1, 5);
	io_Vars.pos(0, 7) = -io_Vars.pos(0, 4);
	io_Vars.pos(1, 7) =  io_Vars.pos(1, 4);
	io_Vars.pos(0, 8) = -io_Vars.pos(0, 3);
	io_Vars.pos(1, 8) =  io_Vars.pos(1, 3);
	io_Vars.pos(0, 9) = -io_Vars.pos(0, 2);
	io_Vars.pos(1, 9) =  io_Vars.pos(1, 2);
	io_Vars.pos(0,10) = -io_Vars.pos(0, 1);
	io_Vars.pos(1,10) =  io_Vars.pos(1, 1);
	io_Vars.pos(0,11) = -io_Vars.pos(0, 0);
	io_Vars.pos(1,11) =  io_Vars.pos(1, 0);

	for(int i=0; i<io_Curve->nVertices; i++)
	{
		io_Vars.conf(i) = 0.0;
		io_Curve->vertexIDs(i) = i;
	}

	int nSegs = io_Curve->nVertices;

	for(int i=0; i<nSegs; i++)
	{
		int im = (i+nSegs-1) % nSegs;
		int ip = (i+1) % nSegs;

		const Eigen::Vector2d e_im = io_Vars.pos.col(i) - io_Vars.pos.col(im);
		const Eigen::Vector2d e_i = io_Vars.pos.col(ip) - io_Vars.pos.col(i);

		io_Curve->restAngles(i) = computeAngle(e_im, e_i);//theta;
		io_Curve->restLengths(i) = (e_i).norm();
	}
}


void initPantsTemplate(SCurve* io_Curve, SVar& io_Vars)
{
	io_Curve->closed = true;
	io_Curve->nVertices = 7;
	io_Curve->restAngles.resize(io_Curve->nVertices);
	io_Curve->restLengths.resize(io_Curve->nVertices);
	io_Curve->vertexIDs.resize(io_Curve->nVertices);

	resize(io_Vars, io_Curve->nVertices);

	io_Vars.pos(0, 0) =  0.2;
	io_Vars.pos(1, 0) =  0.0;
	io_Vars.pos(0, 1) = -1.2;
	io_Vars.pos(1, 1) =  0.2;
	io_Vars.pos(0, 2) = -1.2;
	io_Vars.pos(1, 2) =  0.9;
	io_Vars.pos(0, 3) =  1.3;
	io_Vars.pos(1, 3) =  0.7;
	io_Vars.pos(0, 4) =  io_Vars.pos(0, 3);
	io_Vars.pos(1, 4) = -io_Vars.pos(1, 3);
	io_Vars.pos(0, 5) =  io_Vars.pos(0, 2);
	io_Vars.pos(1, 5) = -io_Vars.pos(1, 2);
	io_Vars.pos(0, 6) =  io_Vars.pos(0, 1);
	io_Vars.pos(1, 6) = -io_Vars.pos(1, 1);

	for(int i=0; i<io_Curve->nVertices; i++)
	{
		io_Vars.conf(i) = 0.0;
		io_Curve->vertexIDs(i) = i;
	}

	int nSegs = io_Curve->nVertices;

	for(int i=0; i<nSegs; i++)
	{
		int im = (i+nSegs-1) % nSegs;
		int ip = (i+1) % nSegs;

		const Eigen::Vector2d e_im = io_Vars.pos.col(i) - io_Vars.pos.col(im);
		const Eigen::Vector2d e_i = io_Vars.pos.col(ip) - io_Vars.pos.col(i);

		io_Curve->restAngles(i) = computeAngle(e_im, e_i);//theta;
		io_Curve->restLengths(i) = (e_i).norm();
	}
}

void initTowelTemplate(SCurve* io_Curve, SVar& io_Vars)
{
	io_Curve->closed = true;
	io_Curve->nVertices = 4;
	io_Curve->restAngles.resize(io_Curve->nVertices);
	io_Curve->restLengths.resize(io_Curve->nVertices);
	io_Curve->vertexIDs.resize(io_Curve->nVertices);

	resize(io_Vars, io_Curve->nVertices);

	io_Vars.pos(0, 0) =  1.5;
	io_Vars.pos(1, 0) =  1.0;
	io_Vars.pos(0, 1) =  io_Vars.pos(0, 0);
	io_Vars.pos(1, 1) = -io_Vars.pos(1, 0);
	io_Vars.pos(0, 2) = -io_Vars.pos(0, 0);;
	io_Vars.pos(1, 2) = -io_Vars.pos(1, 0);;
	io_Vars.pos(0, 3) = -io_Vars.pos(0, 0);
	io_Vars.pos(1, 3) =  io_Vars.pos(1, 0);;

	for(int i=0; i<io_Curve->nVertices; i++)
	{
		io_Vars.conf(i) = 0.0;
		io_Curve->vertexIDs(i) = i;
	}

	int nSegs = io_Curve->nVertices;

	for(int i=0; i<nSegs; i++)
	{
		int im = (i+nSegs-1) % nSegs;
		int ip = (i+1) % nSegs;

		const Eigen::Vector2d e_im = io_Vars.pos.col(i) - io_Vars.pos.col(im);
		const Eigen::Vector2d e_i = io_Vars.pos.col(ip) - io_Vars.pos.col(i);

		io_Curve->restAngles(i) = computeAngle(e_im, e_i);//theta;
		io_Curve->restLengths(i) = (e_i).norm();
	}
}