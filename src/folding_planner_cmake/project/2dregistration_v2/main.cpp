#include <iostream>
using namespace std;

#include <aerialeye/glinit.h>
#include <stdint.h>
#include <GL/freeglut.h>
#include <GL/glut.h>

#include <aerialeye/glpanel.h>
#include <aerialeye/glframework.h>

#include "datatypes.h"
#include "registration.h"
#include "distancefield.h"
#include "garment_template.h"
#include "ImageUtil.h"
#include "ImagePreprocessor.h"
#include "ObjectSegmenter.h"
#include "FoldPlanner.h"


#include <bluetail/image/imageiohdr.h>

SParameters params;
SCurve curve;
SVar initialVars;
SVar vars;
SDisplayParams displayParams;
SSolverVars solverVars;

ImagePreprocessor* mImagePreprocessor = new ImagePreprocessor();
FoldPlanner* mFoldPlanner = new FoldPlanner();

//GarmentType garmentType = GarmentType::PANTS;
//char* inputFileName = "pants.jpg";

GarmentType garmentType;
char* inputFileName;

void idle()
{
	g_GLFramework->redisplayQuest();
}

void drawDF(const SImage<double, 1>& in_DF, const SRegion& in_Region)
{
	const double dist_scale = (in_Region.right-in_Region.left) / in_DF.width;

	glBegin(GL_QUADS);
	for(int j=0; j<in_DF.height; j++)
	{
		for(int i=0; i<in_DF.width; i++)
		{
			float c = in_DF.ptr[j*in_DF.width+i] * dist_scale;
			if(c > 0) glColor3f(c, c, c);
			else glColor3f(0.0, 0.0, -c);

			glVertex2f(in_Region.left + (in_Region.right-in_Region.left) * i / in_DF.width, in_Region.bottom + (in_Region.top-in_Region.bottom) * j / in_DF.height);
			glVertex2f(in_Region.left + (in_Region.right-in_Region.left) * (i+1) / in_DF.width, in_Region.bottom + (in_Region.top-in_Region.bottom) * j / in_DF.height);
			glVertex2f(in_Region.left + (in_Region.right-in_Region.left) * (i+1) / in_DF.width, in_Region.bottom + (in_Region.top-in_Region.bottom) * (j+1) / in_DF.height);
			glVertex2f(in_Region.left + (in_Region.right-in_Region.left) * i / in_DF.width, in_Region.bottom + (in_Region.top-in_Region.bottom) * (j+1) / in_DF.height);
		}
	}
	glEnd();
}

void drawCurve(const SCurve* in_Curve, const SVar& in_Vars)
{
	Eigen::Vector2d center = Eigen::Vector2d::Zero();

	glColor3f(0.2, 0.7, 0.2);
	::glLineWidth(4.0);
	::glBegin(GL_LINE_STRIP);

	for(int i=0; i<in_Curve->nVertices; i++)
		glVertex2d(in_Vars.pos.col(i).x() - center.x(), in_Vars.pos.col(i).y() - center.y());
	if(in_Curve->closed)
		glVertex2d(in_Vars.pos.col(0).x() - center.x(), in_Vars.pos.col(0).y() - center.y());
	::glEnd();

	// Inserted points
	::glPointSize(8.0); //3.0
	::glBegin(GL_POINTS);
	for(int i=0; i<in_Curve->nVertices; i++)
	{
		if(in_Curve->vertexIDs(i) < 0)
		{
			glColor3f(0.2, 0.7, 0.2);
			glVertex2d(in_Vars.pos.col(i).x() - center.x(), in_Vars.pos.col(i).y() - center.y());
		}	
	}
	::glEnd();

	// The input feature points
	::glPointSize(13.0); //5.0
	::glBegin(GL_POINTS);
	for(int i=0; i<in_Curve->nVertices; i++)
	{
		if(in_Curve->vertexIDs(i) >= 0)
		{
			glColor3f(0.7, 0.2, 0.2);	
			glVertex2d(in_Vars.pos.col(i).x() - center.x(), in_Vars.pos.col(i).y() - center.y());
		}
	}
	::glEnd();
}

void draw()
{
	glPushMatrix();
	glViewport(0, 0, g_GLFramework->MainPanel->Width, g_GLFramework->MainPanel->Height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	int mins = min(g_GLFramework->MainPanel->Width, g_GLFramework->MainPanel->Height);
	
	double xc = (displayParams.region.right + displayParams.region.left) * 0.5; double yc = (displayParams.region.top + displayParams.region.bottom) * 0.5;
	double hw = (displayParams.region.right - displayParams.region.left) * 0.5; double hh = (displayParams.region.top - displayParams.region.bottom) * 0.5;
	double w = max(hw, hh);
	
	glOrtho((xc-1.05*w)*double(g_GLFramework->MainPanel->Width)/mins, (xc+1.05*w)*double(g_GLFramework->MainPanel->Width)/mins, 
		(yc-1.05*w)*double(g_GLFramework->MainPanel->Height)/mins, (yc+1.05*w)*double(g_GLFramework->MainPanel->Height)/mins, -1.0, 2.0);

	drawDF(params.df, params.region);

	drawCurve(&curve, vars);

	glPopMatrix();	
}

void keyboard(char key, int x, int y)
{
	switch(key)
	{
	case 's':
	case 'S':
		secantLMMethod(&params, &curve, initialVars, solverVars, vars);
		showFeaturePoints(&curve, vars);
		mImagePreprocessor->RescalePoints(&curve, vars);
		mFoldPlanner->MappingTrajectory(mImagePreprocessor->GetPointList(), garmentType);
		break;
	case 'i':
	case 'I':
		initSolverVars(&params, &curve, initialVars, solverVars);
		break;
	case ' ':
		secantLMMethodSingleUpdate(&params, &curve, initialVars, solverVars, vars);
		updateCurveSubdivision(&params, initialVars, vars, &curve, solverVars);
		//updateRestLength(solverVars.x.pos, &curve);
		break;
	}
}

void windowCreate()
{

}

void genInitialVars()
{
	curve.closed = true;
	curve.nVertices = 64;
	curve.restAngles.resize(curve.nVertices);
	curve.restLengths.resize(curve.nVertices);

	resize(vars, curve.nVertices);
	resize(initialVars, curve.nVertices);

	const double R = 1.7;
	const double phi = 2.0 * PI / curve.nVertices;
	const double restL = 2.0 * R * sin(phi * 0.5);

	params.refLength = restL;

	for(int i=0; i<curve.nVertices; i++)
	{
		curve.restAngles(i) = PI;
		curve.restLengths(i) = restL;

		const double theta = 2.0 * PI * i / curve.nVertices;
		//const double _r = R * ((i%2 == 0) ? 0.7 : 1.3);
		const double _r = R;
		vars.pos(0, i) = _r * cos(theta);
		vars.pos(1, i) = _r * sin(theta);
		vars.conf(i) = 0.0;
	}

	initialVars = vars;

	initSolverVars(&params, &curve, initialVars, solverVars);
}

void genGarmentVars()
{
	initGarmentTemplate(&curve, vars, garmentType);
	params.refLength = 1.0e-1;

	initialVars = vars;

	initSolverVars(&params, &curve, initialVars, solverVars);
}

void testAngleOmega()
{
	Eigen::Vector2d e_im;
	Eigen::Vector2d e_i;

	e_im << 1, 0;
	
	int nSteps = 36;
	for(int i=0; i<=nSteps; i++)
	{
		double phi_angle = i * 360.0 / nSteps;
		double phi = i * 2.0 * PI / nSteps;
		double cP = cos(phi); double sP = sin(phi);
		e_i << cP, sP;

		double computedTheta = computeAngle(e_im, e_i);
		double Theta_angle = computedTheta * 180.0 / PI;
		double omega = computeOmegaFromAngle(computedTheta);

		std::cout << "phi: " << phi_angle << " deg, e_i: [" << cP << ", " << sP << "], theta: " << Theta_angle << " deg, omega: " << omega << std::endl; 
	}
}


void init()
{
	mImagePreprocessor->GenerateGarmentMask(inputFileName, garmentType);
	//mImagePreprocessor->GenerateGarmentMask("pants.jpg", GarmentType::PANTS);

	SImage<double, 1> the_Img = ImageUtil::ConvertToSImage(mImagePreprocessor->GetGarmentMask());

	g_GLFramework->MainPanel->Width = 1000;
	g_GLFramework->MainPanel->Height = 1000;
	g_GLFramework->Caption = "2D registration test";

	//SImage<double, 1> the_Img; 
	//initImage(-1, -1, the_Img);
	//loadImageHdr("test.hdr", the_Img);
	//flipImageY(the_Img);
	initImage(the_Img.width, the_Img.height, params.df);

	printf("%d, %d\n", the_Img.width, the_Img.height);

	generateDF(the_Img, params.df);

	params.alpha = 0.001;
	params.YA = 0.01;
	params.fit = 20000.0;
	params.conf = 100.0;
	params.substep_fit = 16;

	params.region.left = -2.0;
	params.region.right = 2.0;
	params.region.top = 2.0;
	params.region.bottom = -2.0;

	params.delta = 1.0e-4;
	params.tau = 0.001;
	params.kmax = 200;
	params.epsilon_1 = 1.0e-10;
	params.epsilon_2 = 1.0e-10;

	displayParams.region.left = params.region.left;
	displayParams.region.right = params.region.right;
	displayParams.region.top = -params.region.top;
	displayParams.region.bottom = -params.region.bottom;

	genInitialVars();
	genGarmentVars();
	//secantLMMethod(&params, &curve, initialVars, solverVars, vars);

	//Start Automatically
	secantLMMethod(&params, &curve, initialVars, solverVars, vars);
	mImagePreprocessor->RescalePoints(&curve, vars);
	mFoldPlanner->MappingTrajectory(mImagePreprocessor->GetPointList(), garmentType);
}

void GLFInitialize(int argc, char* argv[])
{
	inputFileName = argv[1];
	int iType = (int)(argv[2][0]-'0');
	garmentType = static_cast<GarmentType>(iType);

	g_GLFramework->OnIdle += _delegator_create(&idle);
	g_GLFramework->MainPanel->OnDraw += _delegator_create(&draw);
	//g_GLFramework->MainPanel->OnMouseDown += _delegator_create(&mouseDown);
	//g_GLFramework->MainPanel->OnMouseDrag += _delegator_create(&mouseDrag);
	g_GLFramework->OnKey += _delegator_create(&keyboard);
	g_GLFramework->OnCreatedWindow += _delegator_create(&windowCreate);
	init();
}

