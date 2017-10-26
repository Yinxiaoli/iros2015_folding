#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include <Eigen/Core>
#include <stdint.h>

#include <bluetail/image/image.h>
using namespace _BlueTail::_Image;

const double PI = 3.14159265359;

struct SRegion
{
	double left;
	double right;
	double top;
	double bottom;
};

struct SDisplayParams
{
	SRegion region;
};

struct SCurve
{
	bool closed;
	int32_t nVertices;
	Eigen::VectorXd restAngles;
	Eigen::VectorXd restLengths;
	Eigen::VectorXi vertexIDs; //use this ID to retrieve feature points; inserted points have a negative id (-1).
};

struct SVar
{
	Eigen::Matrix2Xd pos;
	Eigen::VectorXd conf;
};

struct SSolverVars
{
	int k ;
	double nu;
	SVar x;
	SVar xnew;

	int m;
	int n;
	int nV;
	
	Eigen::MatrixXd B;

	double epsilon;

	Eigen::VectorXd g;
	Eigen::VectorXd f;

	Eigen::MatrixXd A_muI;
	Eigen::VectorXd h;

	double mu;

	Eigen::MatrixXd I;
	bool found;

	int j;

};

struct SParameters
{
	//material params
	double YA; //young modulus * cross-section area
	double alpha; //bending coeff
	double fit; //fit coeff
	double conf; //confidence coeff

	//distance field
	SImage<double, 1> df;
	SRegion region;

	//solver params
	int kmax; //maximum iterations for optimization solver
	double epsilon_1;
	double epsilon_2;
	double tau;
	double delta;

	int substep_fit;

	//geometry
	double refLength;
};

extern SParameters params;
extern SCurve curve;
extern Eigen::Matrix2Xd positions;
extern SDisplayParams displayParams;
extern SSolverVars solverVars;

void resize(SVar& io_Vars, int in_Size);
void initSolverVars(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_InitialVars, SSolverVars& io_Vars, bool update = false);

#endif
