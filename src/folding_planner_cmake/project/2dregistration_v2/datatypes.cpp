#include "datatypes.h"
#include "registration.h"

void resize(SVar& io_Vars, int in_Size)
{
	io_Vars.pos.resize(2, in_Size);
	io_Vars.conf.resize(in_Size);
}

void initSolverVars(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_InitialVars, SSolverVars& io_Vars, bool update)
{
	if(!update)
	{
		io_Vars.k = 0;
		io_Vars.nu = 2.0;
		io_Vars.j = 0;		
		io_Vars.epsilon = 1.0e-7;
	}

	if(!update || io_Vars.x.pos.cols() != in_Curve->nVertices)
	{
		resize(io_Vars.x, in_Curve->nVertices);
		resize(io_Vars.xnew, in_Curve->nVertices);
		io_Vars.x = in_InitialVars;
		io_Vars.xnew = in_InitialVars;

		int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;
		int nAngles = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 2; 

		io_Vars.m = nSegs + nAngles + nSegs;
		io_Vars.n = in_Curve->nVertices * 2;
		io_Vars.nV = in_Curve->nVertices;
	
		io_Vars.B.resize(io_Vars.m, io_Vars.n);

		io_Vars.g.resize(io_Vars.n);
		io_Vars.f.resize(io_Vars.m);
	
		io_Vars.A_muI.resize(io_Vars.n, io_Vars.n);
		io_Vars.h.resize(io_Vars.n);
		io_Vars.I.resize(io_Vars.n, io_Vars.n);
		io_Vars.I.setIdentity();
	}

	computeNumericalDerivative(in_Params, in_Curve, io_Vars.x, io_Vars.epsilon, io_Vars.B);
	compute_f(in_Params, in_Curve, io_Vars.x, io_Vars.f);
	io_Vars.g = io_Vars.B.transpose() * io_Vars.f;
	io_Vars.A_muI = io_Vars.B.transpose() * io_Vars.B;

	io_Vars.mu = in_Params->tau * io_Vars.A_muI.lpNorm<Eigen::Infinity>();
	io_Vars.found = (io_Vars.g.lpNorm<Eigen::Infinity>() <= in_Params->epsilon_1);

	printf("|g|_inf: %f\n", io_Vars.g.lpNorm<Eigen::Infinity>());
}
