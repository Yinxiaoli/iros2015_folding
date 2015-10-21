#include "datatypes.h"
#include "registration.h"

void resize(SVar& io_vars, int in_size)
{
	io_vars.pos.resize(2, in_size);
	io_vars.conf.resize(in_size);
}

void initSolverVars(const SParameters* in_params, const SCurve* in_curve, const SVar& in_initial_vars, SSolverVars& io_vars, bool update)
{
	if(!update)
	{
		io_vars.k = 0;
		io_vars.nu = 2.0;
		io_vars.j = 0;		
		io_vars.epsilon = 1.0e-7;
	}

	if(!update || io_vars.x.pos.cols() != in_curve->nVertices)
	{
		resize(io_vars.x, in_curve->nVertices);
		resize(io_vars.xnew, in_curve->nVertices);
		io_vars.x = in_initial_vars;
		io_vars.xnew = in_initial_vars;

		int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;
		int nAngles = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 2; 

		io_vars.m = nSegs + nAngles + nSegs;
		io_vars.n = in_curve->nVertices * 2;
		io_vars.nV = in_curve->nVertices;
	
		io_vars.B.resize(io_vars.m, io_vars.n);

		io_vars.g.resize(io_vars.n);
		io_vars.f.resize(io_vars.m);
	
		io_vars.A_muI.resize(io_vars.n, io_vars.n);
		io_vars.h.resize(io_vars.n);
		io_vars.I.resize(io_vars.n, io_vars.n);
		io_vars.I.setIdentity();
	}

	ComputeNumericalDerivative(in_params, in_curve, io_vars.x, io_vars.epsilon, io_vars.B);
	Compute_f(in_params, in_curve, io_vars.x, io_vars.f);
	io_vars.g = io_vars.B.transpose() * io_vars.f;
	io_vars.A_muI = io_vars.B.transpose() * io_vars.B;

	io_vars.mu = in_params->tau * io_vars.A_muI.lpNorm<Eigen::Infinity>();
	io_vars.found = (io_vars.g.lpNorm<Eigen::Infinity>() <= in_params->epsilon_1);

	printf("|g|_inf: %f\n", io_vars.g.lpNorm<Eigen::Infinity>());
}
