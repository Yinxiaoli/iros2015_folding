#include "registration.h"
#include <Eigen/Cholesky>
#include <vector>

struct SPoint2D
{
	double x[2];
};

double computeAngle(const Eigen::Vector2d& e_im, const Eigen::Vector2d& e_i)
{
	//compute theta
	//when e_im.dot(e_i) = 1.0, theta = pi;
	const double r_sinT = e_im(0)*e_i(1) - e_im(1)*e_i(0);
	const double r_cosT = -e_im.dot(e_i);
	return atan2(r_sinT, r_cosT);
}

double sgn(const double& x)
{
	return (x >= 0) ? 1.0 : -1.0;
}


double computeOmegaFromAngle(const double& theta)
{
	const double sinT = sin(theta);
	const double cosT = cos(theta);
	const double tan_phi_over_2 = sgn(sinT) * sqrt((1.0+cosT)/(max(0.0, 1.0-cosT) + 1.0e-10));
	return 2.0 * tan_phi_over_2;
}

double computeOmega(const Eigen::Vector2d& e_im, const Eigen::Vector2d& e_i)
{
	//compute 2.0 * tan(phi / 2.0)
	//phi = pi - theta
	return computeOmegaFromAngle(computeAngle(e_im, e_i));	
}

double sampleDistanceField(const SImage<double, 1>& in_DF, const SRegion& in_Region, const Eigen::Vector2d& x)
{
	assert(in_DF.width == in_DF.height);
	assert(in_Region.right-in_Region.left == in_Region.top-in_Region.bottom);

	Eigen::Vector2d proj_x = x;
	proj_x(0) = std::max(in_Region.left, std::min(in_Region.right, x(0)));
	proj_x(1) = std::max(in_Region.bottom, std::min(in_Region.top, x(1)));

	const double dist_scale = (in_Region.right-in_Region.left) / in_DF.width;

	double fi = (proj_x(0) - in_Region.left) * in_DF.width / (in_Region.right - in_Region.left);
	double fj = (proj_x(1) - in_Region.bottom) * in_DF.height / (in_Region.top - in_Region.bottom);
	double _i = floor(fi);
	double _j = floor(fj);

	int i = std::max(0, std::min(in_DF.width-1, int(_i)));
	int j = std::max(0, std::min(in_DF.height-1, int(_j)));
	int ip = std::min(in_DF.width-1, i+1);
	int jp = std::min(in_DF.height-1, j+1);

	double s = fi - _i;
	double t = fj - _j;

	double dist_proj = (1.0 - t) * (s * in_DF.ptr[j*in_DF.width+ip] + (1.0-s) * in_DF.ptr[j*in_DF.width+i])
		+ t * (s * in_DF.ptr[jp*in_DF.width+ip] + (1.0-s) * in_DF.ptr[jp*in_DF.width+i]);

	return dist_proj * dist_scale + (proj_x - x).norm();
}

double integrateDF2OverSegment(const SParameters* in_Params, const Eigen::Vector2d& x1, const Eigen::Vector2d& x2, const double restL)
{
	double tot = 0.0;
	const double len = restL / in_Params->substep_fit;

	//printf("integ_DF: ");
	for(int i=0; i<in_Params->substep_fit; i++)
	{
		const Eigen::Vector2d& p1 = x1 + (x2-x1) * double(i)/double(in_Params->substep_fit);
		const Eigen::Vector2d& p2 = x1 + (x2-x1) * double(i+1)/double(in_Params->substep_fit);
		
		const double d1 = sampleDistanceField(in_Params->df, in_Params->region, p1);
		const double d2 = sampleDistanceField(in_Params->df, in_Params->region, p2);

		//printf("[%f, %f, %f], ", d1, d2, len);

		const double e1 = 0.5 * (exp(d1) + exp(-d1)) - 1.0;
		const double e2 = 0.5 * (exp(d2) + exp(-d2)) - 1.0;

		//tot += 0.5 * (d1*d1+d2*d2) * len;
		tot += 0.5 * (e1*e1+e2*e2) * len;
	}
	//printf("\n");

	return tot;
}

inline double computeSegmentElasticEnergy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const double& YA, const double& restLength)
{
	const Eigen::Vector2d diff = p1 - p2;
	return 0.5 * YA * restLength
		* (diff.norm() / restLength - 1)
		* (diff.norm() / restLength - 1);
}

inline double computeSegmentBendingEnergy(const Eigen::Vector2d& pp, const Eigen::Vector2d& pc, const Eigen::Vector2d& pn, 
	const double& alpha, const double& restLengthP, const double& restLengthN, const double& theta)
{
	const Eigen::Vector2d e_im = pc - pp;
	const Eigen::Vector2d e_i = pn - pc;

	/*
	double omega = 2.0 * fabs(e_im(0)*e_i(1)-e_im(1)*e_i(0)) / (e_im.norm()*e_i.norm() + e_im.dot(e_i));
	double omega_bar = 2.0 * fabs(tan(phi*0.5));
	//*/
	double omega = computeOmega(e_im, e_i);
	double omega_bar = computeOmegaFromAngle(theta);
	return alpha * (omega - omega_bar) * (omega - omega_bar) / (restLengthP + restLengthN);
}

double computeEnergy_elastic(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;

	double energy = 0.0;

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_Curve->nVertices;
		energy += computeSegmentElasticEnergy(in_Position.col(ip), in_Position.col(i), in_Params->YA, in_Curve->restLengths(i));
	}

	return energy;
}

double computeEnergy_bending(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position)
{
	int nAngles = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 2; 

	double energy = 0.0;

	for(int i=0; i<nAngles; i++)
	{
		int iim = in_Curve->closed ? 
			(i + in_Curve->nVertices - 1) % in_Curve->nVertices : i;
		int ii = in_Curve->closed ? i : i + 1;
		int iip = in_Curve->closed ? (i + 1) % in_Curve->nVertices : i + 2;

		double restL_m = in_Curve->closed ? in_Curve->restLengths((i + in_Curve->nVertices - 1) % in_Curve->nVertices) : 
			in_Curve->restLengths(i);
		double restL = in_Curve->closed ? in_Curve->restLengths(i) : 
			in_Curve->restLengths(i+1);

		energy += computeSegmentBendingEnergy(in_Position.col(iim), in_Position.col(ii), in_Position.col(iip),
			in_Params->alpha, restL_m, restL, in_Curve->restAngles(i));
	}

	return energy;
}

double computeEnergy_fit(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;

	double energy = 0.0;

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_Curve->nVertices;
		double int_df_seg = integrateDF2OverSegment(in_Params, in_Position.col(i), in_Position.col(ip), in_Curve->restLengths(i));
		energy += 0.5 * in_Params->fit * int_df_seg;
	}

	return energy;
}

double computeEnergy(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_Vars)
{
	const double E_elastic = computeEnergy_elastic(in_Params, in_Curve, in_Vars.pos);
	const double E_bending = computeEnergy_bending(in_Params, in_Curve, in_Vars.pos);
	const double E_fit = computeEnergy_fit(in_Params, in_Curve, in_Vars.pos);
	return E_elastic + E_bending + E_fit;
}

void compute_f_elastic(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position, Eigen::VectorXd& io_f, int offset_row)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;

	assert(io_f.rows() >= nSegs + offset_row);
	assert(in_Curve->nVertices == in_Position.cols());

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_Curve->nVertices;
		const Eigen::Vector2d diff = in_Position.col(ip) - in_Position.col(i);
		io_f(i+offset_row) = sqrt(0.5 * in_Params->YA * in_Curve->restLengths(i)) * (diff.norm() / in_Curve->restLengths(i) - 1);
	}
}

void compute_f_bending(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position, Eigen::VectorXd& io_f, int offset_row)
{
	int nAngles = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 2; 

	assert(io_f.rows() >= nAngles + offset_row);
	assert(in_Curve->nVertices == in_Position.cols());

	for(int i=0; i<nAngles; i++)
	{
		int iim = in_Curve->closed ? 
			(i + in_Curve->nVertices - 1) % in_Curve->nVertices : i;
		int ii = in_Curve->closed ? i : i + 1;
		int iip = in_Curve->closed ? (i + 1) % in_Curve->nVertices : i + 2;

		double restL_m = in_Curve->closed ? in_Curve->restLengths((i + in_Curve->nVertices - 1) % in_Curve->nVertices) : 
			in_Curve->restLengths(i);
		double restL = in_Curve->closed ? in_Curve->restLengths(i) : 
			in_Curve->restLengths(i+1);

		const Eigen::Vector2d e_im = in_Position.col(ii) - in_Position.col(iim);
		const Eigen::Vector2d e_i = in_Position.col(iip) - in_Position.col(ii);

		/*
		double omega = 2.0 * fabs(e_im(0)*e_i(1)-e_im(1)*e_i(0)) / (e_im.norm()*e_i.norm() + e_im.dot(e_i));
		double omega_bar = 2.0 * fabs(tan(in_Curve->restAngles(i)*0.5));
		//*/
		double omega = computeOmega(e_im, e_i);
		double omega_bar = computeOmegaFromAngle(in_Curve->restAngles(i));
		io_f(i+offset_row) = sqrt(in_Params->alpha/(restL_m + restL)) * (omega - omega_bar);
	}
}

void compute_f_fit(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position, Eigen::VectorXd& io_f, int offset_row)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;

	assert(io_f.rows() >= nSegs + offset_row);
	assert(in_Curve->nVertices == in_Position.cols());

	//printf("int_seg: ");
	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_Curve->nVertices;
		double int_df_seg = integrateDF2OverSegment(in_Params, in_Position.col(i), in_Position.col(ip), in_Curve->restLengths(i));
		//printf("%f, ", int_df_seg);
		io_f(i+offset_row) = sqrt(in_Params->fit * 0.5) * sqrt(int_df_seg);
	}
	//printf("\n");
}

void compute_f(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_Vars, Eigen::VectorXd& io_f)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;
	int nAngles = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 2; 

	int nElems = nSegs + nAngles + nSegs;

	assert(io_f.rows() == nElems);
	assert(in_Curve->nVertices == in_Vars.pos.cols());
	assert(in_Curve->nVertices == in_Vars.pos.cols());

	io_f.setZero();

	compute_f_elastic(in_Params, in_Curve, in_Vars.pos, io_f, 0);
	compute_f_bending(in_Params, in_Curve, in_Vars.pos, io_f, nSegs);
	compute_f_fit(in_Params, in_Curve, in_Vars.pos, io_f, nSegs+nAngles);

	/*
	printf("f: [");
	for(int i=0; i<nElems; i++)
	{
		if(i<nElems-1) printf("%f, ", io_f(i));
		else printf("%f]\n", io_f(i));
	}
	//*/
}

void computeNumericalDerivative_elastic(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position, double epsilon, Eigen::MatrixXd& io_Jacobian, int offset_row)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;

	assert(io_Jacobian.rows() >= nSegs + offset_row);
	assert(io_Jacobian.cols() == in_Curve->nVertices * 2);

	const Eigen::Vector2d dx(epsilon, 0.0); 
	const Eigen::Vector2d dy(0.0, epsilon);

	//dJ/dxi, dJ/dyi
	for(int i=0; i<nSegs; i++)
	{
		//l_i is a function of x_i, x_{i+1}, y_i and y_{i+1}
		int ip = (i+1) % in_Curve->nVertices;

		const Eigen::Vector2d xi = in_Position.col(i);
		const Eigen::Vector2d xip = in_Position.col(ip);

		const Eigen::Vector2d diff0 = xip - xi;
		const double fi = sqrt(in_Params->YA * in_Curve->restLengths(i) * 0.5) * (diff0.norm() / in_Curve->restLengths(i) - 1);

		//dJ/dxi_i
		const Eigen::Vector2d xi_dx = xi + dx;
		const Eigen::Vector2d diff1 = xip - xi_dx;
		const double fi_i_dx = sqrt(in_Params->YA * in_Curve->restLengths(i) * 0.5) * (diff1.norm() / in_Curve->restLengths(i) - 1);
		io_Jacobian(i+offset_row, i) = (fi_i_dx - fi) / epsilon;

		//dJ/dyi_i
		const Eigen::Vector2d xi_dy = xi + dy;
		const Eigen::Vector2d diff2 = xip - xi_dy;
		const double fi_i_dy = sqrt(in_Params->YA * in_Curve->restLengths(i) * 0.5) * (diff2.norm() / in_Curve->restLengths(i) - 1);
		io_Jacobian(i+offset_row, i+in_Curve->nVertices) = (fi_i_dy - fi) / epsilon;

		//dJ/dxi_ip
		const Eigen::Vector2d xip_dx = xip + dx;
		const Eigen::Vector2d diff3 = xip_dx - xi;
		const double fi_ip_dx = sqrt(in_Params->YA * in_Curve->restLengths(i) * 0.5) * (diff3.norm() / in_Curve->restLengths(i) - 1);
		io_Jacobian(i+offset_row, ip) = (fi_ip_dx - fi) / epsilon;
		
		//dJ/dyi_i
		const Eigen::Vector2d xip_dy = xip + dy;
		const Eigen::Vector2d diff4 = xip_dy - xi;
		const double fi_ip_dy = sqrt(in_Params->YA * in_Curve->restLengths(i) * 0.5) * (diff4.norm() / in_Curve->restLengths(i) - 1);
		io_Jacobian(i+offset_row, ip+in_Curve->nVertices) = (fi_ip_dy - fi) / epsilon;
	}
}

void computeNumericalDerivative_bending(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position, double epsilon, Eigen::MatrixXd& io_Jacobian, int offset_row)
{
	int nAngles = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 2; 

	assert(io_Jacobian.rows() >= nAngles + offset_row);
	assert(io_Jacobian.cols() == in_Curve->nVertices * 2);

	const Eigen::Vector2d dx(epsilon, 0.0); 
	const Eigen::Vector2d dy(0.0, epsilon);

	//dB/dxi, dB/dyi
	for(int i=0; i<nAngles; i++)
	{
		int iim = in_Curve->closed ? 
			(i + in_Curve->nVertices - 1) % in_Curve->nVertices : i;
		int ii = in_Curve->closed ? i : i + 1;
		int iip = in_Curve->closed ? (i + 1) % in_Curve->nVertices : i + 2;

		double restL_m = in_Curve->restLengths(iim);
		double restL = in_Curve->restLengths(ii);

		const Eigen::Vector2d xim = in_Position.col(iim);
		const Eigen::Vector2d xi = in_Position.col(ii);
		const Eigen::Vector2d xip = in_Position.col(iip);

		const Eigen::Vector2d xim_dx = xim + dx; const Eigen::Vector2d xim_dy = xim + dy;
		const Eigen::Vector2d xi_dx = xi + dx; const Eigen::Vector2d xi_dy = xi + dy;
		const Eigen::Vector2d xip_dx = xip + dx; const Eigen::Vector2d xip_dy = xip + dy;

		const Eigen::Vector2d e_prev = xi - xim;
		const Eigen::Vector2d e_next = xip - xi;

		const Eigen::Vector2d e_prev_im_dx = xi - xim_dx; const Eigen::Vector2d e_prev_im_dy = xi - xim_dy;
		const Eigen::Vector2d e_prev_i_dx = xi_dx - xim; const Eigen::Vector2d e_prev_i_dy = xi_dy - xim;

		const Eigen::Vector2d e_next_i_dx = xip - xi_dx; const Eigen::Vector2d e_next_i_dy = xip - xi_dy;
		const Eigen::Vector2d e_next_ip_dx = xip_dx - xi; const Eigen::Vector2d e_next_ip_dy = xip_dy - xi;

		/*
		const double omega = 2.0 * fabs(e_prev(0)*e_next(1)-e_prev(1)*e_next(0)) / (e_prev.norm()*e_next.norm() + e_prev.dot(e_next));

		const double omega_im_dx = 2.0 * fabs(e_prev_im_dx(0)*e_next(1)-e_prev_im_dx(1)*e_next(0)) / (e_prev_im_dx.norm()*e_next.norm() + e_prev_im_dx.dot(e_next));
		const double omega_im_dy = 2.0 * fabs(e_prev_im_dy(0)*e_next(1)-e_prev_im_dy(1)*e_next(0)) / (e_prev_im_dy.norm()*e_next.norm() + e_prev_im_dy.dot(e_next));
		const double omega_i_dx = 2.0 * fabs(e_prev_i_dx(0)*e_next_i_dx(1)-e_prev_i_dx(1)*e_next_i_dx(0)) / (e_prev_i_dx.norm()*e_next_i_dx.norm() + e_prev_i_dx.dot(e_next_i_dx));
		const double omega_i_dy = 2.0 * fabs(e_prev_i_dy(0)*e_next_i_dy(1)-e_prev_i_dy(1)*e_next_i_dy(0)) / (e_prev_i_dy.norm()*e_next_i_dy.norm() + e_prev_i_dy.dot(e_next_i_dy));
		const double omega_ip_dx = 2.0 * fabs(e_prev(0)*e_next_ip_dx(1)-e_prev(1)*e_next_ip_dx(0)) / (e_prev.norm()*e_next_ip_dx.norm() + e_prev.dot(e_next_ip_dx));
		const double omega_ip_dy = 2.0 * fabs(e_prev(0)*e_next_ip_dy(1)-e_prev(1)*e_next_ip_dy(0)) / (e_prev.norm()*e_next_ip_dy.norm() + e_prev.dot(e_next_ip_dy));
		//*/

		const double omega = computeOmega(e_prev, e_next);

		const double omega_im_dx = computeOmega(e_prev_im_dx, e_next);
		const double omega_im_dy = computeOmega(e_prev_im_dy, e_next);
		const double omega_i_dx = computeOmega(e_prev_i_dx, e_next_i_dx);
		const double omega_i_dy = computeOmega(e_prev_i_dy, e_next_i_dy);
		const double omega_ip_dx = computeOmega(e_prev, e_next_ip_dx);
		const double omega_ip_dy = computeOmega(e_prev, e_next_ip_dy);		

		const double fi = sqrt(in_Params->alpha/(restL_m + restL)) * omega; //omega_bar will be cancelled out, so omit it
		const double fi_im_dx = sqrt(in_Params->alpha/(restL_m + restL)) * omega_im_dx;
		const double fi_im_dy = sqrt(in_Params->alpha/(restL_m + restL)) * omega_im_dy;
		const double fi_i_dx = sqrt(in_Params->alpha/(restL_m + restL)) * omega_i_dx;
		const double fi_i_dy = sqrt(in_Params->alpha/(restL_m + restL)) * omega_i_dy;
		const double fi_ip_dx = sqrt(in_Params->alpha/(restL_m + restL)) * omega_ip_dx;
		const double fi_ip_dy = sqrt(in_Params->alpha/(restL_m + restL)) * omega_ip_dy;
		
		io_Jacobian(i+offset_row, iim) = (fi_im_dx - fi) / epsilon;
		io_Jacobian(i+offset_row, iim+in_Curve->nVertices) = (fi_im_dy - fi) / epsilon;

		io_Jacobian(i+offset_row, ii) = (fi_i_dx - fi) / epsilon;
		io_Jacobian(i+offset_row, ii+in_Curve->nVertices) = (fi_i_dy - fi) / epsilon;

		io_Jacobian(i+offset_row, iip) = (fi_ip_dx - fi) / epsilon;
		io_Jacobian(i+offset_row, iip+in_Curve->nVertices) = (fi_ip_dy - fi) / epsilon;
	}
}

void computeNumericalDerivative_fit(const SParameters* in_Params, const SCurve* in_Curve, const Eigen::Matrix2Xd& in_Position, double epsilon, Eigen::MatrixXd& io_Jacobian, int offset_row)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;

	assert(io_Jacobian.rows() >= nSegs + offset_row);
	assert(io_Jacobian.cols() == in_Curve->nVertices * 2);

	const Eigen::Vector2d dx(epsilon, 0.0); 
	const Eigen::Vector2d dy(0.0, epsilon);

	//dfit/dxi, dfit/dyi
	for(int i=0; i<nSegs; i++)
	{
		//l_i is a function of x_i, x_{i+1}, y_i and y_{i+1}
		int ip = (i+1) % in_Curve->nVertices;

		const Eigen::Vector2d xi = in_Position.col(i);
		const Eigen::Vector2d xip = in_Position.col(ip);
		const double restL = in_Curve->restLengths(i);

		const double fi = sqrt(in_Params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_Params, xi, xip, restL));

		//dfit/dxi_i
		const Eigen::Vector2d xi_dx = xi + dx;
		const double fi_i_dx = sqrt(in_Params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_Params, xi_dx, xip, restL));
		io_Jacobian(i+offset_row, i) = (fi_i_dx - fi) / epsilon;

		//dfit/dyi_i
		const Eigen::Vector2d xi_dy = xi + dy;
		const double fi_i_dy = sqrt(in_Params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_Params, xi_dy, xip, restL));
		io_Jacobian(i+offset_row, i+in_Curve->nVertices) = (fi_i_dy - fi) / epsilon;

		//dfit/dxi_ip
		const Eigen::Vector2d xip_dx = xip + dx;
		const double fi_ip_dx = sqrt(in_Params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_Params, xi, xip_dx, restL));
		io_Jacobian(i+offset_row, ip) = (fi_ip_dx - fi) / epsilon;
		
		//dfit/dyi_i
		const Eigen::Vector2d xip_dy = xip + dy;
		const double fi_ip_dy = sqrt(in_Params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_Params, xi, xip_dy, restL));
		io_Jacobian(i+offset_row, ip+in_Curve->nVertices) = (fi_ip_dy - fi) / epsilon;
	}
}

void computeNumericalDerivative(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_Vars, double epsilon, Eigen::MatrixXd& io_Jacobian)
{
	int nSegs = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 1;
	int nAngles = in_Curve->closed ? in_Curve->nVertices : in_Curve->nVertices - 2; 

	//assert(io_Jacobian.rows() == nSegs + nAngles + nSegs);
	//assert(io_Jacobian.cols() == in_Curve->nVertices * 2);
	//assert(in_Vars.pos.cols() == in_Curve->nVertices);
	//assert(in_Vars.conf.cols() == in_Curve->nVertices);

	io_Jacobian.setZero();

	computeNumericalDerivative_elastic(in_Params, in_Curve, in_Vars.pos, epsilon, io_Jacobian, 0);
	computeNumericalDerivative_bending(in_Params, in_Curve, in_Vars.pos, epsilon, io_Jacobian, nSegs);
	computeNumericalDerivative_fit(in_Params, in_Curve, in_Vars.pos, epsilon, io_Jacobian, nSegs+nAngles);

	/*
	printf("B: [\n");
	for(int j=0; j<io_Jacobian.rows(); j++)
	{
		printf("[");
		for(int i=0; i<io_Jacobian.cols(); i++)
		{
			if(i<io_Jacobian.cols()-1) printf("%f, ", io_Jacobian(j, i));
			else printf("%f],\n", io_Jacobian(j, i));
		}
	}
	printf("]\n");
	//*/
}

void updateRestLength(const Eigen::Matrix2Xd& in_Position, SCurve* io_Curve)
{
	int nSegs = io_Curve->closed ? io_Curve->nVertices : io_Curve->nVertices - 1;
	assert(io_Curve->nVertices == in_Position.cols());

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % io_Curve->nVertices;
		const Eigen::Vector2d diff = in_Position.col(ip) - in_Position.col(i);
		io_Curve->restLengths(i) = diff.norm();
	}
}

void updateCurveSubdivision(const SParameters* in_Params, SVar& io_InitialVars, SVar& io_Vars, SCurve* io_Curve, SSolverVars& io_SolverVars)
{
	int nSegs = io_Curve->closed ? io_Curve->nVertices : io_Curve->nVertices - 1;
	//assert(io_Curve->nVertices == in_Position.cols());

	std::vector<SPoint2D> pos;
	std::vector<double> angles;
	std::vector<int> vids;

	bool subdivided = false;

	for(int i=0; i<nSegs; i++)
	{
		SPoint2D p; p.x[0] = io_Vars.pos.col(i)(0); p.x[1] = io_Vars.pos.col(i)(1);
		pos.push_back(p);
		vids.push_back(io_Curve->vertexIDs(i));

		if(i!=0 || io_Curve->closed) angles.push_back(io_Curve->restAngles(i));

		int ip = (i+1) % io_Curve->nVertices;
		const Eigen::Vector2d diff = io_Vars.pos.col(ip) - io_Vars.pos.col(i);
		if(diff.norm() > in_Params->refLength)
		{
			SPoint2D p;
			p.x[0] = (io_Vars.pos.col(ip)(0) + io_Vars.pos.col(i)(0)) * 0.5;
			p.x[1] = (io_Vars.pos.col(ip)(1) + io_Vars.pos.col(i)(1)) * 0.5;
			pos.push_back(p);
			//angles.push_back((io_Curve->restAngles(i) + io_Curve->restAngles(ip)) * 0.5);
			angles.push_back(PI);
			vids.push_back(-1);
			subdivided = true;
		}
	}

	if(!io_Curve->closed)
	{
		int ilast = io_Curve->nVertices-1;
		SPoint2D p; p.x[0] = io_Vars.pos.col(ilast)(0); p.x[1] = io_Vars.pos.col(ilast)(1);
		pos.push_back(p);
		vids.push_back(io_Curve->vertexIDs(ilast));
	}

	io_Curve->nVertices = pos.size();
	nSegs = io_Curve->closed ? io_Curve->nVertices : io_Curve->nVertices - 1;
	io_Curve->restLengths.resize(nSegs);
	int nAngles = io_Curve->closed ? io_Curve->nVertices : io_Curve->nVertices - 2; 
	io_Curve->restAngles.resize(nAngles);
	io_Curve->vertexIDs.resize(io_Curve->nVertices);

	resize(io_Vars, io_Curve->nVertices);
	resize(io_InitialVars, io_Curve->nVertices);

	for(int i=0; i<io_Curve->nVertices; i++)
	{
		io_Vars.pos.col(i)(0) = pos[i].x[0];
		io_Vars.pos.col(i)(1) = pos[i].x[1];
		io_Vars.conf(i) = 0.0;
		io_Curve->vertexIDs(i) = vids[i];
	}

	nSegs = io_Curve->closed ? io_Curve->nVertices : io_Curve->nVertices - 1;
	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % io_Curve->nVertices;
		const Eigen::Vector2d diff = io_Vars.pos.col(ip) - io_Vars.pos.col(i);
		io_Curve->restLengths(i) = diff.norm();
	}

	for(int i=0; i<nAngles; i++)
		io_Curve->restAngles(i) = angles[i];

	io_InitialVars = io_Vars;

	initSolverVars(in_Params, io_Curve, io_InitialVars, io_SolverVars, true);
}

bool secantLMMethodSingleUpdate(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_InitialVars, SSolverVars& io_SolverVars, SVar& solution)
{
	if(io_SolverVars.found || io_SolverVars.k > in_Params->kmax)
		return true;

	io_SolverVars.k++;
	io_SolverVars.A_muI = io_SolverVars.B.transpose() * io_SolverVars.B + io_SolverVars.mu * io_SolverVars.I;
	io_SolverVars.h = io_SolverVars.A_muI.ldlt().solve(-io_SolverVars.g);

	if(io_SolverVars.h.norm() <= in_Params->epsilon_2 * (io_SolverVars.x.pos.norm() + in_Params->epsilon_2))
		io_SolverVars.found = true;
	else
	{
		for(int q=0; q<io_SolverVars.nV; q++)
		{
			io_SolverVars.xnew.pos(0, q) = io_SolverVars.x.pos(0, q) + io_SolverVars.h(q);
			io_SolverVars.xnew.pos(1, q) = io_SolverVars.x.pos(1, q) + io_SolverVars.h(q + io_SolverVars.nV);
		}
	}
			
	double Fnew = 0.5 * computeEnergy(in_Params, in_Curve, io_SolverVars.xnew);
	double F = 0.5 * computeEnergy(in_Params, in_Curve, io_SolverVars.x);

	double gain_denom = - io_SolverVars.h.dot(io_SolverVars.B.transpose() * io_SolverVars.f) 
		- 0.5 * io_SolverVars.h.dot(io_SolverVars.B.transpose() * io_SolverVars.B * io_SolverVars.h);
	double gain = (F - Fnew) / gain_denom;

	if(gain > 0)
	{
		io_SolverVars.x = io_SolverVars.xnew;
		compute_f(in_Params, in_Curve, io_SolverVars.x, io_SolverVars.f);
		computeNumericalDerivative(in_Params, in_Curve, io_SolverVars.x, io_SolverVars.epsilon, io_SolverVars.B);
		io_SolverVars.g = io_SolverVars.B.transpose() * io_SolverVars.f;
		io_SolverVars.found = (io_SolverVars.g.lpNorm<Eigen::Infinity>() <= in_Params->epsilon_1);
		printf("k: %d, gain: %f, |g|_inf: %f\n", io_SolverVars.k, gain, io_SolverVars.g.lpNorm<Eigen::Infinity>());
		io_SolverVars.mu = io_SolverVars.mu * std::max(1.0/3.0, 1.0 - (2.0 * gain - 1.0) * (2.0 * gain - 1.0) * (2.0 * gain - 1.0));
		io_SolverVars.nu = 2.0;
	}
	else
	{
		io_SolverVars.mu = io_SolverVars.mu * io_SolverVars.nu;
		io_SolverVars.nu = io_SolverVars.nu * 2.0;
	}

	if(io_SolverVars.found)
		printf("found in %d steps\n", io_SolverVars.k);

	solution = io_SolverVars.x;

	return io_SolverVars.found || io_SolverVars.k > in_Params->kmax;
}

void showFeaturePoints(const SCurve* in_Curve, const SVar& solution)
{
	printf("Feature points:\n");
	for(int i=0; i<in_Curve->nVertices; i++)
	{
		if(in_Curve->vertexIDs(i) >= 0)
		{
			printf("%d: %f, %f\n", in_Curve->vertexIDs(i), solution.pos.col(i).x(), solution.pos.col(i).y());
		}
	}
}

void secantLMMethod(const SParameters* in_Params, SCurve* in_Curve, SVar& in_InitialVars, SSolverVars& io_SolverVars, SVar& solution)
{
	initSolverVars(in_Params, in_Curve, in_InitialVars, io_SolverVars);

	while(1)
	{
		if(secantLMMethodSingleUpdate(in_Params, in_Curve, in_InitialVars, io_SolverVars, solution))
			break;
		updateCurveSubdivision(in_Params, in_InitialVars, solution, in_Curve, io_SolverVars);
	}

	printf("found in %d steps\n", io_SolverVars.k);
	solution = io_SolverVars.x;
	showFeaturePoints(in_Curve, solution);
}
