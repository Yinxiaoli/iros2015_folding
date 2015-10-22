#include "registration.h"
#include <Eigen/Cholesky>
#include <vector>

struct SPoint2D
{
	double x[2];
};

double ComputeAngle(const Eigen::Vector2d& input_one, const Eigen::Vector2d& input_two)
{
	//compute theta
	//when e_im.dot(e_i) = 1.0, theta = pi;
	const double r_sinT = input_one(0)*input_two(1) - input_one(1)*input_two(0);
	const double r_cosT = -input_one.dot(input_two);
	return atan2(r_sinT, r_cosT);
}

double sgn(const double& x)
{
	return (x >= 0) ? 1.0 : -1.0;
}


double ComputeOmegaFromAngle(const double& theta)
{
	const double sinT = sin(theta);
	const double cosT = cos(theta);
	const double tan_phi_over_2 = sgn(sinT) * sqrt((1.0+cosT)/(max(0.0, 1.0-cosT) + 1.0e-10));
	return 2.0 * tan_phi_over_2;
}

double computeOmega(const Eigen::Vector2d& input_one, const Eigen::Vector2d& input_two)
{
	//compute 2.0 * tan(phi / 2.0)
	//phi = pi - theta
	return ComputeOmegaFromAngle(ComputeAngle(input_one, input_two));	
}

double sampleDistanceField(const SImage<double, 1>& in_df, const SRegion& in_region, const Eigen::Vector2d& x)
{
	assert(in_df.width == in_df.height);
	assert(in_region.right-in_region.left == in_region.top-in_region.bottom);

	Eigen::Vector2d proj_x = x;
	proj_x(0) = std::max(in_region.left, std::min(in_region.right, x(0)));
	proj_x(1) = std::max(in_region.bottom, std::min(in_region.top, x(1)));

	const double dist_scale = (in_region.right-in_region.left) / in_df.width;

	double fi = (proj_x(0) - in_region.left) * in_df.width / (in_region.right - in_region.left);
	double fj = (proj_x(1) - in_region.bottom) * in_df.height / (in_region.top - in_region.bottom);
	double _i = floor(fi);
	double _j = floor(fj);

	int i = std::max(0, std::min(in_df.width-1, int(_i)));
	int j = std::max(0, std::min(in_df.height-1, int(_j)));
	int ip = std::min(in_df.width-1, i+1);
	int jp = std::min(in_df.height-1, j+1);

	double s = fi - _i;
	double t = fj - _j;

	double dist_proj = (1.0 - t) * (s * in_df.ptr[j*in_df.width+ip] + (1.0-s) * in_df.ptr[j*in_df.width+i])
		+ t * (s * in_df.ptr[jp*in_df.width+ip] + (1.0-s) * in_df.ptr[jp*in_df.width+i]);

	return dist_proj * dist_scale + (proj_x - x).norm();
}

double integrateDF2OverSegment(const SParameters* in_params, const Eigen::Vector2d& x1, const Eigen::Vector2d& x2, const double rest_length)
{
	double tot = 0.0;
	const double len = rest_length / in_params->substep_fit;

	//printf("integ_DF: ");
	for(int i=0; i<in_params->substep_fit; i++)
	{
		const Eigen::Vector2d& p1 = x1 + (x2-x1) * double(i)/double(in_params->substep_fit);
		const Eigen::Vector2d& p2 = x1 + (x2-x1) * double(i+1)/double(in_params->substep_fit);
		
		const double d1 = sampleDistanceField(in_params->df, in_params->region, p1);
		const double d2 = sampleDistanceField(in_params->df, in_params->region, p2);

		//printf("[%f, %f, %f], ", d1, d2, len);

		const double e1 = 0.5 * (exp(d1) + exp(-d1)) - 1.0;
		const double e2 = 0.5 * (exp(d2) + exp(-d2)) - 1.0;

		//tot += 0.5 * (d1*d1+d2*d2) * len;
		tot += 0.5 * (e1*e1+e2*e2) * len;
	}
	//printf("\n");

	return tot;
}

inline double computeSegmentElasticEnergy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const double& YA, const double& rest_length)
{
	const Eigen::Vector2d diff = p1 - p2;
	return 0.5 * YA * rest_length
		* (diff.norm() / rest_length - 1)
		* (diff.norm() / rest_length - 1);
}

inline double computeSegmentBendingEnergy(const Eigen::Vector2d& pp, const Eigen::Vector2d& pc, const Eigen::Vector2d& pn, 
	const double& alpha, const double& rest_length_p, const double& rest_length_n, const double& theta)
{
	const Eigen::Vector2d e_im = pc - pp;
	const Eigen::Vector2d e_i = pn - pc;

	/*
	double omega = 2.0 * fabs(e_im(0)*e_i(1)-e_im(1)*e_i(0)) / (e_im.norm()*e_i.norm() + e_im.dot(e_i));
	double omega_bar = 2.0 * fabs(tan(phi*0.5));
	//*/
	double omega = computeOmega(e_im, e_i);
	double omega_bar = ComputeOmegaFromAngle(theta);
	return alpha * (omega - omega_bar) * (omega - omega_bar) / (rest_length_p + rest_length_n);
}

double computeEnergy_elastic(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;

	double energy = 0.0;

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_curve->nVertices;
		energy += computeSegmentElasticEnergy(in_position.col(ip), in_position.col(i), in_params->YA, in_curve->restLengths(i));
	}

	return energy;
}

double computeEnergy_bending(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position)
{
	int nAngles = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 2; 

	double energy = 0.0;

	for(int i=0; i<nAngles; i++)
	{
		int iim = in_curve->closed ? 
			(i + in_curve->nVertices - 1) % in_curve->nVertices : i;
		int ii = in_curve->closed ? i : i + 1;
		int iip = in_curve->closed ? (i + 1) % in_curve->nVertices : i + 2;

		double restL_m = in_curve->closed ? in_curve->restLengths((i + in_curve->nVertices - 1) % in_curve->nVertices) : 
			in_curve->restLengths(i);
		double restL = in_curve->closed ? in_curve->restLengths(i) : 
			in_curve->restLengths(i+1);

		energy += computeSegmentBendingEnergy(in_position.col(iim), in_position.col(ii), in_position.col(iip),
			in_params->alpha, restL_m, restL, in_curve->restAngles(i));
	}

	return energy;
}

double computeEnergy_fit(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;

	double energy = 0.0;

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_curve->nVertices;
		double int_df_seg = integrateDF2OverSegment(in_params, in_position.col(i), in_position.col(ip), in_curve->restLengths(i));
		energy += 0.5 * in_params->fit * int_df_seg;
	}

	return energy;
}

double ComputeEnergy(const SParameters* in_params, const SCurve* in_curve, const SVar& in_vars)
{
	const double E_elastic = computeEnergy_elastic(in_params, in_curve, in_vars.pos);
	const double E_bending = computeEnergy_bending(in_params, in_curve, in_vars.pos);
	const double E_fit = computeEnergy_fit(in_params, in_curve, in_vars.pos);
	return E_elastic + E_bending + E_fit;
}

void compute_f_elastic(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position, Eigen::VectorXd& io_f, int offset_row)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;

	assert(io_f.rows() >= nSegs + offset_row);
	assert(in_curve->nVertices == in_position.cols());

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_curve->nVertices;
		const Eigen::Vector2d diff = in_position.col(ip) - in_position.col(i);
		io_f(i+offset_row) = sqrt(0.5 * in_params->YA * in_curve->restLengths(i)) * (diff.norm() / in_curve->restLengths(i) - 1);
	}
}

void compute_f_bending(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position, Eigen::VectorXd& io_f, int offset_row)
{
	int nAngles = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 2; 

	assert(io_f.rows() >= nAngles + offset_row);
	assert(in_curve->nVertices == in_position.cols());

	for(int i=0; i<nAngles; i++)
	{
		int iim = in_curve->closed ? 
			(i + in_curve->nVertices - 1) % in_curve->nVertices : i;
		int ii = in_curve->closed ? i : i + 1;
		int iip = in_curve->closed ? (i + 1) % in_curve->nVertices : i + 2;

		double restL_m = in_curve->closed ? in_curve->restLengths((i + in_curve->nVertices - 1) % in_curve->nVertices) : 
			in_curve->restLengths(i);
		double restL = in_curve->closed ? in_curve->restLengths(i) : 
			in_curve->restLengths(i+1);

		const Eigen::Vector2d e_im = in_position.col(ii) - in_position.col(iim);
		const Eigen::Vector2d e_i = in_position.col(iip) - in_position.col(ii);

		/*
		double omega = 2.0 * fabs(e_im(0)*e_i(1)-e_im(1)*e_i(0)) / (e_im.norm()*e_i.norm() + e_im.dot(e_i));
		double omega_bar = 2.0 * fabs(tan(in_curve->restAngles(i)*0.5));
		//*/
		double omega = computeOmega(e_im, e_i);
		double omega_bar = ComputeOmegaFromAngle(in_curve->restAngles(i));
		io_f(i+offset_row) = sqrt(in_params->alpha/(restL_m + restL)) * (omega - omega_bar);
	}
}

void compute_f_fit(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position, Eigen::VectorXd& io_f, int offset_row)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;

	assert(io_f.rows() >= nSegs + offset_row);
	assert(in_curve->nVertices == in_position.cols());

	//printf("int_seg: ");
	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % in_curve->nVertices;
		double int_df_seg = integrateDF2OverSegment(in_params, in_position.col(i), in_position.col(ip), in_curve->restLengths(i));
		//printf("%f, ", int_df_seg);
		io_f(i+offset_row) = sqrt(in_params->fit * 0.5) * sqrt(int_df_seg);
	}
	//printf("\n");
}

void Compute_f(const SParameters* in_params, const SCurve* in_curve, const SVar& in_vars, Eigen::VectorXd& io_f)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;
	int nAngles = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 2; 

	int nElems = nSegs + nAngles + nSegs;

	assert(io_f.rows() == nElems);
	assert(in_curve->nVertices == in_vars.pos.cols());
	assert(in_curve->nVertices == in_vars.pos.cols());

	io_f.setZero();

	compute_f_elastic(in_params, in_curve, in_vars.pos, io_f, 0);
	compute_f_bending(in_params, in_curve, in_vars.pos, io_f, nSegs);
	compute_f_fit(in_params, in_curve, in_vars.pos, io_f, nSegs+nAngles);

	/*
	printf("f: [");
	for(int i=0; i<nElems; i++)
	{
		if(i<nElems-1) printf("%f, ", io_f(i));
		else printf("%f]\n", io_f(i));
	}
	//*/
}

void computeNumericalDerivative_elastic(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position, double epsilon, Eigen::MatrixXd& io_jacobian, int offset_row)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;

	assert(io_jacobian.rows() >= nSegs + offset_row);
	assert(io_jacobian.cols() == in_curve->nVertices * 2);

	const Eigen::Vector2d dx(epsilon, 0.0); 
	const Eigen::Vector2d dy(0.0, epsilon);

	//dJ/dxi, dJ/dyi
	for(int i=0; i<nSegs; i++)
	{
		//l_i is a function of x_i, x_{i+1}, y_i and y_{i+1}
		int ip = (i+1) % in_curve->nVertices;

		const Eigen::Vector2d xi = in_position.col(i);
		const Eigen::Vector2d xip = in_position.col(ip);

		const Eigen::Vector2d diff0 = xip - xi;
		const double fi = sqrt(in_params->YA * in_curve->restLengths(i) * 0.5) * (diff0.norm() / in_curve->restLengths(i) - 1);

		//dJ/dxi_i
		const Eigen::Vector2d xi_dx = xi + dx;
		const Eigen::Vector2d diff1 = xip - xi_dx;
		const double fi_i_dx = sqrt(in_params->YA * in_curve->restLengths(i) * 0.5) * (diff1.norm() / in_curve->restLengths(i) - 1);
		io_jacobian(i+offset_row, i) = (fi_i_dx - fi) / epsilon;

		//dJ/dyi_i
		const Eigen::Vector2d xi_dy = xi + dy;
		const Eigen::Vector2d diff2 = xip - xi_dy;
		const double fi_i_dy = sqrt(in_params->YA * in_curve->restLengths(i) * 0.5) * (diff2.norm() / in_curve->restLengths(i) - 1);
		io_jacobian(i+offset_row, i+in_curve->nVertices) = (fi_i_dy - fi) / epsilon;

		//dJ/dxi_ip
		const Eigen::Vector2d xip_dx = xip + dx;
		const Eigen::Vector2d diff3 = xip_dx - xi;
		const double fi_ip_dx = sqrt(in_params->YA * in_curve->restLengths(i) * 0.5) * (diff3.norm() / in_curve->restLengths(i) - 1);
		io_jacobian(i+offset_row, ip) = (fi_ip_dx - fi) / epsilon;
		
		//dJ/dyi_i
		const Eigen::Vector2d xip_dy = xip + dy;
		const Eigen::Vector2d diff4 = xip_dy - xi;
		const double fi_ip_dy = sqrt(in_params->YA * in_curve->restLengths(i) * 0.5) * (diff4.norm() / in_curve->restLengths(i) - 1);
		io_jacobian(i+offset_row, ip+in_curve->nVertices) = (fi_ip_dy - fi) / epsilon;
	}
}

void computeNumericalDerivative_bending(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position, double epsilon, Eigen::MatrixXd& io_jacobian, int offset_row)
{
	int nAngles = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 2; 

	assert(io_jacobian.rows() >= nAngles + offset_row);
	assert(io_jacobian.cols() == in_curve->nVertices * 2);

	const Eigen::Vector2d dx(epsilon, 0.0); 
	const Eigen::Vector2d dy(0.0, epsilon);

	//dB/dxi, dB/dyi
	for(int i=0; i<nAngles; i++)
	{
		int iim = in_curve->closed ? 
			(i + in_curve->nVertices - 1) % in_curve->nVertices : i;
		int ii = in_curve->closed ? i : i + 1;
		int iip = in_curve->closed ? (i + 1) % in_curve->nVertices : i + 2;

		double restL_m = in_curve->restLengths(iim);
		double restL = in_curve->restLengths(ii);

		const Eigen::Vector2d xim = in_position.col(iim);
		const Eigen::Vector2d xi = in_position.col(ii);
		const Eigen::Vector2d xip = in_position.col(iip);

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

		const double fi = sqrt(in_params->alpha/(restL_m + restL)) * omega; //omega_bar will be cancelled out, so omit it
		const double fi_im_dx = sqrt(in_params->alpha/(restL_m + restL)) * omega_im_dx;
		const double fi_im_dy = sqrt(in_params->alpha/(restL_m + restL)) * omega_im_dy;
		const double fi_i_dx = sqrt(in_params->alpha/(restL_m + restL)) * omega_i_dx;
		const double fi_i_dy = sqrt(in_params->alpha/(restL_m + restL)) * omega_i_dy;
		const double fi_ip_dx = sqrt(in_params->alpha/(restL_m + restL)) * omega_ip_dx;
		const double fi_ip_dy = sqrt(in_params->alpha/(restL_m + restL)) * omega_ip_dy;
		
		io_jacobian(i+offset_row, iim) = (fi_im_dx - fi) / epsilon;
		io_jacobian(i+offset_row, iim+in_curve->nVertices) = (fi_im_dy - fi) / epsilon;

		io_jacobian(i+offset_row, ii) = (fi_i_dx - fi) / epsilon;
		io_jacobian(i+offset_row, ii+in_curve->nVertices) = (fi_i_dy - fi) / epsilon;

		io_jacobian(i+offset_row, iip) = (fi_ip_dx - fi) / epsilon;
		io_jacobian(i+offset_row, iip+in_curve->nVertices) = (fi_ip_dy - fi) / epsilon;
	}
}

void computeNumericalDerivative_fit(const SParameters* in_params, const SCurve* in_curve, const Eigen::Matrix2Xd& in_position, double epsilon, Eigen::MatrixXd& io_jacobian, int offset_row)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;

	assert(io_jacobian.rows() >= nSegs + offset_row);
	assert(io_jacobian.cols() == in_curve->nVertices * 2);

	const Eigen::Vector2d dx(epsilon, 0.0); 
	const Eigen::Vector2d dy(0.0, epsilon);

	//dfit/dxi, dfit/dyi
	for(int i=0; i<nSegs; i++)
	{
		//l_i is a function of x_i, x_{i+1}, y_i and y_{i+1}
		int ip = (i+1) % in_curve->nVertices;

		const Eigen::Vector2d xi = in_position.col(i);
		const Eigen::Vector2d xip = in_position.col(ip);
		const double restL = in_curve->restLengths(i);

		const double fi = sqrt(in_params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_params, xi, xip, restL));

		//dfit/dxi_i
		const Eigen::Vector2d xi_dx = xi + dx;
		const double fi_i_dx = sqrt(in_params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_params, xi_dx, xip, restL));
		io_jacobian(i+offset_row, i) = (fi_i_dx - fi) / epsilon;

		//dfit/dyi_i
		const Eigen::Vector2d xi_dy = xi + dy;
		const double fi_i_dy = sqrt(in_params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_params, xi_dy, xip, restL));
		io_jacobian(i+offset_row, i+in_curve->nVertices) = (fi_i_dy - fi) / epsilon;

		//dfit/dxi_ip
		const Eigen::Vector2d xip_dx = xip + dx;
		const double fi_ip_dx = sqrt(in_params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_params, xi, xip_dx, restL));
		io_jacobian(i+offset_row, ip) = (fi_ip_dx - fi) / epsilon;
		
		//dfit/dyi_i
		const Eigen::Vector2d xip_dy = xip + dy;
		const double fi_ip_dy = sqrt(in_params->fit * 0.5) * sqrt(integrateDF2OverSegment(in_params, xi, xip_dy, restL));
		io_jacobian(i+offset_row, ip+in_curve->nVertices) = (fi_ip_dy - fi) / epsilon;
	}
}

void ComputeNumericalDerivative(const SParameters* in_params, const SCurve* in_curve, const SVar& in_vars, double epsilon, Eigen::MatrixXd& io_jacobian)
{
	int nSegs = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 1;
	int nAngles = in_curve->closed ? in_curve->nVertices : in_curve->nVertices - 2; 

	//assert(io_jacobian.rows() == nSegs + nAngles + nSegs);
	//assert(io_jacobian.cols() == in_curve->nVertices * 2);
	//assert(in_vars.pos.cols() == in_curve->nVertices);
	//assert(in_vars.conf.cols() == in_curve->nVertices);

	io_jacobian.setZero();

	computeNumericalDerivative_elastic(in_params, in_curve, in_vars.pos, epsilon, io_jacobian, 0);
	computeNumericalDerivative_bending(in_params, in_curve, in_vars.pos, epsilon, io_jacobian, nSegs);
	computeNumericalDerivative_fit(in_params, in_curve, in_vars.pos, epsilon, io_jacobian, nSegs+nAngles);

	/*
	printf("B: [\n");
	for(int j=0; j<io_jacobian.rows(); j++)
	{
		printf("[");
		for(int i=0; i<io_jacobian.cols(); i++)
		{
			if(i<io_jacobian.cols()-1) printf("%f, ", io_jacobian(j, i));
			else printf("%f],\n", io_jacobian(j, i));
		}
	}
	printf("]\n");
	//*/
}

void UpdateRestLength(const Eigen::Matrix2Xd& in_position, SCurve* io_curve)
{
	int nSegs = io_curve->closed ? io_curve->nVertices : io_curve->nVertices - 1;
	assert(io_curve->nVertices == in_position.cols());

	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % io_curve->nVertices;
		const Eigen::Vector2d diff = in_position.col(ip) - in_position.col(i);
		io_curve->restLengths(i) = diff.norm();
	}
}

void UpdateCurveSubdivision(const SParameters* in_params, SVar& io_initial_vars, SVar& io_vars, SCurve* io_curve, SSolverVars& io_solver_vars)
{
	int nSegs = io_curve->closed ? io_curve->nVertices : io_curve->nVertices - 1;
	//assert(io_curve->nVertices == in_position.cols());

	std::vector<SPoint2D> pos;
	std::vector<double> angles;
	std::vector<int> vids;

	bool subdivided = false;

	for(int i=0; i<nSegs; i++)
	{
		SPoint2D p; p.x[0] = io_vars.pos.col(i)(0); p.x[1] = io_vars.pos.col(i)(1);
		pos.push_back(p);
		vids.push_back(io_curve->vertexIDs(i));

		if(i!=0 || io_curve->closed) angles.push_back(io_curve->restAngles(i));

		int ip = (i+1) % io_curve->nVertices;
		const Eigen::Vector2d diff = io_vars.pos.col(ip) - io_vars.pos.col(i);
		if(diff.norm() > in_params->refLength)
		{
			SPoint2D p;
			p.x[0] = (io_vars.pos.col(ip)(0) + io_vars.pos.col(i)(0)) * 0.5;
			p.x[1] = (io_vars.pos.col(ip)(1) + io_vars.pos.col(i)(1)) * 0.5;
			pos.push_back(p);
			//angles.push_back((io_curve->restAngles(i) + io_curve->restAngles(ip)) * 0.5);
			angles.push_back(PI);
			vids.push_back(-1);
			subdivided = true;
		}
	}

	if(!io_curve->closed)
	{
		int ilast = io_curve->nVertices-1;
		SPoint2D p; p.x[0] = io_vars.pos.col(ilast)(0); p.x[1] = io_vars.pos.col(ilast)(1);
		pos.push_back(p);
		vids.push_back(io_curve->vertexIDs(ilast));
	}

	io_curve->nVertices = pos.size();
	nSegs = io_curve->closed ? io_curve->nVertices : io_curve->nVertices - 1;
	io_curve->restLengths.resize(nSegs);
	int nAngles = io_curve->closed ? io_curve->nVertices : io_curve->nVertices - 2; 
	io_curve->restAngles.resize(nAngles);
	io_curve->vertexIDs.resize(io_curve->nVertices);

	resize(io_vars, io_curve->nVertices);
	resize(io_initial_vars, io_curve->nVertices);

	for(int i=0; i<io_curve->nVertices; i++)
	{
		io_vars.pos.col(i)(0) = pos[i].x[0];
		io_vars.pos.col(i)(1) = pos[i].x[1];
		io_vars.conf(i) = 0.0;
		io_curve->vertexIDs(i) = vids[i];
	}

	nSegs = io_curve->closed ? io_curve->nVertices : io_curve->nVertices - 1;
	for(int i=0; i<nSegs; i++)
	{
		int ip = (i+1) % io_curve->nVertices;
		const Eigen::Vector2d diff = io_vars.pos.col(ip) - io_vars.pos.col(i);
		io_curve->restLengths(i) = diff.norm();
	}

	for(int i=0; i<nAngles; i++)
		io_curve->restAngles(i) = angles[i];

	io_initial_vars = io_vars;

	initSolverVars(in_params, io_curve, io_initial_vars, io_solver_vars, true);
}

bool SecantLMMethodSingleUpdate(const SParameters* in_params, const SCurve* in_curve, const SVar& in_initial_vars, SSolverVars& io_solver_vars, SVar& solution)
{
	if(io_solver_vars.found || io_solver_vars.k > in_params->kmax)
		return true;

	io_solver_vars.k++;
	io_solver_vars.A_muI = io_solver_vars.B.transpose() * io_solver_vars.B + io_solver_vars.mu * io_solver_vars.I;
	io_solver_vars.h = io_solver_vars.A_muI.ldlt().solve(-io_solver_vars.g);

	if(io_solver_vars.h.norm() <= in_params->epsilon_2 * (io_solver_vars.x.pos.norm() + in_params->epsilon_2))
		io_solver_vars.found = true;
	else
	{
		for(int q=0; q<io_solver_vars.nV; q++)
		{
			io_solver_vars.xnew.pos(0, q) = io_solver_vars.x.pos(0, q) + io_solver_vars.h(q);
			io_solver_vars.xnew.pos(1, q) = io_solver_vars.x.pos(1, q) + io_solver_vars.h(q + io_solver_vars.nV);
		}
	}
			
	double Fnew = 0.5 * ComputeEnergy(in_params, in_curve, io_solver_vars.xnew);
	double F = 0.5 * ComputeEnergy(in_params, in_curve, io_solver_vars.x);

	double gain_denom = - io_solver_vars.h.dot(io_solver_vars.B.transpose() * io_solver_vars.f) 
		- 0.5 * io_solver_vars.h.dot(io_solver_vars.B.transpose() * io_solver_vars.B * io_solver_vars.h);
	double gain = (F - Fnew) / gain_denom;

	if(gain > 0)
	{
		io_solver_vars.x = io_solver_vars.xnew;
		Compute_f(in_params, in_curve, io_solver_vars.x, io_solver_vars.f);
		ComputeNumericalDerivative(in_params, in_curve, io_solver_vars.x, io_solver_vars.epsilon, io_solver_vars.B);
		io_solver_vars.g = io_solver_vars.B.transpose() * io_solver_vars.f;
		io_solver_vars.found = (io_solver_vars.g.lpNorm<Eigen::Infinity>() <= in_params->epsilon_1);
		printf("k: %d, gain: %f, |g|_inf: %f\n", io_solver_vars.k, gain, io_solver_vars.g.lpNorm<Eigen::Infinity>());
		io_solver_vars.mu = io_solver_vars.mu * std::max(1.0/3.0, 1.0 - (2.0 * gain - 1.0) * (2.0 * gain - 1.0) * (2.0 * gain - 1.0));
		io_solver_vars.nu = 2.0;
	}
	else
	{
		io_solver_vars.mu = io_solver_vars.mu * io_solver_vars.nu;
		io_solver_vars.nu = io_solver_vars.nu * 2.0;
	}

	if(io_solver_vars.found)
		printf("found in %d steps\n", io_solver_vars.k);

	solution = io_solver_vars.x;

	return io_solver_vars.found || io_solver_vars.k > in_params->kmax;
}

void ShowFeaturePoints(const SCurve* in_curve, const SVar& solution)
{
	printf("Feature points:\n");
	for(int i=0; i<in_curve->nVertices; i++)
	{
		if(in_curve->vertexIDs(i) >= 0)
		{
			printf("%d: %f, %f\n", in_curve->vertexIDs(i), solution.pos.col(i).x(), solution.pos.col(i).y());
		}
	}
}

void SecantLMMethod(const SParameters* in_params, SCurve* in_curve, SVar& in_initial_vars, SSolverVars& io_solver_vars, SVar& solution)
{
	initSolverVars(in_params, in_curve, in_initial_vars, io_solver_vars);

	while(1)
	{
		if(SecantLMMethodSingleUpdate(in_params, in_curve, in_initial_vars, io_solver_vars, solution))
			break;
		UpdateCurveSubdivision(in_params, in_initial_vars, solution, in_curve, io_solver_vars);
	}

	printf("found in %d steps\n", io_solver_vars.k);
	solution = io_solver_vars.x;
	ShowFeaturePoints(in_curve, solution);
}
