// A set of functions that implement the computations of the
// E_fit, E_bend, and E_stretch in the paper.

#ifndef __REGISTRATION_H__
#define __REGISTRATION_H__

#include "datatypes.h"
#include "distancefield.h"

double ComputeAngle(const Eigen::Vector2d& input_one, const Eigen::Vector2d& input_two);
double ComputeOmegaFromAngle(const double& theta);

double ComputeEnergy(const SParameters* in_params, const SCurve* in_curve, const SVar& in_vars);
//void setupMatrix(const SParameters* in_params, const SCurve* in_curve);

void Compute_f(const SParameters* in_params, const SCurve* in_curve, const SVar& in_vars, Eigen::VectorXd& io_f);
void ComputeNumericalDerivative(const SParameters* in_params, const SCurve* in_curve, const SVar& in_vars, double epsilon, Eigen::MatrixXd& io_Jacobian);

void UpdateRestLength(const Eigen::Matrix2Xd& in_position, SCurve* io_Curve);
void UpdateCurveSubdivision(const SParameters* in_params, SVar& io_InitialVars, SVar& io_vars, SCurve* io_Curve, SSolverVars& io_solver_vars);

void SecantLMMethod(const SParameters* in_params, SCurve* in_curve, SVar& in_initial_vars, SSolverVars& io_solver_vars, SVar& solution);
bool SecantLMMethodSingleUpdate(const SParameters* in_params, const SCurve* in_curve, const SVar& in_initial_vars, SSolverVars& io_solver_vars, SVar& solution);

void ShowFeaturePoints(const SCurve* in_curve, const SVar& solution);

#endif
