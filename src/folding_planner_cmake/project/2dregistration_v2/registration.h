#ifndef __REGISTRATION_H__
#define __REGISTRATION_H__

#include "datatypes.h"
#include "distancefield.h"

double computeAngle(const Eigen::Vector2d& e_im, const Eigen::Vector2d& e_i);
double computeOmegaFromAngle(const double& theta);

double computeEnergy(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_Vars);
//void setupMatrix(const SParameters* in_Params, const SCurve* in_Curve);

void compute_f(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_Vars, Eigen::VectorXd& io_f);
void computeNumericalDerivative(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_Vars, double epsilon, Eigen::MatrixXd& io_Jacobian);

void updateRestLength(const Eigen::Matrix2Xd& in_Position, SCurve* io_Curve);
void updateCurveSubdivision(const SParameters* in_Params, SVar& io_InitialVars, SVar& io_Vars, SCurve* io_Curve, SSolverVars& io_SolverVars);

void secantLMMethod(const SParameters* in_Params, SCurve* in_Curve, SVar& in_InitialVars, SSolverVars& io_SolverVars, SVar& solution);
bool secantLMMethodSingleUpdate(const SParameters* in_Params, const SCurve* in_Curve, const SVar& in_InitialVars, SSolverVars& io_SolverVars, SVar& solution);

void showFeaturePoints(const SCurve* in_Curve, const SVar& solution);

#endif
