#include "kdl_planner.h"

KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration,
                       Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd,
                       double _trajRadius,
                       PathType _pathType,
                       VelProfileType _velProfileType)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    trajRadius_ = _trajRadius;
    pathType_ = _pathType;
    velProfileType_ = _velProfileType;
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

void KDLPlanner::trapezoidal_vel(double t, double t_c, double &s, double &s_dot, double &s_ddot)
{
    double t_f = trajDuration_;
    double _s_ddot_c = 1.0 / (t_c * (t_f - t_c));

    if (t >= 0 && t <= t_c) 
    {
        s = 0.5 * _s_ddot_c * pow(t, 2);
        s_dot = _s_ddot_c * t;
        s_ddot = _s_ddot_c;
    }
    else if (t > t_c && t <= t_f - t_c)
    {
        s = _s_ddot_c * t_c * (t - t_c/2);
        s_dot = _s_ddot_c * t_c;
        s_ddot = 0.0;
    }
    else if(t > t_f - t_c && t <= t_f)
    {
        double tau = t_f - t;
        s = 1 - 0.5 * _s_ddot_c * pow(tau, 2);
        s_dot = _s_ddot_c * tau;
        s_ddot = -_s_ddot_c;
    }
}

void KDLPlanner::cubic_polynomial(double t, double t_f, double &s, double &s_dot, double &s_ddot)
{
    double s_0 = 0;
    double s_f = 1;
    double s_dot_0 = 0;
    double s_dot_f = 0; //assigned values
    double a0 = s_0;
    double a1 = s_dot_0;
    double a2 = (3 * (s_f - s_0) / (t_f * t_f)) - (2 * s_dot_0 + s_dot_f) / t_f;
    double a3 = (-2 * (s_f - s_0) / (t_f * t_f * t_f)) + (s_dot_0 + s_dot_f) / (t_f * t_f);

    s = a3 * pow(t, 3) + a2 * pow(t, 2) + a1 * t + a0;
    s_dot = 3 * a3 * pow(t, 2) + 2 * a2 * t + a1;
    s_ddot = 6 * a3 * t + 2 * a2;
}

void KDLPlanner::computeLinearPoint(const double& s, const double& s_dot, const double& s_ddot,
                                   trajectory_point& point)
{
    point.pos = trajInit_ + s * (trajEnd_ - trajInit_);
    point.vel = s_dot * (trajEnd_ - trajInit_);
    point.acc = s_ddot * (trajEnd_ - trajInit_);
}

void KDLPlanner::computeCircularPoint(const double& s, const double& s_dot, const double& s_ddot,
                                     trajectory_point& point)
{
    double angle = 2.0 * M_PI * s;
    double omega = 2.0 * M_PI * s_dot;
    double alpha = 2.0 * M_PI * s_ddot;
    
    point.pos[0] = trajInit_[0];  // x remains constant
    point.pos[1] = trajInit_[1] - trajRadius_ * cos(angle);
    point.pos[2] = trajInit_[2] - trajRadius_ * sin(angle);
    
    point.vel[0] = 0.0;
    point.vel[1] = trajRadius_ * omega * sin(angle);
    point.vel[2] = -trajRadius_ * omega * cos(angle);
    
    point.acc[0] = 0.0;
    point.acc[1] = trajRadius_ * (alpha * sin(angle) + omega * omega * cos(angle));
    point.acc[2] = trajRadius_ * (-alpha * cos(angle) + omega * omega * sin(angle));
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
    trajectory_point traj;
    double s = 0.0, s_dot = 0.0, s_ddot = 0.0;

    // Compute s, s_dot, s_ddot based on velocity profile type
    if (velProfileType_ == VelProfileType::TRAPEZOIDAL)
    {
        trapezoidal_vel(time, accDuration_, s, s_dot, s_ddot);
    }
    else // CUBIC
    {
        cubic_polynomial(time, trajDuration_, s, s_dot, s_ddot);
    }

    // Compute actual trajectory point based on path type
    if (pathType_ == PathType::LINEAR)
    {
        computeLinearPoint(s, s_dot, s_ddot, traj);
    }
    else // CIRCULAR
    {
        computeCircularPoint(s, s_dot, s_ddot, traj);
    }

    return traj;
}


// }
