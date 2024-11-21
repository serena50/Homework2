#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

class KDLPlanner
{

public:

    enum class PathType {
        LINEAR,
        CIRCULAR
    };

    enum class VelProfileType {
        TRAPEZOIDAL,
        CUBIC
    };

    KDLPlanner();

     // Constructor for both linear and circular trajectories
    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd,
               double _trajRadius,
               PathType _pathType = PathType::LINEAR,
               VelProfileType _velProfileType = VelProfileType::TRAPEZOIDAL);

   trajectory_point compute_trajectory(double time);

   KDL::Trajectory* getTrajectory();

    // Velocity profile functions
    void trapezoidal_vel(double t, double t_c, double &s, double &s_dot, double &s_ddot);
    void cubic_polynomial(double t, double t_f, double &s, double &s_dot, double &s_ddot);


private:

    KDL::Path_RoundedComposite* path_;
    KDL::Trajectory* traject_;
    
    double trajDuration_, accDuration_;
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;
    PathType pathType_;
    VelProfileType velProfileType_;
    double trajRadius_;

    // Helper function to compute trajectory point based on path type
    void computeLinearPoint(const double& s, const double& s_dot, const double& s_ddot,
                           trajectory_point& point);
    void computeCircularPoint(const double& s, const double& s_dot, const double& s_ddot,
                            trajectory_point& point);

};

#endif
