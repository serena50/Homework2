#include "kdl_control.h"

//Joint Space Controller

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}


Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity();
}


//Operational Space Controller
Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                     KDL::Twist &_desVel,
                                     KDL::Twist &_desAcc,
                                     double _Kpp, double _Kpo,
                                     double _Kdp, double _Kdo)
{
    Eigen::Matrix<double,6,6> Kp, Kd;
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

    // Get current state
    KDL::Frame curr_pose = robot_->getEEFrame();
    Eigen::VectorXd q_dot = robot_->getJntVelocities();

    // Get inertia matrix B(q)
    Eigen::Matrix<double,7,7> B = robot_->getJsim();
    // Get geometric Jacobian
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;

    // Compute robust SVD-based damped pseudo-inverse
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd s = svd.singularValues();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd U = svd.matrixU();

    // Compute condition number
    double max_sv = s(0);
    double min_sv = s(s.size()-1);
    double condition_number = max_sv / (min_sv + 1e-10);

    // Adaptive damping 
    double lambda_max = 0.1;
    double lambda_min = 0.001;  
    double cond_threshold = 100.0;  

    double lambda;
    if (condition_number < cond_threshold) {
        lambda = lambda_min;
    } else {
        lambda = lambda_min + (lambda_max - lambda_min) * 
                std::min(1.0, (condition_number - cond_threshold) / 200.0);
    }

    // Compute damped pseudo-inverse with truncation only in EXTREME cases
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(V.cols(), U.cols());
    const double sv_threshold = max_sv * 1e-5;  // really low treshold

    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > sv_threshold) {
            S_inv(i,i) = s(i) / (s(i) * s(i) + lambda * lambda);
        }
    }

    Eigen::Matrix<double,7,6> J_pinv = V * S_inv * U.transpose();

    // Compute errors with bounds
    Vector6d x_tilde, dx_tilde;
    computeErrors(_desPos, curr_pose, _desVel, robot_->getEEVelocity(), x_tilde, dx_tilde);

    // Bounds (permissive!) 
    const double MAX_LINEAR_ERROR = 10.0;     // 10 metri - never reached, for mathematical purposes only
    const double MAX_ANGULAR_ERROR = 3.14;    // 180 gradi - same as before

    for (int i = 0; i < 3; ++i) {
        x_tilde(i) = std::clamp(x_tilde(i), -MAX_LINEAR_ERROR, MAX_LINEAR_ERROR);
        dx_tilde(i) = std::clamp(dx_tilde(i), -MAX_LINEAR_ERROR, MAX_LINEAR_ERROR);
        x_tilde(i+3) = std::clamp(x_tilde(i+3), -MAX_ANGULAR_ERROR, MAX_ANGULAR_ERROR);
        dx_tilde(i+3) = std::clamp(dx_tilde(i+3), -MAX_ANGULAR_ERROR, MAX_ANGULAR_ERROR);
    }

    // Desired acceleration con bounds larghi
    Vector6d xd_ddot;
    xd_ddot << _desAcc.vel.data[0], _desAcc.vel.data[1], _desAcc.vel.data[2],
            _desAcc.rot.data[0], _desAcc.rot.data[1], _desAcc.rot.data[2];

    const double MAX_LINEAR_ACC = 20.0;    // 
    const double MAX_ANGULAR_ACC = 10.0;   // both these bounds will never be reached in our case

    for (int i = 0; i < 3; ++i) {
        xd_ddot(i) = std::clamp(xd_ddot(i), -MAX_LINEAR_ACC, MAX_LINEAR_ACC);
        xd_ddot(i+3) = std::clamp(xd_ddot(i+3), -MAX_ANGULAR_ACC, MAX_ANGULAR_ACC);
    }

    // Control law with bounds
    Vector6d J_dot_q_dot = robot_->getEEJacDot() * q_dot;
    J_dot_q_dot = J_dot_q_dot.array().min(10.0).max(-10.0);  


    Vector6d inner_term = xd_ddot + Kd * dx_tilde + Kp * x_tilde - J_dot_q_dot;

    const double MAX_INNER_TERM = 100.0;  // Will prevent spikes
    inner_term = inner_term.array().min(MAX_INNER_TERM).max(-MAX_INNER_TERM);

    Eigen::VectorXd y = J_pinv * inner_term;
    y = y.array().min(50.0).max(-50.0);  

    // Compute final torques
    Eigen::VectorXd tau = B * y + robot_->getCoriolis() + robot_->getGravity();

    // Torques limit
    const double MAX_TORQUE = 500.0;  // Alto ma non infinito
    for (int i = 0; i < tau.size(); ++i) {
        if (!std::isfinite(tau(i))) {
            tau(i) = robot_->getGravity()(i);  // Fallback at gravity if NaN/Inf
        }
        tau(i) = std::clamp(tau(i), -MAX_TORQUE, MAX_TORQUE);
    }

    return tau;
}
