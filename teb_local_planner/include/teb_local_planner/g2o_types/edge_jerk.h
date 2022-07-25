#ifndef EDGE_JERK_H_
#define EDGE_JERK_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include <geometry_msgs/Twist.h>

namespace teb_local_planner
{
class EdgeJerk : public BaseTebMultiEdge<2, double>
{
public:
  EdgeJerk()
  {
    this->resize(7);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeJerk()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[5]);
    const VertexTimeDiff* dt3 = static_cast<const VertexTimeDiff*>(_vertices[6]);

    // VELOCITY & Jerk
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    const Eigen::Vector2d diff3 = pose4->position() - pose3->position();

    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    double dist3 = diff3.norm();

    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
    const double angle_diff3 = g2o::normalize_theta(pose4->theta() - pose3->theta());


    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius =  dist1/(2*sin(angle_diff1/2));
            dist1 = fabs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius =  dist2/(2*sin(angle_diff2/2));
            dist2 = fabs( angle_diff2 * radius ); // actual arg length!
        }
        if (angle_diff3 != 0)
        {
            const double radius =  dist3/(2*sin(angle_diff3/2));
            dist3 = fabs( angle_diff3 * radius ); // actual arg length!
        }
    }

    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();
    double vel3 = dist3 / dt3->dt();


    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) );
    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) );
    vel3 *= fast_sigmoid( 100*(diff3.x()*cos(pose3->theta()) + diff3.y()*sin(pose3->theta())) );

    const double acc_lin0  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );
    const double acc_lin1  = (vel3 - vel2)*2 / ( dt2->dt() + dt3->dt() );

    const double jerk_lin = (acc_lin1-acc_lin0)/(0.25*dt1->dt()+0.5*dt2->dt()+0.25*dt3->dt());
    _error[0] = penaltyBoundToInterval(jerk_lin,0.5,cfg_->optim.penalty_epsilon);

    // ANGULAR Jerk
    const double omega1 = angle_diff1 / dt1->dt();
    const double omega2 = angle_diff2 / dt2->dt();
    const double omega3 = angle_diff3 / dt3->dt();

    const double acc_rot0  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );
    const double acc_rot1 =  (omega3 - omega2)*2 / ( dt2->dt() + dt3->dt() );

    const double jerk_rot  = (acc_rot1-acc_rot0)/(0.25*dt1->dt()+0.5*dt2->dt()+0.25*dt3->dt());
    _error[1] = penaltyBoundToInterval(jerk_rot,0.3,cfg_->optim.penalty_epsilon);


    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() rotational: _error[1]=%f\n",_error[1]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
class EdgeJerkStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkStart()
  {
    _measurement = NULL;
    this->resize(5);
  }


  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);

    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & Jerk
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

    double dist1 = diff1.norm();
    double dist2 = diff2.norm();

    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius =  dist1/(2*sin(angle_diff1/2));
            dist1 = fabs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius =  dist2/(2*sin(angle_diff2/2));
            dist2 = fabs( angle_diff2 * radius ); // actual arg length!
        }

    }



    const double vel0 = _measurement->linear.x;
    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();

    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) );
    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) );
    //vel3 *= fast_sigmoid( 100*(diff3.x()*cos(pose3->theta()) + diff3.y()*sin(pose3->theta())) );

    //vel2 *= fast_sigmoid( 100*(diff.x()*cos(pose1->theta()) + diff.y()*sin(pose1->theta())) );

    const double acc_lin0  = (vel1 - vel0)/ dt1->dt() ;
    const double acc_lin1  = (vel2 - vel1) / dt2->dt() ;

    const double jerk_lin = (acc_lin1-acc_lin0)/(0.5*dt1->dt()+0.5*dt2->dt());
    _error[0] = penaltyBoundToInterval(jerk_lin,0.5,cfg_->optim.penalty_epsilon);


    // ANGULAR Jerk
    const double omega0 = _measurement->angular.z;
    const double omega1 = angle_diff1 / dt1->dt();
    const double omega2 = angle_diff2 / dt2->dt();

    const double acc_rot0  = (omega1 - omega0) / dt1->dt();
    const double acc_rot1  = (omega2 - omega1) / dt2->dt();

    const double jerk_rot  = (acc_rot1-acc_rot0)/(0.5*dt1->dt()+0.5*dt2->dt());
    _error[1] = penaltyBoundToInterval(jerk_rot,0.3,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerkStart::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerkStart::computeError() rotational: _error[1]=%f\n",_error[1]);
  }

  /**
   * @brief Set the initial velocity that is taken into account for calculating the Jerk
   * @param vel_start twist message containing the translational and rotational velocity
   */
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeJerkGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkGoal()
  {
    _measurement = NULL;
    this->resize(5);
  }


  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeJerkGoal()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);

    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & Jerk
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

    double dist1 = diff1.norm();
    double dist2 = diff2.norm();

    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius =  dist1/(2*sin(angle_diff1/2));
            dist1 = fabs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius =  dist2/(2*sin(angle_diff2/2));
            dist2 = fabs( angle_diff2 * radius ); // actual arg length!
        }

    }

    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();
    double vel3 = _measurement->linear.x;

    // consider directions
    //vel2 *= g2o::sign(diff[0]*cos(pose1->theta()) + diff[1]*sin(pose1->theta()));

    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) );
    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) );
    //vel3 *= fast_sigmoid( 100*(diff3.x()*cos(pose3->theta()) + diff3.y()*sin(pose3->theta())) );

    //vel2 *= fast_sigmoid( 100*(diff.x()*cos(pose1->theta()) + diff.y()*sin(pose1->theta())) );

    const double acc_lin0  = (vel2 - vel1)/ dt1->dt() ;
    const double acc_lin1  = (vel3 - vel2) / dt2->dt() ;

    const double jerk_lin = (acc_lin1-acc_lin0)/(0.5*dt1->dt()+0.5*dt2->dt());
    _error[0] = penaltyBoundToInterval(jerk_lin,0.5,cfg_->optim.penalty_epsilon);


    // ANGULAR Jerk

    const double omega1 = angle_diff1 / dt1->dt();
    const double omega2 = angle_diff2 / dt2->dt();
    const double omega3 = _measurement->angular.z;

    const double acc_rot0  = (omega2 - omega1) / dt1->dt();
    const double acc_rot1  = (omega3 - omega2) / dt2->dt();

    const double jerk_rot  = (acc_rot1-acc_rot0)/(0.5*dt1->dt()+0.5*dt2->dt());
    _error[1] = penaltyBoundToInterval(jerk_rot,0.3,cfg_->optim.penalty_epsilon);

  }

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the Jerk
   * @param vel_goal twist message containing the translational and rotational velocity
   */
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};
#endif /* EDGE_JERK_H_ */
