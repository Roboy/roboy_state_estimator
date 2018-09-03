#pragma once

#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/rviz_visualization.hpp>

// ros
#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include "Eigen/unsupported/EulerAngles.hpp"

#include <thread>
#include <std_msgs/Float32.h>

using namespace Eigen;
using namespace std;

typedef EulerSystem<EULER_Z, EULER_Y, EULER_X> ShoulderSystem;
typedef EulerAngles<double, ShoulderSystem> ShoulderAngles;

class RoboyStateEstimator:public rviz_visualization{
public:
    /**
     * Constructor
     */
    RoboyStateEstimator();

    /**
     * Destructor
     */
    ~RoboyStateEstimator();

    void estimateJointAngles();
private:

    bool getTransform(const char *from, const char *to, Matrix4d &transform);

    void JointAngleCB(const std_msgs::Float32::ConstPtr &msg);
//    /**
//     * Subscriber callback for motor status
//     * @param msg
//     */
//    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
//    /*
//     * This function loads the controllers registered to the individual joint interfaces
//     * @param controllers names of controllers
//     * @return success
//     */
//    bool loadControllers(vector<string> controllers);
//
//    /*
//     * This function unloads the controllers registered to the individual joint interfaces
//     * @param controllers names of controllers
//     * @return success
//     */
//    bool unloadControllers(vector<string> controllers);
//
//    /*
//	 * This function starts the controllers registered to the individual joint interfaces
//	 * @param controllers names of controllers
//	 * @return success
//	 */
//    bool startControllers(vector<string> controllers);
//
//    /*
//	 * This function stops the controllers registered to the individual joint interfaces
//	 * @param controllers names of controllers
//	 * @return success
//	 */
//    bool stopControllers(vector<string> controllers);
//
//    /**
//     * This function initialises the requested motors
//     */
//    bool initializeControllers(roboy_communication_middleware::Initialize::Request &req,
//                               roboy_communication_middleware::Initialize::Response &res);
//
//    bool updateTarget(roboy_communication_middleware::Initialize::Request &req,
//                      roboy_communication_middleware::Initialize::Response &res);

    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher robot_state_pub, joint_state_pub;
    ros::Subscriber joint_angle_sub;
    boost::shared_ptr<boost::thread> joint_angle_estimator_thread;
    tf::TransformListener tf_listener;
    float elbow_left =0;
};

