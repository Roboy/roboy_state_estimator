#pragma once

#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/rviz_visualization.hpp>

// ros
#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Float32.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include "Eigen/unsupported/EulerAngles.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <thread>

static const std::string ZED_LEFT = "zed left", ZED_RIGHT = "zed right";
using namespace cv;
using namespace Eigen;
using namespace std;



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

    void leftCameraCB(const sensor_msgs::Image::ConstPtr &msg);

    void rightCameraCB(const sensor_msgs::Image::ConstPtr &msg);

    void detectAruco();

    void estimateJointAngles();
private:

    bool getTransform(const char *from, const char *to, Matrix4d &transform);

    void JointAngleCB(const std_msgs::Float32::ConstPtr &msg);

    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher robot_state_pub, joint_state_pub;
    ros::Subscriber joint_angle_sub, left_zed_camera_sub, right_zed_camera_sub;
    cv_bridge::CvImagePtr zed_left_ptr, zed_right_ptr;
    Mat camMatrix, distCoeffs;
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::Dictionary> dictionary;
    vector<int> arucoIDs;
    float markerLength = 0.07f;
    float K_left[9] = {700.5650024414062, 0.0, 639.5999755859375, 0.0, 700.5650024414062, 391.23699951171875, 0.0, 0.0, 1.0},
            D[5] = {-0.17171600461006165, 0.024814900010824203, 0, 0, 0};
    boost::shared_ptr<boost::thread> joint_angle_estimator_thread;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    float elbow_left =0;
};

