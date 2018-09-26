#include "roboy_state_estimator/roboyStateEstimator.hpp"

RoboyStateEstimator::RoboyStateEstimator() {
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy", ros::init_options::NoRosout);
    }
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    robot_state_pub = nh->advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);
    joint_angle_sub = nh->subscribe("/roboy/middleware/joint_angle/elbow_left", 1, &RoboyStateEstimator::JointAngleCB, this);
//    camera_info_sub = nh->subscribe("/roboy/middleware/joint_angle/elbow_left", 1, &RoboyStateEstimator::JointAngleCB, this);

    left_zed_camera_sub = nh->subscribe("/zed/left/image_raw_color", 1, &RoboyStateEstimator::leftCameraCB, this);
    right_zed_camera_sub = nh->subscribe("/zed/right/image_raw_color", 1, &RoboyStateEstimator::rightCameraCB, this);

    camMatrix = Mat(3, 3, CV_32FC1, K_left);
    distCoeffs = Mat(1, 5, CV_32FC1, D);

    detectorParams = aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    detectorParams->cornerRefinementMaxIterations = 100;
    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_ARUCO_ORIGINAL));

    cv::namedWindow(ZED_LEFT);
    cv::namedWindow(ZED_RIGHT);
    moveWindow(ZED_LEFT, 0, 0);
    moveWindow(ZED_RIGHT, 700, 0);

//    joint_angle_estimator_thread.reset(new boost::thread(&RoboyStateEstimator::estimateJointAngles, this));
//    joint_angle_estimator_thread->detach();
}

RoboyStateEstimator::~RoboyStateEstimator() {
    if(joint_angle_estimator_thread->joinable())
        joint_angle_estimator_thread->join();
}

void RoboyStateEstimator::leftCameraCB(const sensor_msgs::Image::ConstPtr &msg) {
    try {
        zed_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void RoboyStateEstimator::rightCameraCB(const sensor_msgs::Image::ConstPtr &msg) {
    try {
        zed_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void RoboyStateEstimator::detectAruco() {
    {
        vector<int> ids;
        vector<vector<Point2f> > corners, rejected;
        vector<Vec3d> rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(zed_left_ptr->image, dictionary, corners, ids, detectorParams, rejected);
        aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
        // draw results
        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(zed_left_ptr->image, corners, ids);
            for (unsigned int i = 0; i < ids.size(); i++) {
                if (!arucoIDs.empty()) {
                    if (std::find(arucoIDs.begin(), arucoIDs.end(), ids[i]) == arucoIDs.end())
                        continue;
                }
                aruco::drawAxis(zed_left_ptr->image, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                markerLength * 0.5f);
//                    double theta = sqrt(pow(rvecs[i][0], 2.0) + pow(rvecs[i][1], 2.0) + pow(rvecs[i][2], 2.0));
//                    Quaterniond q(rvecs[i][0] / theta, rvecs[i][1] / theta, rvecs[i][2] / theta, theta);
//                    q.normalize();
                tf::Transform trans;
                trans.setRotation(tf::Quaternion(0,0,0,1));
                Quaterniond q_cv_coordinates_to_gazebo(0, 0, 0.7071068, 0.7071068);
                Vector3d pos(tvecs[i][0], tvecs[i][1],tvecs[i][2]);
//                pos = q_cv_coordinates_to_gazebo.matrix()*pos;
                trans.setOrigin(tf::Vector3(pos[0],pos[1],pos[2]));
                char str[100];
                sprintf(str, "zed_left_aruco_%d", ids[i]);
                tf_broadcaster.sendTransform(
                        tf::StampedTransform(trans, ros::Time::now(), "zed_left_camera_optical_frame", str));
            }
        }
        cv::imshow(ZED_LEFT, zed_left_ptr->image);
    }
    {
        vector<int> ids;
        vector<vector<Point2f> > corners, rejected;
        vector<Vec3d> rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(zed_right_ptr->image, dictionary, corners, ids, detectorParams, rejected);
        aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
        // draw results
        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(zed_right_ptr->image, corners, ids);
            for (unsigned int i = 0; i < ids.size(); i++) {
                if (!arucoIDs.empty()) {
                    if (std::find(arucoIDs.begin(), arucoIDs.end(), ids[i]) == arucoIDs.end())
                        continue;
                }
                aruco::drawAxis(zed_right_ptr->image, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                markerLength * 0.5f);
//                    double theta = sqrt(pow(rvecs[i][0], 2.0) + pow(rvecs[i][1], 2.0) + pow(rvecs[i][2], 2.0));
//                    Quaterniond q(rvecs[i][0] / theta, rvecs[i][1] / theta, rvecs[i][2] / theta, theta);
//                    q.normalize();
                tf::Transform trans;
                trans.setRotation(tf::Quaternion(0,0,0,1));
                Quaterniond q_cv_coordinates_to_gazebo(0, 0, 0.7071068, 0.7071068);
                Vector3d pos(tvecs[i][0], tvecs[i][1],tvecs[i][2]);
//                pos = q_cv_coordinates_to_gazebo.matrix()*pos;
                trans.setOrigin(tf::Vector3(pos[0],pos[1],pos[2]));
                char str[100];
                sprintf(str, "zed_right_aruco_%d", ids[i]);
                tf_broadcaster.sendTransform(
                        tf::StampedTransform(trans, ros::Time::now(), "zed_right_camera_optical_frame", str));
            }
        }
        cv::imshow(ZED_RIGHT, zed_right_ptr->image);
    }
}

void RoboyStateEstimator::estimateJointAngles(){
    ros::Rate rate(100);
    moveit_msgs::DisplayRobotState msg;
    static int id = 0;
    msg.state.joint_state.header.frame_id = "world";
    msg.state.joint_state.name.push_back("sphere_axis0");
    msg.state.joint_state.name.push_back("sphere_axis1");
    msg.state.joint_state.name.push_back("sphere_axis2");
    msg.state.joint_state.name.push_back("elbow_left");
    msg.state.joint_state.position.push_back(0);
    msg.state.joint_state.position.push_back(0);
    msg.state.joint_state.position.push_back(0);
    msg.state.joint_state.position.push_back(0);
    msg.state.joint_state.velocity.push_back(0);
    msg.state.joint_state.velocity.push_back(0);
    msg.state.joint_state.velocity.push_back(0);
    msg.state.joint_state.velocity.push_back(0);
    msg.state.joint_state.effort.push_back(0);
    msg.state.joint_state.effort.push_back(0);
    msg.state.joint_state.effort.push_back(0);
    msg.state.joint_state.effort.push_back(0);
    ros::Duration timeout(0.001);
    while(ros::ok()){
        Matrix4d trans;
        if(tf_listener.waitForTransform("tracker_1","tracker_2",ros::Time::now(),timeout)) {
            getTransform("tracker_1", "tracker_2", trans);
            Matrix3d rot = trans.block(0, 0, 3, 3);
            ShoulderAngles ea(rot);
            ROS_INFO_STREAM_THROTTLE(1, endl << trans << endl << ea.angles().transpose());

            msg.state.joint_state.header.seq = id++;
            msg.state.joint_state.header.stamp = ros::Time::now();

            msg.state.joint_state.velocity[0] = (msg.state.joint_state.position[0] - ea.angles()[1]) / 0.01;
            msg.state.joint_state.velocity[1] = (msg.state.joint_state.position[1] + ea.angles()[0]) / 0.01;
            msg.state.joint_state.velocity[2] = (msg.state.joint_state.position[2] - ea.angles()[2]) / 0.01;
            msg.state.joint_state.velocity[3] = (msg.state.joint_state.position[3] - elbow_left) / 0.01;

//        msg.state.joint_state.velocity[0] = 0;
//        msg.state.joint_state.velocity[1] = 0;
//        msg.state.joint_state.velocity[2] = 0;

            msg.state.joint_state.position[0] = ea.angles()[1];
            msg.state.joint_state.position[1] = -ea.angles()[0];
            msg.state.joint_state.position[2] = ea.angles()[2];
            msg.state.joint_state.position[3] = elbow_left;


            robot_state_pub.publish(msg);
        }
        if (zed_right_ptr != nullptr && zed_left_ptr != nullptr) {
            detectAruco();
            cv::waitKey(1);
        }
        rate.sleep();
    }
}

bool RoboyStateEstimator::getTransform(const char *from, const char *to, Matrix4d &transform){
    tf::StampedTransform trans;
    try {
        tf_listener.lookupTransform(to, from, ros::Time(0), trans);
    }
    catch (tf::TransformException ex) {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        return false;
    }

    Eigen::Affine3d trans_;
    tf::transformTFToEigen(trans, trans_);
    transform = trans_.matrix();
    return true;
}

void RoboyStateEstimator::JointAngleCB(const std_msgs::Float32::ConstPtr &msg){
    elbow_left = msg->data;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roboyStateEstimator", ros::init_options::NoRosout);

    RoboyStateEstimator stateEstimator;

    stateEstimator.estimateJointAngles();

    return 0;
}
