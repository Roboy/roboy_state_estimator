#include "roboy_state_estimator/roboyStateEstimator.hpp"

RoboyStateEstimator::RoboyStateEstimator() {
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy", ros::init_options::NoRosout);
    }
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    robot_state_pub = nh->advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);

    joint_angle_estimator_thread.reset(new boost::thread(&RoboyStateEstimator::estimateJointAngles, this));
    joint_angle_estimator_thread->detach();
}

RoboyStateEstimator::~RoboyStateEstimator() {
    if(joint_angle_estimator_thread->joinable())
        joint_angle_estimator_thread->join();
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

    while(ros::ok()){
        Matrix4d trans;
        if(!getTransform( "tracker_1", "tracker_2", trans ))
            continue;
        Matrix3d rot = trans.block(0,0,3,3);
        ShoulderAngles ea(rot);
        ROS_INFO_STREAM_THROTTLE(1, endl <<  trans << endl << ea.angles().transpose());

        msg.state.joint_state.header.seq = id++;
        msg.state.joint_state.header.stamp = ros::Time::now();

        msg.state.joint_state.velocity[0] = (msg.state.joint_state.position[0]-ea.angles()[1])/0.01;
        msg.state.joint_state.velocity[1] = (msg.state.joint_state.position[1]+ea.angles()[0])/0.01;
        msg.state.joint_state.velocity[2] = (msg.state.joint_state.position[2]-ea.angles()[2])/0.01;

//        msg.state.joint_state.velocity[0] = 0;
//        msg.state.joint_state.velocity[1] = 0;
//        msg.state.joint_state.velocity[2] = 0;

        msg.state.joint_state.position[0] = ea.angles()[1];
        msg.state.joint_state.position[1] = -ea.angles()[0];
        msg.state.joint_state.position[2] = ea.angles()[2];


        robot_state_pub.publish(msg);

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

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roboyStateEstimator", ros::init_options::NoRosout);

    RoboyStateEstimator stateEstimator;

    while(ros::ok()){
        ROS_INFO_THROTTLE(10, "roboy state estimator active");
    }

    return 0;
}
