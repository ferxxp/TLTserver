#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
ros::init(argc, argv, "myRobot_move_joint");
ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
tf::TransformBroadcaster broadcaster;
ros::Rate loop_rate(30);

const double degree = M_PI/180;
double rot4 = 90;

geometry_msgs::TransformStamped odom_trans[2];
sensor_msgs::JointState joint_state;
odom_trans[0].header.frame_id = "bot";
odom_trans[0].child_frame_id = "mid";
odom_trans[1].header.frame_id = "mid";
odom_trans[1].child_frame_id = "top";

joint_state.name.resize(2);
joint_state.position.resize(2);
joint_state.name[0] ="base_to_mid";
joint_state.name[1] ="mid_to_top";


while (ros::ok()) {
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;


    // update transform
    // (moving in a circle with radius=2)
    odom_trans[0].header.stamp = ros::Time::now();
    odom_trans[0].transform.translation.x = 0;
    odom_trans[0].transform.translation.y = 0;
    odom_trans[0].transform.translation.z=0;
    odom_trans[0].transform.rotation = tf::createQuaternionMsgFromYaw(0);
    odom_trans[1].header.stamp = ros::Time::now();
    odom_trans[1].transform.translation.x = 0;
    odom_trans[1].transform.translation.y = 0;
    odom_trans[1].transform.translation.z=0;
    odom_trans[1].transform.rotation = tf::createQuaternionMsgFromYaw(0);

    //send the joint state and transform
  //  joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans[0]);
    broadcaster.sendTransform(odom_trans[1]);

    rot4 += 1;
    if (rot4 > 90) rot4 = 0;

    loop_rate.sleep();
}
return 0;
}
