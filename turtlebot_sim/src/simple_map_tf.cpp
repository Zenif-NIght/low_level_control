#include  <ros/ros.h>
#include <ros/time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_map_tf");
  ros::NodeHandle nh;

  // Read in odom frame
  std::string odom_frame = "odom";
  nh.param("odom_frame", odom_frame, std::string("odom"));

  // Initialize ros variables
  tf::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped map_odom_tf;

  // Initialize transform
  map_odom_tf.transform.translation.x = 0.0;
  map_odom_tf.transform.translation.y = 0.0;
  map_odom_tf.transform.translation.z = 0.0;
  map_odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

  // Initialize header information
  map_odom_tf.header.frame_id = "map";
  map_odom_tf.header.stamp = ros::Time::now();
  map_odom_tf.child_frame_id = odom_frame;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    // Update header
    map_odom_tf.header.stamp = ros::Time::now();

    // Broadcast the tf
    tf_broadcaster.sendTransform(map_odom_tf);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
