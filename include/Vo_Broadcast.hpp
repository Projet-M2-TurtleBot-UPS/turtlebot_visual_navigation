/***********************************************************************/
/*                         Vo_Broadcast.hpp                            */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#ifndef VO_H
#define VO_H

# include <ros/ros.h>
# include <angles/angles.h>
# include <math.h>
# include <ar_track_alvar_msgs/AlvarMarkers.h>
# include <nav_msgs/Odometry.h>
# include <tf/tf.h>
# include <nav_msgs/Odometry.h>
# include <Target.hpp>
# include <Set_Marker.hpp>

using namespace std;

class Vo_Broadcast
{
	private:
    ros::NodeHandle nh_;                        // The node handle for Recherche
    ros::Subscriber ar_subscriber_;             // Subscriber to /ar_pose_marker
    ros::Subscriber odom_subscriber_;           // Subscriber to /odom
    ros::Publisher vo_publisher_;               // Publisher to /mobile_base/commands/velocity
    vector<float> robot_pose_;
    vector<float> robot_orien_;
    vector<float> ar_pose_;
    vector<float> ar_orien_;
    vector<Target> ar_list;
    int ar_id;
    nav_msgs::Odometry odom_message;
    Set_Marker set_maker;


	public:

	// Constructors
    Vo_Broadcast(ros::NodeHandle nh);
    Vo_Broadcast(){};

	// Destructor
	~Vo_Broadcast(){};

    // Setters

    // Getters
    int get_Ar_id();

    //callback
    void odomCallback(const nav_msgs::Odometry &message);
    void arCallback(ar_track_alvar_msgs::AlvarMarkers req);

    // Operators
    tf::Quaternion calculateAngle(float roll, float pitch, float yaw);
    geometry_msgs::Quaternion transformAngle();

    void transform_rep();
};

#endif //VO_H

