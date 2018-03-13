/***********************************************************************/
/*                          Follow_Path.hpp                            */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef FOLLOW_PATH_H
#define FOLLOW_PATH_H

# include "ros/ros.h"
# include <tf/tf.h>
# include <fstream>
# include <math.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/Twist.h>

using namespace std;

class Follow_Path
{
	private:
		ros::Subscriber sub_Odom_;				//subscriber  /Odom
		//ros::Subscriber sub_Stop_;			//subscriber  /Stopping
		ros::Publisher pub_Velocity_;			//publisher   /mobile_base/command/velocity
		ros::NodeHandle nh_;				
		std::vector<float> Robot_pose_;			//(x,y,z) in m in world
		std::vector<float> Robot_orientation_;	//(x,y,z,w) in rad in world
		//std::vector<int> map_data_;			//(pixel)


	public:

		// Constructor
		Follow_Path(ros::NodeHandle nh);
		Follow_Path(){};

		//Destructor
		~Follow_Path(){};

		// CallBACK
		void CallBack_Odom(const nav_msgs::Odometry::ConstPtr& msg);
		//void CallBack_Stop(const geometry_msgs::Twist::ConstPtr& msg);

		//Publisher Operator
		void send_MSG_Velocity (float linear, float angular);

		// Tool operator
		vector<float> vector_AB (vector<float> a, vector<float> b);
		vector<float> vector_vision();
		float distance (vector<float> p1, vector<float> p2);
		vector<float> normalized_2D (vector<float> vec);
		float dot_2D (vector<float> p1, vector<float> p2);
		float determinant_2D (vector<float> vec1, vector<float> vec2);
		float calculate_angle (vector<float> u, vector<float> v);
		


		// Operator
		void goToPoint(vector<float> goal);

		
		// Getter

		// Setter

		// DEBUG MOD
		
};

#endif //FOLLOW_PATH_H