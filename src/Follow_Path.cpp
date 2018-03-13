/***********************************************************************/
/*                             Target.cpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/

# include "Follow_Path.hpp"
# define DIST_MIN 0.10
# define COEF_ANGULAR 3
# define RATIO_VELOCITY 5
# define SPEED_LINEAR 0.3


//Constructor
Follow_Path::Follow_Path(ros::NodeHandle nh)
{
	nh_=nh;
	// Publisher
	pub_Velocity_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
	// Subscriber
	sub_Odom_ = nh_.subscribe("odom",1000,&Follow_Path::CallBack_Odom,this);
	Robot_pose_.resize(3);			//(x,y,z) in m in world
	Robot_orientation_.resize(4);	//(x,y,z,w) in rad in world
	Robot_pose_[0]= 0.0f;
	Robot_pose_[1]= 0.0f;
	Robot_pose_[2]= 0.0f;
	Robot_orientation_[0]= 0.0f;
	Robot_orientation_[1]= 0.0f;
	Robot_orientation_[2]= 0.0f;
	Robot_orientation_[3]= 0.0f;
}


void Follow_Path::CallBack_Odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	Robot_pose_[0]= msg->pose.pose.position.x;
	Robot_pose_[1]= msg->pose.pose.position.y;
	Robot_pose_[2]= msg->pose.pose.position.z;
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);
	Robot_orientation_[0]= roll;
	Robot_orientation_[1]= pitch;
	Robot_orientation_[2]= yaw;

}

void Follow_Path::send_MSG_Velocity (float linear, float angular)
{
	geometry_msgs::Twist msg;
	msg.linear.x = linear;
	msg.angular.z = angular;
	pub_Velocity_.publish(msg);
}


vector<float> Follow_Path::vector_AB (vector<float> a, vector<float> b)
{
	vector<float> vec;
	vec.push_back(b[0]-a[0]);
	vec.push_back(b[1]-a[1]);
	vec.push_back(b[2]-a[2]);
	return vec;
}

vector<float> Follow_Path::vector_vision()
{
	//vector Xrobot/world = Rz * (1,0,0)t 
	vector<float> vision;
	float alpha = Robot_orientation_[2];

	float x = cos(alpha);
	float y = sin(alpha);

	vision.push_back(x);
	vision.push_back(y);
	vision.push_back(0.0f);
	printf("%f , %f\n", x,y);
	return vision;
}

float Follow_Path::distance (vector<float> p1, vector<float> p2)
{
	float diff_x=(p1[0]-p2[0]);
	float diff_y=(p1[1]-p2[1]);
	float diff_z=(p1[2]-p2[2]);
	float dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
	return dist;
}

vector<float> Follow_Path::normalized_2D (vector<float> vec)
{
	float xx=(vec[0]*vec[0]);
	float yy=(vec[1]*vec[1]);
	float norm = sqrt(xx + yy);

	vector<float> res;
	res.push_back(vec[0]/norm);
	res.push_back(vec[1]/norm);

	return res;
}

float Follow_Path::dot_2D (vector<float> vec1, vector<float> vec2)
{
	vector<float> normV1 = normalized_2D(vec1);
	vector<float> normV2 = normalized_2D(vec2);
	float norm = normV1[0]*normV2[0] + normV1[1]*normV2[1];
	return norm;
}

float Follow_Path::determinant_2D (vector<float> vec1, vector<float> vec2)
{
	vector<float> normV1 = normalized_2D(vec1);
	vector<float> normV2 = normalized_2D(vec2);
	float det = normV1[0]*normV2[1] - normV1[1]*normV2[0];
	return det;
}



float Follow_Path::calculate_angle (vector<float> u, vector<float> v)
{
	float det = determinant_2D(u,v);
	float sign = det/abs(det);
	float var = (u[0]*v[0]+u[1]*v[1])/(sqrt(u[0]*u[0]+u[1]*u[1])*sqrt(v[0]*v[0]+v[1]*v[1]));
	float alpha = acos(var);
	alpha = alpha *(sign);
	return alpha;
}




void Follow_Path::goToPoint(vector<float> goal)
{
	float dist = distance(Robot_pose_,goal);
	float angle;
	float linear;
	vector<float> vecGoal ;
	vector<float> vecRobot;
	ros::Duration duration(0.05);

	printf("Distance error: %f\n",dist);
	printf("xrobot: %f   xgoal:%f\n",Robot_pose_[0],goal[0]);
	while(dist>DIST_MIN && ros::ok())
	{
		ros::spinOnce();
		vecGoal = vector_AB(Robot_pose_,goal);
		vecRobot = vector_vision();
		angle = calculate_angle(vecRobot,vecGoal);
		printf("Error angles: %f\n",angle);
		printf("Distance error: %f\n",dist);
		linear = fmax(SPEED_LINEAR - abs(angle/RATIO_VELOCITY), 0.0f);
		send_MSG_Velocity(linear,angle/COEF_ANGULAR);
		dist = distance(Robot_pose_,goal);
		duration.sleep();
	}
}

