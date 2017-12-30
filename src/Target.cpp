/***********************************************************************/
/*                             Target.cpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
# include "Target.hpp"
# include "Color.hpp"

# define SCALE_X 0.2
# define SCALE_Y 0.2
# define SCALE_Z 0.02


//Constructor
	Target::Target(int id,int type, vector<float> pos, vector<float> orien)
	{
		id_ =id;
		type_=type;
		position_=pos;
		orientation_ = orien;
		switch (type) {
			case 0: color_=color_blue(); break;
			case 1: color_=color_magenta(); break;
			case 2: color_=color_red(); break;
			default : color_=color_black(); break;
		}
	}

//Destructor

	Target::~Target(){}

//Create_MSG_Marker
//this fonction creates a visualization_msg  of the target
	visualization_msgs::Marker Target::create_MSG_Marker ()
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "Target";
		marker.id = id_;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = position_[0];
		marker.pose.position.y = position_[1];
		marker.pose.position.z = position_[2];
		marker.pose.orientation.x = orientation_[0];
		marker.pose.orientation.y = orientation_[1];
		marker.pose.orientation.z = orientation_[2];
		marker.pose.orientation.w = orientation_[3];
		marker.scale.x = SCALE_X;
		marker.scale.y = SCALE_Y;
		marker.scale.z = SCALE_Z;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = color_[0];
		marker.color.g = color_[1];
		marker.color.b = color_[2];
		marker.lifetime = ros::Duration();

		return marker;
	}

//Getter

	vector<float> Target::get_Position (){return position_;}
	vector<float> Target::get_Orientation (){return orientation_;}
	vector<float> Target::get_Color (){return color_;}
	int Target::get_Id (){return id_;}

//Setter
	void Target::set_Position (vector<float> pos){position_=pos;}
	void Target::set_Orientation (vector<float> orien){orientation_=orien;}
	void Target::set_Color (vector<float> color){color_=color;}

