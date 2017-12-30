/***********************************************************************/
/*                             Map_Gui.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
# include "Map_Gui.hpp"

using namespace std;




//Constructor
Map_Gui::Map_Gui(ros::NodeHandle nh)
{
	nh_=nh;
	//Publisher
	vis_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}




//Destructor
Map_Gui::~Map_Gui(){}




//Add_Object
//add a object of type Marker on map.
//id: 			object's id
//type:			object's type
//position:		object's position in x, y and z (in m)
//orientation:	object's orientation in x, y, z and w (in rad)
//scale:		object's scale in x, y and z (in m)
//color:		object's color (RGB [0.0 .. 1.0])
int Map_Gui::add_Object (int id,int type,vector<float> position, vector<float> orientation, vector<float> scale, vector<float> color)
{
	//Init variables
	int i = 0;

	//Verification
	if(position.size()!=3 )
	{
		ROS_ERROR("[ Add_object ]Error: size Position (%d to 3).",position.size());
		return -1;
	}
	if(orientation.size()!=4 )
	{
		ROS_ERROR("[ Add_object ]Error: size Orientation (%d to 4).",orientation.size());
		return -2;
	}
	if(scale.size()!=3 )
	{
		ROS_ERROR("[ Add_object ]Error: size scale (%d to 3).",scale.size());
		return -3;
	}
	if(color.size()!=3 )
	{
		ROS_ERROR("[ Add_object ]Error: size Color (%d to 3).",color.size());
		return -4;
	}

	//Init MSG
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = id;
	marker.type = type;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position[0];
	marker.pose.position.y = position[1];
	marker.pose.position.z = position[2];
	marker.pose.orientation.x = orientation[0];
	marker.pose.orientation.y = orientation[1];
	marker.pose.orientation.z = orientation[2];
	marker.pose.orientation.w = orientation[3];
	marker.scale.x = scale[0];
	marker.scale.y = scale[1];
	marker.scale.z = scale[2];
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
	marker.lifetime = ros::Duration();

	//Send msg
	while(i<5)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}

	return 0;
}




//Add_Object
//add a object of type Marker on map.
//marker: 	object's message of type visualization_msgs::Marker
int Map_Gui::add_Object (visualization_msgs::Marker marker)
{
	//Init variables
	int i =0;

	//Send msg
	while(i<5)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}

	return 0;
}




//Debug Mod
//Test_add_Object
//add a blue cube Marker at the position (0,0,0) without rotation.
//id:	object's id
int Map_Gui::Test_add_Object (int id)
{
	vector<float> position;
	position.push_back(0.0f);
	position.push_back(0.0f);
	position.push_back(0.0f);

	vector<float> orientation ;
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(1.0f);

	vector<float> scale ;
	scale.push_back(0.1);
	scale.push_back(0.1);
	scale.push_back(0.01);

	vector<float> color;
	color.push_back(0.0);
	color.push_back(0.0);
	color.push_back(1.0);

	add_Object (visualization_msgs::Marker::CUBE,id,position,orientation,scale,color);
}
