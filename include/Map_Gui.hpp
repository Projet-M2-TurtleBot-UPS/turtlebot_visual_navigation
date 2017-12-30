/***********************************************************************/
/*                            Map_Gui.hpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef MAP_GUI_H
#define MAP_GUI_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

using namespace std;

class Map_Gui
{
	private:
		ros::Publisher vis_pub ;
		ros::NodeHandle nh_;

	public:

		//Constructor
		Map_Gui(ros::NodeHandle nh);
		Map_Gui(){};

		//Destructor
		~Map_Gui();

		//Operator
		int add_Object (int id,int type,vector<float> position, vector<float> orientation, vector<float> scale, vector<float> color);
		int add_Object (visualization_msgs::Marker marker);

		//Debug Mod
		int Test_add_Object (int id);
};


#endif //MAP_GUI_H