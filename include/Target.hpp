/***********************************************************************/
/*                             Target.cpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef MAP_TARGET_H
#define MAP_TARGET_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <fstream>

using namespace std;



class Target
{
	private:
		int id_;					// Object's id
		int type_;					// Target's type (0:Random 1:Start 2:ArCode)
		vector<float> position_;	// Target's position in x, y and z (in m)
		vector<float> orientation_;	// Target's orientation in x, y, z and w (in rad)
		vector<float> color_;		// Target's color in RGB ([0.0 .. 1.0])

	public:

		//Constructor
		Target(int id,int type, vector<float> pos, vector<float> orien);
		Target(){};

		//Destructor
		~Target();

		//Operator
		visualization_msgs::Marker create_MSG_Marker ();

		//Getter
		vector<float> get_Position ();
		vector<float> get_Orientation ();
		vector<float> get_Color ();
		int get_Id ();

		//Setter
		void set_Position (vector<float> pos);
		void set_Orientation (vector<float> orien);
		void set_Color (vector<float> color);

};


#endif //MAP_TARGET_H