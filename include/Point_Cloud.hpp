/***********************************************************************/
/*                             Point_Cloud.hpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "Target.hpp"
#include "Map_node.hpp"
#include <fstream>
#include <cstdlib>
#include <ctime>

using namespace std;

class Point_Cloud
{
	private:
		vector<int> map_treatment_;
		vector<Target> targets_;
		vector<Target> markers_;
		vector<float> start_point_;
		Map_node map_real_;
		float dist_vis_;
		int max_err_cons_;
		int width_map_;
		int height_map_;
		float resolution_;

	public:

		//Constructor
		Point_Cloud(Map_node &map, vector<Target> &markers, vector<float> &start_point);
		Point_Cloud(){};

		//Destructor
		~Point_Cloud(){};

		//Operator
		vector<Target> create_Point_Cloud();
		void fill_Circle(int x, int y);
		bool is_In_Circle(int x_Center, int y_Center, int x_Point, int y_Point);
		bool is_Target_Redundant(int x_Target, int y_Target);
		bool is_Target_Connected_Pix(int x_Target, int y_Target);
		bool is_Target_Connected_Pos(float x_Target, float y_Target);
		


		//Getter

		//Setter

		//Debug
		void write_BMP (vector<int> &map_data_);

};


#endif //POINT_CLOUD_H