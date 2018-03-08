/***********************************************************************/
/*                          Graph.hpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

#ifndef CREATE_GRAPH_H
#define CREATE_GRAPH_H

# include "Target.hpp"
# include "Map_node.hpp"

# define DISTANCE_COST 3
# define ROTATION_COST 20
# define AR_COST -15

class Graph
{
	private:
		Target start_point_;	// The position of the robot
		Target end_point_;	// The position where we want the robot to go
		vector<Target> list_Target_;	// The list of all the point of the point cloud

	public:

		//Constructor
		Graph(Target &start_point, Target &end_point, vector<Target> &list_Target);
		Graph(){};

		//Destructor
		~Graph(){};

		//Operator
		vector<Target> a_Star();
		void init_Sons(vector<Target> closed_Target, Target &current);
		vector<Target> reconstruct_path(vector<Target> &came_From, Target &current);
		double heuristic(Target &current_point);
		int get_Lowest_FScore(vector<float> &fScore, vector<Target> &closed_Target);

		int get_Index_From_Target( vector<Target> list, Target &t);
		bool exist(vector<Target> &targets, Target &t);
		bool exist(vector<Target> &targets, int id_Target);

		//Getter

		//Setter
};

#endif //GRAPH_H
