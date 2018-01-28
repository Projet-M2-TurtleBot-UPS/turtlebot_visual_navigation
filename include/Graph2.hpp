/***********************************************************************/
/*                          Graph2.hpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

#ifndef GRAPH2_H
#define GRAPH2_H

# include "Target.hpp"
# include "Map_node.hpp"

# define DISTANCE_COST 3
# define ROTATION_COST 20
# define AR_COST -15

class Graph2
{
	private:
		Target start_point_;
		Target end_point_;
		vector<Target> list_Target_;
		Map_node node_;

	public:

		//Constructor
		Graph2(Target &start_point, Target &end_point, vector<Target> &list_Target, Map_node &node);
		Graph2(){};

		//Destructor
		~Graph2(){};

		//Operator
		vector<Target> a_Star();
		void init_Sons(vector<Target> closed_Target, Target &current);
		vector<Target> reconstruct_path(vector<Target> &came_From, Target &current);
		double heuristic(Target &current_point);
		int get_Lowest_fScore(vector<float> &fScore, vector<Target> &closed_Target);

		int get_Index_From_Target( vector<Target> list, Target &t);
		bool exist(vector<Target> &targets, Target &t);
		bool exist(vector<Target> &targets, int id_Target);
		vector<Target> get_neighbor(vector<int> vec_id);

		//Getter

		//Setter
};

#endif //GRAPH_H