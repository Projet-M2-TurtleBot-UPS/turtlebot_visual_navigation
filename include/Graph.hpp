/***********************************************************************/
/*                          Graph.hpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

#ifndef GRAPH_H
#define GRAPH_H

# include "Node.hpp"
# include "Target.hpp"
# include "Map_node.hpp"

/*# define DISTANCE_COST 10
# define ROTATION_COST 20
# define AR_COST -1*/

class Graph
{
	private:
		Target start_point_;
		Target end_point_;
		vector<Node> list_nodes_;
		vector<Target> list_Target_;
		Map_node node_;
		float current_Cost_;


	public:

		//Constructor
		Graph(Target &start_point, Target &end_point, vector<Target> &list_Target, Map_node &node);
		Graph(){};

		//Destructor
		~Graph(){};

		//Operator
		vector<Target> a_Star();
		void init_Sons(Target &current);
		float cost_calculation(Target &mother, Target &son);
		Target get_Optimum(vector<Target> &tabou);
		void add_Node(Target &mother, Target &son, float cost);
		vector<Target> find_Path();

		//Getter

		//Setter
		void set_Start_Point(Target &t);
		void set_End_Point(Target &t);
};

#endif //GRAPH_H