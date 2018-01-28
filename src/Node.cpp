/***********************************************************************/
/*                          Node.cpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

# include "Graph.hpp"

//Constructor
	Node::Node(Target &mother, Target &son, float cost)
	{
		mother_ = mother;
		son_ = son;
		cost = cost;
		treated_ = false;
	}

//Getter
Target Node::get_Mother(){return mother_;}
Target Node::get_Son(){return son_;}
int Node::get_Cost(){return cost_;}
bool Node::is_Treated(){return treated_;}

//setter
void Node::set_Mother(Target &mother){mother_ = mother;}
void Node::set_Son(Target &son){son_ = son;}
void Node::set_Cost(float cost){cost_ = cost;}
void Node::set_Treated(bool treated){treated_ = treated;}