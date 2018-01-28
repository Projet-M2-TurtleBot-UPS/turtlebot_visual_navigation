/***********************************************************************/
/*                          Node.cpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

#ifndef NODE_H
#define NODE_H

# include "Target.hpp"

class Node
{
	private:
		Target mother_;
		Target son_;
		float cost_;
		bool treated_;

	public:
		//Constructor
		Node(Target &mother, Target &son, float cost);
		Node(){};

		//Destructor
		~Node(){};

		//Getter
		Target get_Mother();
		Target get_Son();
		int get_Cost();
		bool is_Treated();

		//Setter
		void set_Mother(Target &mother);
		void set_Son(Target &son);
		void set_Cost(float cost);
		void set_Treated(bool treated);
};

#endif //NODE_H