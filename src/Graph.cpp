/***********************************************************************/
/*                          Graph.cpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

# include "Graph.hpp"

//Constructor
	Graph::Graph(Target &start_point, Target &end_point, vector<Target> &list_Target, Map_node &node)
	{
		start_point_ = start_point;
		end_point_ = end_point;
		list_Target_= list_Target;
		node_ = node;
		current_Cost_ = 0;
	}

	vector<Target> Graph::a_Star(){
		printf("hello\n");
		Target current = start_point_;
		vector<Target> tabou;
		printf("hello2\n");
		while(!current.equals(end_point_)){
			//init_Sons(current);
			vector<Target> targets = current.get_Sons();
			if(current.get_Type() == 2){
				tabou.push_back(current);
			}
			printf("hello3\n");

			for(int i=0; i<targets.size(); ++i){
				Target son = targets[i];
				float cost = cost_calculation(current, son);
				add_Node(current, son, cost);
			}
			printf("hello4\n");
			current = get_Optimum(tabou);
		}
		return find_Path();
	}

	void Graph::init_Sons(Target &current){
		vector<Target>::iterator it;
		for(it=list_Target_.begin(); it!=list_Target_.end();++it){
			if((!current.equals(*it)) && (!node_.is_intersection(current.get_Position(),it->get_Position())))
			{
				current.add_Son(*it);
			}
		}
	}

	//ADD ROTATION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	float Graph::cost_calculation(Target &mother, Target &son){
		float cost = current_Cost_;
		//Direct cost
		cost += mother.euclidean_Distance(son)/**DISTANCE_COST*/;
		//If son is an AR marker
		/*if(son.get_Type() == 2)
			cost += AR_COST;*/
		//Heuristic
		cost += son.euclidean_Distance(end_point_);
	}

	Target Graph::get_Optimum(vector<Target> &tabou){
		float min_Cost = list_nodes_[0].get_Cost();
		Target min_Target = list_nodes_[0].get_Son();
		Target temp_Target;
		bool is_Tabou;
		int min_Index = 0;
		for(int i = 1; i<list_nodes_.size(); ++i){
			is_Tabou = false;
			for(int j=0; j<tabou.size(); ++j){
				if(list_nodes_[i].get_Son().equals(tabou[j])){
					is_Tabou = true;
				}
			}
			if(!is_Tabou && !list_nodes_[i].is_Treated()){
				temp_Target = list_nodes_[i].get_Son();
				if(list_nodes_[i].get_Cost()<min_Cost){
					min_Cost = list_nodes_[i].get_Cost();
					min_Target = temp_Target;
					min_Index = i;
				}
			}
		}
		current_Cost_ = min_Cost;
		list_nodes_[min_Index].set_Treated(true);
		return temp_Target;
	}

	void Graph::add_Node(Target &mother, Target &son, float cost){
		Node temp_Node;
		for(int i =0; i<list_nodes_.size(); ++i){
			temp_Node = list_nodes_[i];
			if(temp_Node.get_Son().equals(son)){
				if(cost<temp_Node.get_Cost()){
					temp_Node.set_Treated(false);
					temp_Node.set_Mother(mother);
					temp_Node.set_Cost(cost);
				}
				return;
			}
		}
		temp_Node = Node(mother, son, cost);
		list_nodes_.push_back(temp_Node);
	}

	vector<Target> Graph::find_Path(){
		vector<Target> path;
		vector<Target> reverse_path;
		Target current = end_point_;
		int i;
		path.push_back(end_point_);
		while(!current.equals(start_point_)){
			i=0;
			while(!list_nodes_[i].get_Son().equals(current)){
				++i;
			}
			current = list_nodes_[i].get_Mother();
			path.push_back(current);
		}
		path.push_back(start_point_);
		for(int i=path.size()-1; i==0; --i){
			reverse_path.push_back(path[i]);
		}
		return reverse_path;
	}

	//Getter


	//Setter
	void Graph::set_Start_Point(Target &t){
		start_point_ = t;
		list_nodes_.resize(0);
	}

	void Graph::set_End_Point(Target &t){
		end_point_ = t;
		list_nodes_.resize(0);
	}