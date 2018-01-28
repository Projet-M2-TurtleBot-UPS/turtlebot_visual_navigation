/***********************************************************************/
/*                          Graph2.cpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

# include "Graph2.hpp"

//Constructor
	Graph2::Graph2(Target &start_point, Target &end_point, vector<Target> &list_Target, Map_node &node)
	{
		start_point_ = start_point;
		end_point_ = end_point;
		list_Target_= list_Target;
	}

	vector<Target> Graph2::a_Star(){
		
		vector<Target> closed_Target;
		vector<Target> open_Target;

		//printf("%d\n", start_point_.get_Id());
		open_Target.push_back(start_point_);

		//printf("%d\n", open_Target[0].get_Id());
		//printf("1ere : %d\n", open_Target[0].get_Id());
		
		vector<Target> came_From;
		came_From.resize(list_Target_.size());
	
		vector<float> gScore;
		gScore.resize(list_Target_.size());
		for(int i = 0; i< gScore.size(); ++i) gScore[i] = INFINITY;
		gScore[(gScore.size()-2)] = 0.0f;

		vector<float> fScore;
		fScore.resize(list_Target_.size());
		
		for(int i = 0; i< fScore.size(); ++i) fScore[i] = INFINITY;
		fScore[(fScore.size()-2)] = 0.0f;
		
		Target current;
		current.set_Id_Failure();
		vector<Target> failure;
		
		//printf("id_start : %d\n", start_point_.get_Id());

		while (open_Target.size()!=0){
			//printf("1\n");
			//printf("taille closed_Target : %d\n", (int)closed_Target.size());

			int index_Target  = get_Lowest_fScore(fScore, open_Target);
			if(index_Target == -1){
				return failure;
			}
			else {

				current= list_Target_[index_Target];
			}

			//printf("new current = %d\n", current.get_Id());
			if(current.equals(end_point_)){
				
				return reconstruct_path(came_From, current);
			}/* else {
				printf("3\n");
				init_Sons(closed_Target, current);
			}*/
			
			if(current.equals(open_Target.back())){
				//printf("a\n");
				open_Target.pop_back();
			} else  {
				//printf("b\n");
				open_Target.erase(open_Target.begin()+get_Index_From_Target(open_Target, current));
			}
			closed_Target.push_back(current);
			
			
			vector<int> neighbor = current.get_Sons();
			Target it;


			//printf("7\n");
			for(int index = 0; index<neighbor.size(); ++index) {
				int index_Neighbor = neighbor[index];

				it = list_Target_[index_Neighbor];
				
				if(!exist(closed_Target, it)){
					if(!exist(open_Target, it.get_Id())){
						open_Target.push_back(it);
					}
				
					
					
					double tentative_gScore = gScore[index_Target] + current.euclidean_Distance(it)*DISTANCE_COST;
					if(it.get_Type() == 2){
						tentative_gScore += AR_COST;
					}

					if(tentative_gScore < gScore[index_Neighbor]){

						came_From[index_Neighbor] = current;
						//printf("%d cameFrom %d\n", id_Neighbor, current.get_Id());

						gScore[index_Neighbor] = tentative_gScore;
						//printf("gscore %f : %f\n", tentative_gScore, gScore[id_Neighbor]);
						//printf("gscore : %f\n", gScore[id_Neighbor]);

						fScore[index_Neighbor] = gScore[index_Neighbor] + heuristic(it);
						//printf("fscore %f : %f\n", tentative_gScore, fScore[id_Neighbor]);
						//printf("fScore : %f\n", fScore[id_Neighbor]);
					}

				}
			}
		}
		return failure;
	}

	vector<Target> Graph2::reconstruct_path(vector<Target> &came_From, Target &current){
		//printf("a\n");
		vector<Target> total_Path;
		total_Path.push_back(current);
		//printf("b\n");
		/*for(int i=0; i<came_From.size(); ++i){
			printf("XXX: %d came_From : %d\n", i, came_From[i].get_Id());
		}*/
		while(!current.equals(start_point_)){
			//printf("id_current : %d\n", current.get_Id());
			int ind = get_Index_From_Target(list_Target_, current);
			if(ind == -1)
				return total_Path;
			current = came_From[ind];
			
			total_Path.push_back(current);
		}
		total_Path.push_back(start_point_);
		//printf("d\n");
		return total_Path;
	}

	double Graph2::heuristic(Target &current_point){
		return current_point.euclidean_Distance(end_point_);
	}

	int Graph2::get_Lowest_fScore(vector<float> &fScore, vector<Target> &open_Target){
		double min = INFINITY;
		int index_Target = -1;
		//printf("size : %d\n", fScore.size());
		for(int i=0; i<fScore.size(); ++i){
			
			if(exist(open_Target, list_Target_[i])){
				//printf("min : %f\n", min);
				if(fScore[i]<min){
					//printf("i : %d\n", i);
					min = fScore[i];
					index_Target = i;
				}
			}
		}
		return index_Target;
	}



	int Graph2::get_Index_From_Target(vector<Target> list, Target &t){
		for(int i=0; i<list.size(); ++i){
			//printf("id : %d : %d\n", t.get_Id(), targets[i].get_Id());
			if(t.equals(list[i])){
				return i;
			}
		}
		//printf("non\n");
		return -1;
	}

	bool Graph2::exist(vector<Target> &targets, Target &t){
		vector<Target>::iterator it;

		for(it=targets.begin(); it!=targets.end(); ++it)
			if(it->equals(t.get_Id()))
				return true;
		return false;
	}

	bool Graph2::exist(vector<Target> &targets, int id_Target){
		vector<Target>::iterator it;

		for(it=targets.begin(); it!=targets.end(); ++it)
			if(it->equals(id_Target))
				return true;
		return false;
	}

	vector<Target> Graph2::get_neighbor(vector<int> vec_id)
	{
		vector<Target> vec;
		vec.resize(vec_id.size());
		for(int i = 0 ; i< vec_id.size(); ++i)
			vec[i] = list_Target_[vec_id[i]];
		return vec;
	}

	//Getter

	//Setter