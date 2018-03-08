/***********************************************************************/
/*                          Create_Graph.cpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

# include "Graph.hpp"

//Constructor
Graph::Graph(Target &start_point, Target &end_point, vector<Target> &list_Target)
{
	start_point_ = start_point;
	end_point_ = end_point;
	list_Target_= list_Target;
}

// a_star
// Return a vector of Targets representing the path determined on the graph by an a star algo
vector<Target> Graph::a_Star()
{	
	vector<Target> closed_Target;
	vector<Target> open_Target;
	open_Target.push_back(start_point_);		
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
	
	while (open_Target.size()!=0){
		int index_Target  = get_Lowest_FScore(fScore, open_Target);
		if(index_Target == -1){
			return failure;
		}
		else {

			current= list_Target_[index_Target];
		}

		if(current.equals(end_point_)){
			
			return reconstruct_path(came_From, current);
		}
		
		if(current.equals(open_Target.back())){
			open_Target.pop_back();
		} else  {
			open_Target.erase(open_Target.begin()+get_Index_From_Target(open_Target, current));
		}
		
		closed_Target.push_back(current);
		vector<int> neighbor = current.get_Sons();
		Target it;

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
					gScore[index_Neighbor] = tentative_gScore;
					fScore[index_Neighbor] = gScore[index_Neighbor] + heuristic(it);
				}

			}
		}
	}
	return failure;
}

// reconstruct_Path
// Return the path determined by a star reconstructed in the right order
// came_from: vector containing the Target from wich Target are coming from
// current: the end Target
vector<Target> Graph::reconstruct_path(vector<Target> &came_From, Target &current)
{
	vector<Target> total_Path;
	total_Path.push_back(current);
	while(!current.equals(start_point_)){
		int ind = get_Index_From_Target(list_Target_, current);

		if(ind == -1){
			return total_Path;
		}
		
		current = came_From[ind];
		total_Path.push_back(current);
	}

	total_Path.push_back(start_point_);
	return total_Path;
}

// heuristic
// Return the euclidean distance between the Target current_point and the end Target
// current_point: the Target from wich we want to determine the heuristic
double Graph::heuristic(Target &current_point)
{
	return current_point.euclidean_Distance(end_point_);
}

// get_Lowest_FScore
// Return the index of the best Target to test considering his fScore
// fScore: a vector containing the cost of getting from the start Target to the end Target passing by a given Target 
// open_Target: the vector of the currently discovered Target that are not evalued yet
int Graph::get_Lowest_FScore(vector<float> &fScore, vector<Target> &open_Target)
{	
	double min = INFINITY;
	int index_Target = -1;

	for(int i=0; i<fScore.size(); ++i){
		if(exist(open_Target, list_Target_[i])){
			if(fScore[i]<min){
				min = fScore[i];
				index_Target = i;
			}
		}
	}
	return index_Target;
}


// get_Index_From_Target
// Return the index of a Target in the list
// list: a list of Targets
// t: the target from wich we want the index
int Graph::get_Index_From_Target(vector<Target> list, Target &t)
{
	for(int i=0; i<list.size(); ++i){
		if(t.equals(list[i])){
			return i;
		}
	}
	return -1;
}

// exist
// Return true if the Target exist in the vector of Target, false otherwise
// targets: the vector of Target in wich we want to test the existence of the Target
// t: the target we want to test the existence of
bool Graph::exist(vector<Target> &targets, Target &t)
{
	vector<Target>::iterator it;

	for(it=targets.begin(); it!=targets.end(); ++it)
		if(it->equals(t.get_Id()))
			return true;
	return false;
}

// exist
// Return true if the Target with this id exist in the vector of Target, false otherwise
// targets: the vector of Target in wich we want to test the existence of the Target
// id_Target: the id of the target we want to test the existence of
bool Graph::exist(vector<Target> &targets, int id_Target)
{
	vector<Target>::iterator it;

	for(it=targets.begin(); it!=targets.end(); ++it)
		if(it->equals(id_Target))
			return true;
	return false;
}

/*vector<Target> Graph::get_neighbor(vector<int> vec_id)
{
	vector<Target> vec;
	vec.resize(vec_id.size());
	for(int i = 0 ; i< vec_id.size(); ++i)
		vec[i] = list_Target_[vec_id[i]];
	return vec;
}*/

//Getter

//Setter