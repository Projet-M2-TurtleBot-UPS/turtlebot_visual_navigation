# include "Target.hpp"
# include "Map_Gui.hpp"
# include "Map_node.hpp"
# include "Line.hpp"
# include "Image_processing.hpp"
# include "Point_Cloud.hpp"
# include "Graph.hpp"
# include "Set_Marker.hpp"
# include "Recherche.hpp"
# include "Follow_Path.hpp"

# include "Smooth_Path.cpp"

# define INIT_STATE 0
# define LOCALISATION_STATE 1
# define PATH_GENERATION_STATE 2
# define MOVEMENT_STATE 3
# define OBSTACLE_POSITIONNING_STATE 4
# define WAIT_STATE 5

static int current_State = INIT_STATE;

void init(Map_node node){
	if(node.get_Resolution_Map()>0.0)
	{
		ROS_INFO("... MAP PROCESSING ...");
		node.close_Map(2);
		//node.open_Map(2);
		node.dilate_Map();
		node.update_Map();
		current_State = LOCALISATION_STATE;
	}
}

void localisation(Recherche recherche){
	ROS_INFO("... LOCALISATION ...");
	//recherche.start();
	current_State = PATH_GENERATION_STATE;
}

vector<vector<float> > path_Generation(Map_node node, Map_Gui mapgui, vector<Target> list_Markers){
	vector<Target> list_Target;

	vector<float> orientation ;
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(-1.57f);
	orientation.push_back(1.0f);

	vector<float> color2;
	color2.push_back(0.0);
	color2.push_back(0.0);
	color2.push_back(0.0);

	vector<float> green;
	green.push_back(0.0);
	green.push_back(1.0);
	green.push_back(0.0);

	vector<float> start_point;
	start_point.push_back(-2.2f);
	start_point.push_back(1.5f);
	start_point.push_back(0.0f);

	vector<float> end_point;
	end_point.push_back(-9.45f);
	end_point.push_back(-3.35f);
	end_point.push_back(0.0f);

	ROS_INFO("... GENERATION PRM ...");
	Point_Cloud p_C = Point_Cloud(node, list_Markers, start_point);
	list_Target = p_C.create_Point_Cloud();

	Target start_target = Target(list_Target.size()+list_Markers.size(), 1, start_point, orientation);
	Target end_target = Target(list_Target.size()+1+list_Markers.size(), 3, end_point, orientation);
	list_Target.push_back(start_target);
	list_Target.push_back(end_target);

	ROS_INFO("... DISPLAY TARGETS ...");
	mapgui.add_List_Target(0,list_Target);
	
	ROS_INFO("... GENERATION LINKS ...");
	vector<Target>::iterator it1, it2;
	for(int i=0; i<list_Target.size()-1;++i)
	{
		for (int j=i+1;j<list_Target.size();++j)
		{
			Target a = list_Target[i];
			Target b = list_Target[j];
			if(!node.is_intersection(a.get_Position(),b.get_Position(),2.5))
			{
				list_Target[i].add_Son(j);
				list_Target[j].add_Son(i);
				/* ROS_INFO("... DISPLAY GRAPH ...")
				mapgui.add_Line(i*list_Target.size()+j,list_Target[i],list_Target[j],color2);*/
			}
		}
	}

	ROS_INFO("... A* ...");
	Graph graph = Graph(start_target, end_target, list_Target);
	vector<Target> path = graph.a_Star();

	if(path.size() == 0){
		ROS_INFO(" [ROBOT] \"Impossible de trouver un chemin pour aller du start_point au end_point...\"\n");
		exit(0);
	} else {
		ROS_INFO("... DISPLAY PATH ...");
		mapgui.add_Line_List (0,path, color2);
		ROS_INFO("... GENERATION SMOOTH PATH ...");
		vector<vector<float > > smooth_path = Generator_smooth_Path(path,40);
		ROS_INFO("... DISPLAY SMOOTH PATH ...");
		mapgui.add_Line_List_float (1, smooth_path, green);
		current_State = MOVEMENT_STATE;
		return smooth_path;
	}
	
}

void movement(){
	current_State = WAIT_STATE;
}

void obstacle_Positionning(){

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"Map_Main");
	ros::NodeHandle nh;
	Map_node node(nh);
	Map_Gui mapgui(nh);
	Set_Marker sm(nh);
	Recherche recherche(nh);
	Follow_Path followPath(nh);

	vector<Target> list_Markers;
	vector<vector<float> > path;
	list_Markers = sm.init_Markers();

	ros::Rate loop_rate(2);

	while(ros::ok())
	{
		ros::spinOnce();
		switch(current_State){
			case INIT_STATE:
				init(node);
				break;
			case LOCALISATION_STATE:
				localisation(recherche);
				break;
			case PATH_GENERATION_STATE:
				path = path_Generation(node, mapgui, list_Markers);
				break;
			case MOVEMENT_STATE:
				ROS_INFO("... MOVING ...");
				for(unsigned int i=0;i<path.size();++i)
				{
					followPath.goToPoint(path[i]);
				}
				break;
			case OBSTACLE_POSITIONNING_STATE:
				obstacle_Positionning();
				break;
			case WAIT_STATE:
				return 0;
				break;
			default:
				ROS_INFO("ERROR UNKNOWN STATE");
				return 0;
				break;
		}		
		loop_rate.sleep();
	}
	return 0;
}