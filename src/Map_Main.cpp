# include "Target.hpp"
# include "Map_Gui.hpp"
# include "Map_node.hpp"
# include "Line.hpp"
# include "Image_processing.hpp"
# include "Point_Cloud.hpp"
# include "Graph.hpp"
# include "Graph2.hpp"


# include "Smooth_Path.cpp"


int main(int argc, char **argv)
{
	ros::init(argc,argv,"Map_Main");
	ros::NodeHandle nh;
	Map_node node(nh);
	Map_Gui mapgui(nh);
	vector<Target> list_Target;
	vector<Target> list_Markers;
	ros::Rate loop_rate(2);
	int id_curr = 6;
	
	vector<float> position;
	position.push_back(0.0f);
	position.push_back(0.0f);
	position.push_back(0.0f);

	vector<float> position1;
	position1.push_back(-0.96f);
	position1.push_back(-1.55f);
	position1.push_back(0.0f);

	vector<float> position2;
	position2.push_back(-5.15f);
	position2.push_back(-3.0f);
	position2.push_back(0.0f);

	vector<float> position3;
	position3.push_back(4.0f);
	position3.push_back(1.0f);
	position3.push_back(0.0f);

	vector<float> position4;
	position4.push_back(-0.4f);
	position4.push_back(5.3f);
	position4.push_back(0.0f);

	vector<float> position5;
	position5.push_back(-2.0f);
	position5.push_back(1.0f);
	position5.push_back(0.0f);

	vector<float> orientation ;
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(1.0f);

	vector<float> color2;
	color2.push_back(0.0);
	color2.push_back(0.0);
	color2.push_back(0.0);

	vector<float> green;
	green.push_back(0.0);
	green.push_back(1.0);
	green.push_back(0.0);

	//list_Target.push_back(Target(0,1,position,orientation));
	list_Markers.push_back(Target(0,2,position1,orientation));
	list_Markers.push_back(Target(1,2,position2,orientation));
	/*list_Target.push_back(Target(3,0,position3,orientation));
	list_Target.push_back(Target(4,0,position4,orientation));
	list_Target.push_back(Target(5,0,position5,orientation));*/
	 
	//create_Filter_Turtlebot2(0.354f,0.05f);

	vector<float> start_point;
	//start_point.push_back(-4.5f);
	//start_point.push_back(-1.0f);
	start_point.push_back(-2.2f);
	start_point.push_back(1.5f);
	start_point.push_back(0.0f);

	vector<float> end_point;
	//end_point.push_back(1.0f);
	//end_point.push_back(-0.8f);
	end_point.push_back(-10.0f);
	end_point.push_back(-4.5f);
	end_point.push_back(0.0f);

	while(ros::ok())
	{
		ros::spinOnce();
		if(node.get_Resolution_Map()>0.0)
		{
			ROS_INFO("... MAP PROCESSING ...");
			node.close_Map(2);
			//node.open_Map(2);
			node.dilate_Map();
			node.update_Map();

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
						//Line line(id_curr,*it1,*it2,color2);
						list_Target[i].add_Son(j);
						list_Target[j].add_Son(i);
						mapgui.add_Line(i*list_Target.size()+j,list_Target[i],list_Target[j],color2);
						//id_curr++;
					}
				}
			}

			ROS_INFO("... A* ...");
			Graph2 graph = Graph2(start_target, end_target, list_Target, node);
			vector<Target> path = graph.a_Star();

			

			if(path.size() == 0){
				ROS_INFO(" [ROBOT] \"Impossible de trouver un chemin pour aller du start_point au end_point...\"\n");
			} else {
				/*ROS_INFO("... DISPLAY PATH ...");
				mapgui.add_Line_strip (0,path, color2);*/
				ROS_INFO("... GENERATION SMOOTH PATH ...");
				vector<vector<float > > smooth_path = Generator_smooth_Path(path,40);
				/*ROS_INFO("... DISPLAY SMOOTH PATH ...");
				for(int i =0; i<smooth_path.size()-1; ++i)
					mapgui.add_Line (i, smooth_path[i],smooth_path[i+1], color2);*/

			}

			
			return 0;
		}
		loop_rate.sleep();
	}
}