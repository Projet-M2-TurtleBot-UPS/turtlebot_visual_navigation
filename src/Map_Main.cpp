# include "Target.hpp"
# include "Map_Gui.hpp"
# include "Map_node.hpp"
# include "Line.hpp"
# include "Image_processing.hpp"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"Map_Main");
	ros::NodeHandle nh;
	Map_node node(nh);
	Map_Gui mapgui(nh);
	vector<Target> list_Target;
	ros::Rate loop_rate(2);
	int id_curr = 6;
	
	vector<float> position;
	position.push_back(0.0f);
	position.push_back(0.0f);
	position.push_back(0.0f);

	vector<float> position1;
	position1.push_back(3.4f);
	position1.push_back(4.5f);
	position1.push_back(0.0f);

	vector<float> position2;
	position2.push_back(0.5f);
	position2.push_back(2.4f);
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

	list_Target.push_back(Target(0,1,position,orientation));
	list_Target.push_back(Target(1,2,position1,orientation));
	list_Target.push_back(Target(2,2,position2,orientation));
	list_Target.push_back(Target(3,0,position3,orientation));
	list_Target.push_back(Target(4,0,position4,orientation));
	list_Target.push_back(Target(5,0,position5,orientation));
	
	//create_Filter_Turtlebot2(0.354f,0.05f);

	while(ros::ok())
	{
		ros::spinOnce();
		if(node.get_Resolution_Map()>0.0)
		{
			node.close_Map(2);
			node.open_Map(2);
			node.dilate_Map();
			node.update_Map();
			for(int i=0;i<list_Target.size();i++)
			{
				mapgui.add_Object(list_Target[i].create_MSG_Marker());
			}

			for(int i=0;i<list_Target.size();i++)
			{
				for (int j=i+1;j<(list_Target.size());j++)
				{
					
					if(!node.is_intersection(list_Target[i].get_Position(),list_Target[j].get_Position()))
					{
						Line line(id_curr,list_Target[i],list_Target[j],color2);
						mapgui.add_Object(line.create_MSG_Marker());
						id_curr++;
					}
				}
			}

				

			return 0;
		}

		loop_rate.sleep();
	}
}