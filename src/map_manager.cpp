#include "project3/map_manager.hpp"

using namespace cv;

map_manager::map_manager(ros::NodeHandle nh){
  ros::ServiceClient client =
      nh.serviceClient<nav_msgs::GetMap>("static_map");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would
  // fail.
  if (!client.exists()) {
    ROS_INFO("Waiting for map_server...");
    client.waitForExistence();
    ROS_INFO("map_server is now ready.");
  }
  
  nav_msgs::GetMap srv;  // Combination of the "request" and the "response".
  client.call(srv); // Call the start Service.
  
  map = srv.response.map;
}


// returns the color at the given coordinate
int map_manager::get_state(int x, int y){	
  int point = map.data[y*map.info.width + x];

  ROS_INFO_STREAM("["<<x<<", "<<y<<"] :"<< point);

  
	if(point == -1)
    return 127;
  else if(point > 50)
    return 0;
  else 
    return 255; 


}

// show the read image in a window
void map_manager::show_image(){
  int I[384][384];

  for(int i = 130; i < 240; i ++)
    for(int j = 130; j < 240; j ++)
    {
      I[i][j] = map_manager::get_state(i, j);
    }

  image = cv::Mat(384, 384, CV_8UC1, I);

  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", image); 
  waitKey(60000);
}

// Check if the particular grid has obstacle
bool map_manager::checkObstacle(std::vector<int> grid){
  if (get_state(grid[0],grid[1])==0)
    return true;
  else
    return false;
}

// get position in meter and return in pixel coordinates in image
std::vector<int> map_manager::computeGridPosition(std::vector<float> position){
  std::vector<int> grid;
  grid.push_back(floor(position[0]*20));
  grid.push_back(floor(position[1]*20));
  return grid;
}