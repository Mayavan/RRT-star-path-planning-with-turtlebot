#include <math.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>

#include "project5/Map_manager.hpp"

// Structure used to store the Closed Nodes

class Planner
{
public:
  Planner(Map_manager manager, int step_size);
  std::vector<geometry_msgs::PoseStamped> makePlan(std::vector<float> root, std::vector<float> target);
  float calculateDistance(std::vector<float> first_point, std::vector<float> second_point);

private:
  struct Tree
  {
    std::vector<float> node;
    float costToCome;
    std::vector<int> branches;
  };
  std::vector<std::vector<int> > Cfree;
  float region_radius;
  int branch_length;
  float distance_to_target;
  std::vector<Tree> tree;
  Map_manager map;

  std::vector<float> get_random_point();
  std::vector<float> find_nearest(std::vector<float> Xrand);
  bool hasObstacle(std::vector<float> Xnear, std::vector<float> Xnew);
  std::vector<float> new_node(std::vector<float> Xnear, std::vector<float> Xrand);
  std::vector<int> get_neighbourhood(std::vector<float> Xnew);
  std::vector<float> get_best_parent(std::vector<int> neighbourhood);
  long findParent(long position_of_child);

  int randNum(int min, int max);
};
