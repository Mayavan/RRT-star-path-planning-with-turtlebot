#include <math.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>

#include "project3/Map_manager.hpp"

// Structure used to store the Closed Nodes

class Planner
{
public:
  Planner(std::vector<std::vector<int> > Cf, Map_manager manager);
  std::vector<geometry_msgs::PoseStamped> makePlan(std::vector<int> start, std::vector<int> goal);
  float calculateDistance(std::vector<float> first_point, std::vector<float> second_point);

private:
  struct Tree
  {
    std::vector<float> node;
    float costToCome;
    std::vector<int> branches;
  };
  std::vector<std::vector<int> > Cfree;
  const int region_radius;
  const int branch_length;
  int distance_to_target;
  std::vector<Tree> tree;

  std::vector<int> get_random_point();
  std::vector<float> find_nearest(std::vector<int> Xrand);
  bool hasObstacle(std::vector<float> Xnear, std::vector<float> Xnew);
  std::vector<float> new_node(std::vector<float> Xnear, std::vector<int> Xrand);
  std::vector<int> get_neighbourhood(std::vector<float> Xnew);
  std::vector<float> get_best_parent(std::vector<int> neighbourhood);

  int randNum(int min, int max);
};
