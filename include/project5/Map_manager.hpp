#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

class Map_manager {
 public:
  Map_manager();
  int get_state(int x, int y);
  void show_image();
  bool checkObstacle(std::vector<int> grid);
  std::vector<float> computeGridCoordinate(std::vector<float> position);
  std::vector<double> computeDistanceCoordinate(std::vector<float> position);
  std::vector<std::vector<int> > getCfree();

 private:
  cv::Mat image;
  std::vector<std::vector<int> > Cfree;

};
