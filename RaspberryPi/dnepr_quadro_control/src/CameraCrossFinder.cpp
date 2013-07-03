#include "ros/ros.h"
#include "dnepr_quadro_control/CameraCrossFinder.h"

bool findCrossCallback(dnepr_quadro_control::CameraCrossFinder::Request  &req,
         dnepr_quadro_control::CameraCrossFinder::Response &res )
{
//ToDo: добавить код Антона
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_cross_finder_server");
  ros::NodeHandle rosNode;

  ros::ServiceServer service = rosNode.advertiseService("camera_find_cross", findCrossCallback);
  ros::spin();

  return 0;
}

