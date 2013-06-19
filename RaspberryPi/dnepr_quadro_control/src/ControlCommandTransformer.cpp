//Узел, занимающийся преобразованием команд полёта в "сырые" значения ручек управления (как с джойстика)
#include <ros/ros.h>
#include "dnepr_quadro_control/RawArduinoControl.h"
#include "dnepr_quadro_control/FlightDirection.h"

ros::Publisher resultPublisher = 0;

void flightDirectionRecievedCallback( const dnepr_quadro_control::FlightDirection& /*msg*/ )
{
//ToDo: process 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_control_loop");
  ros::NodeHandle rosNode;

  resultPublisher = rosNode.advertise<RawArduinoControl>("raw_arduino_control", 1000);

  ros::Subscriber flight_direction_sub = rosNode.subscribe( "flight_direction", 1000, flightDirectionRecievedCallback );

  ros::spin();
  return 0;
}
