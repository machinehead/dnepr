//Узел, занимающийся общением с ардуино. Получает значения ручек управления, публикует данные IMU
#include <ros/ros.h>
#include <dnepr_quadro_control/RawArduinoControl.h>
#include <sensor_msgs/Imu.h>

void controlRecievedCallback( const dnepr_quadro_control::RawArduinoControl& /*msg*/ )
{
//ToDo: send
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_connector");
  ros::NodeHandle rosNode;

  ros::Subscriber subscriber = rosNode.subscribe( "raw_arduino_control", 1000, controlRecievedCallback );
  ros::Publisher publisher = rosNode.advertise<sensor_msgs::Imu>("raw_imu", 1000);

  ros::Rate loop_rate(100);//можно будет настроить потом частоту

  while (ros::ok())
  {
    //Тут будет магия по получению данных с ардуино


    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
