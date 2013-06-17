//Узел, занимающийся основным управлением. Получает данные со всех датчиков, формирует команды полета
#include <ros/ros.h>
#include <dnepr_quadro_control/FlightDirection.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>

//ToDo:вероятно, надо будет сделать получатель PointCloud'ов и камеру nodelet'ами

void pointCloudRecievedCallback( const sensor_msgs::PointCloud& /*msg*/ )
{
//ToDo: process
}

void imuRecievedCallback( const sensor_msgs::Imu& /*msg*/ )
{
//ToDo: process
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_control_loop");
  ros::NodeHandle rosNode;

  ros::Subscriber pointCloudSubscriber = rosNode.subscribe( "point_cloud_source", 1000, pointCloudRecievedCallback );
  ros::Subscriber imuSubscriber = rosNode.subscribe( "raw_imu", 1000, imuRecievedCallback );
  ros::Publisher flightDirectionPublisher = rosNode.advertise<dnepr_quadro_control::FlightDirection>("flight_direction", 1000);

  ros::ServiceClient cameraClient = rosNode.serviceClient<dnepr_quadro_control::CameraCrossFinder>("camera_find_cross");

  while (ros::ok())
  {
    //Тут будет магия

    ros::spinOnce();
  }
  return 0;
}
