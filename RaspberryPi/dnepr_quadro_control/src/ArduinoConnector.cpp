//Узел, занимающийся общением с ардуино. Получает значения ручек управления, публикует данные IMU
#include <ros/ros.h>
#include <ros/console.h>
#include <dnepr_quadro_control/RawArduinoControl.h>
#include <sensor_msgs/Imu.h>
#include <fstream>

using namespace std;

ros::Publisher publisher;
fstream arduinoSerialStream;

static inline char read8( char& checksum )
{
	char result;
	arduinoSerialStream.get( result );
	checksum ^= result;
	return result;
}

static inline int read16( char& checksum )
{
	int result = read8( checksum );
	result += (int)read8( checksum ) << 8;
	return result;
}

//input format: time;acc0,acc1,acc2;gyro0,gyro1,gyro2;mag0,mag1,mag2;alt;vario\n
static void readAndPublishImu()
{
	string str;
	arduinoSerialStream >> str;
	ROS_INFO( "%s", str.c_str() );
/*	int millis = 0;
	arduinoSerialStream >> millis;
	ROS_INFO( "Millis = %d", millis );
	arduinoSerialStream.get();//;
	int acc[3];
	for( int i = 0; i < 3; i++ ) {
		arduinoSerialStream >> acc[i];
		ROS_INFO( "acc%d = %d", i, acc[i] );
		arduinoSerialStream.get();//,
	}
	int gyro[3];
	for( int i = 0; i < 3; i++ ) {
		arduinoSerialStream >> gyro[i];
		ROS_INFO( "gyro%d = %d", i, gyro[i] );
		arduinoSerialStream.get();//,
	}
	int mag[3];
	for( int i = 0; i < 3; i++ ) {
		arduinoSerialStream >> mag[i];
		ROS_INFO( "mag%d = %d", i, mag[i] );
		arduinoSerialStream.get();//,
	}
	int alt, vario;
	arduinoSerialStream >> alt;
	ROS_INFO( "alt = %d", alt );
	arduinoSerialStream.get();//;
	arduinoSerialStream >> vario;
	ROS_INFO( "vario = %d", vario );
	arduinoSerialStream.get();//\n*/
}

static void controlRecievedCallback( const dnepr_quadro_control::RawArduinoControl& /*msg*/ )
{
	//ToDo: send
}

int main(int argc, char **argv)
{
	//пока зашиваем в код порт
	arduinoSerialStream.open("/dev/ttyACM0", std::fstream::in | std::fstream::out | std::fstream::app); 

	ros::init(argc, argv, "arduino_connector");
	ros::NodeHandle rosNode;

	publisher = rosNode.advertise<sensor_msgs::Imu>("raw_imu", 1000);


	ros::Subscriber subscriber = rosNode.subscribe( "raw_arduino_control", 1000, controlRecievedCallback );

	while (ros::ok())
	{
		ROS_INFO("In cycle");
		readAndPublishImu();

		ros::spinOnce();
	}
	return 0;
}
