/* Auto-generated by genmsg_cpp for file /home/egor/dnepr/RaspberryPi/tuc_pelican/kinect_utils/msg/flightCommand.msg */
#ifndef KINECT_UTILS_MESSAGE_FLIGHTCOMMAND_H
#define KINECT_UTILS_MESSAGE_FLIGHTCOMMAND_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace kinect_utils
{
template <class ContainerAllocator>
struct flightCommand_ {
  typedef flightCommand_<ContainerAllocator> Type;

  flightCommand_()
  : header()
  , dx(0.0)
  , dy(0.0)
  , dz(0.0)
  , dyaw(0.0)
  , dx_raw(0.0)
  , dy_raw(0.0)
  , dz_raw(0.0)
  , dyaw_raw(0.0)
  {
  }

  flightCommand_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , dx(0.0)
  , dy(0.0)
  , dz(0.0)
  , dyaw(0.0)
  , dx_raw(0.0)
  , dy_raw(0.0)
  , dz_raw(0.0)
  , dyaw_raw(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _dx_type;
  float dx;

  typedef float _dy_type;
  float dy;

  typedef float _dz_type;
  float dz;

  typedef float _dyaw_type;
  float dyaw;

  typedef float _dx_raw_type;
  float dx_raw;

  typedef float _dy_raw_type;
  float dy_raw;

  typedef float _dz_raw_type;
  float dz_raw;

  typedef float _dyaw_raw_type;
  float dyaw_raw;


  typedef boost::shared_ptr< ::kinect_utils::flightCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinect_utils::flightCommand_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct flightCommand
typedef  ::kinect_utils::flightCommand_<std::allocator<void> > flightCommand;

typedef boost::shared_ptr< ::kinect_utils::flightCommand> flightCommandPtr;
typedef boost::shared_ptr< ::kinect_utils::flightCommand const> flightCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::kinect_utils::flightCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::kinect_utils::flightCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace kinect_utils

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::kinect_utils::flightCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::kinect_utils::flightCommand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::kinect_utils::flightCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f07e65da1d7b0cd6af40da29405284d2";
  }

  static const char* value(const  ::kinect_utils::flightCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf07e65da1d7b0cd6ULL;
  static const uint64_t static_value2 = 0xaf40da29405284d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::kinect_utils::flightCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "kinect_utils/flightCommand";
  }

  static const char* value(const  ::kinect_utils::flightCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::kinect_utils::flightCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32 dx\n\
float32 dy\n\
float32 dz\n\
float32 dyaw\n\
float32 dx_raw\n\
float32 dy_raw\n\
float32 dz_raw\n\
float32 dyaw_raw\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::kinect_utils::flightCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::kinect_utils::flightCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::kinect_utils::flightCommand_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::kinect_utils::flightCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.dx);
    stream.next(m.dy);
    stream.next(m.dz);
    stream.next(m.dyaw);
    stream.next(m.dx_raw);
    stream.next(m.dy_raw);
    stream.next(m.dz_raw);
    stream.next(m.dyaw_raw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct flightCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinect_utils::flightCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::kinect_utils::flightCommand_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "dx: ";
    Printer<float>::stream(s, indent + "  ", v.dx);
    s << indent << "dy: ";
    Printer<float>::stream(s, indent + "  ", v.dy);
    s << indent << "dz: ";
    Printer<float>::stream(s, indent + "  ", v.dz);
    s << indent << "dyaw: ";
    Printer<float>::stream(s, indent + "  ", v.dyaw);
    s << indent << "dx_raw: ";
    Printer<float>::stream(s, indent + "  ", v.dx_raw);
    s << indent << "dy_raw: ";
    Printer<float>::stream(s, indent + "  ", v.dy_raw);
    s << indent << "dz_raw: ";
    Printer<float>::stream(s, indent + "  ", v.dz_raw);
    s << indent << "dyaw_raw: ";
    Printer<float>::stream(s, indent + "  ", v.dyaw_raw);
  }
};


} // namespace message_operations
} // namespace ros

#endif // KINECT_UTILS_MESSAGE_FLIGHTCOMMAND_H
