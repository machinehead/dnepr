/* Auto-generated by genmsg_cpp for file /home/egor/dnepr_quadro_control/srv/CameraCrossFinder.srv */
#ifndef DNEPR_QUADRO_CONTROL_SERVICE_CAMERACROSSFINDER_H
#define DNEPR_QUADRO_CONTROL_SERVICE_CAMERACROSSFINDER_H
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

#include "ros/service_traits.h"




namespace dnepr_quadro_control
{
template <class ContainerAllocator>
struct CameraCrossFinderRequest_ {
  typedef CameraCrossFinderRequest_<ContainerAllocator> Type;

  CameraCrossFinderRequest_()
  {
  }

  CameraCrossFinderRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CameraCrossFinderRequest
typedef  ::dnepr_quadro_control::CameraCrossFinderRequest_<std::allocator<void> > CameraCrossFinderRequest;

typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderRequest> CameraCrossFinderRequestPtr;
typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderRequest const> CameraCrossFinderRequestConstPtr;


template <class ContainerAllocator>
struct CameraCrossFinderResponse_ {
  typedef CameraCrossFinderResponse_<ContainerAllocator> Type;

  CameraCrossFinderResponse_()
  : XPos(0)
  , YPos(0)
  , Probability(0)
  {
  }

  CameraCrossFinderResponse_(const ContainerAllocator& _alloc)
  : XPos(0)
  , YPos(0)
  , Probability(0)
  {
  }

  typedef int32_t _XPos_type;
  int32_t XPos;

  typedef int32_t _YPos_type;
  int32_t YPos;

  typedef int32_t _Probability_type;
  int32_t Probability;


  typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CameraCrossFinderResponse
typedef  ::dnepr_quadro_control::CameraCrossFinderResponse_<std::allocator<void> > CameraCrossFinderResponse;

typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderResponse> CameraCrossFinderResponsePtr;
typedef boost::shared_ptr< ::dnepr_quadro_control::CameraCrossFinderResponse const> CameraCrossFinderResponseConstPtr;

struct CameraCrossFinder
{

typedef CameraCrossFinderRequest Request;
typedef CameraCrossFinderResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CameraCrossFinder
} // namespace dnepr_quadro_control

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dnepr_quadro_control/CameraCrossFinderRequest";
  }

  static const char* value(const  ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "57dc642c6923055af3b4113f13634c17";
  }

  static const char* value(const  ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x57dc642c6923055aULL;
  static const uint64_t static_value2 = 0xf3b4113f13634c17ULL;
};

template<class ContainerAllocator>
struct DataType< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dnepr_quadro_control/CameraCrossFinderResponse";
  }

  static const char* value(const  ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
int32 XPos\n\
int32 YPos\n\
\n\
int32 Probability\n\
\n\
\n\
";
  }

  static const char* value(const  ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CameraCrossFinderRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.XPos);
    stream.next(m.YPos);
    stream.next(m.Probability);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CameraCrossFinderResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<dnepr_quadro_control::CameraCrossFinder> {
  static const char* value() 
  {
    return "57dc642c6923055af3b4113f13634c17";
  }

  static const char* value(const dnepr_quadro_control::CameraCrossFinder&) { return value(); } 
};

template<>
struct DataType<dnepr_quadro_control::CameraCrossFinder> {
  static const char* value() 
  {
    return "dnepr_quadro_control/CameraCrossFinder";
  }

  static const char* value(const dnepr_quadro_control::CameraCrossFinder&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "57dc642c6923055af3b4113f13634c17";
  }

  static const char* value(const dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dnepr_quadro_control/CameraCrossFinder";
  }

  static const char* value(const dnepr_quadro_control::CameraCrossFinderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "57dc642c6923055af3b4113f13634c17";
  }

  static const char* value(const dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dnepr_quadro_control/CameraCrossFinder";
  }

  static const char* value(const dnepr_quadro_control::CameraCrossFinderResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // DNEPR_QUADRO_CONTROL_SERVICE_CAMERACROSSFINDER_H

