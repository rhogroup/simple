/* Auto-generated by genmsg_cpp for file /home/user/fuerte_workspace/sandbox/simple/srv/controller_pub.srv */
#ifndef SIMPLE_SERVICE_CONTROLLER_PUB_H
#define SIMPLE_SERVICE_CONTROLLER_PUB_H
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




namespace simple
{
template <class ContainerAllocator>
struct controller_pubRequest_ {
  typedef controller_pubRequest_<ContainerAllocator> Type;

  controller_pubRequest_()
  : publish_active(false)
  {
  }

  controller_pubRequest_(const ContainerAllocator& _alloc)
  : publish_active(false)
  {
  }

  typedef uint8_t _publish_active_type;
  uint8_t publish_active;


  typedef boost::shared_ptr< ::simple::controller_pubRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simple::controller_pubRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct controller_pubRequest
typedef  ::simple::controller_pubRequest_<std::allocator<void> > controller_pubRequest;

typedef boost::shared_ptr< ::simple::controller_pubRequest> controller_pubRequestPtr;
typedef boost::shared_ptr< ::simple::controller_pubRequest const> controller_pubRequestConstPtr;


template <class ContainerAllocator>
struct controller_pubResponse_ {
  typedef controller_pubResponse_<ContainerAllocator> Type;

  controller_pubResponse_()
  : success(false)
  {
  }

  controller_pubResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


  typedef boost::shared_ptr< ::simple::controller_pubResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simple::controller_pubResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct controller_pubResponse
typedef  ::simple::controller_pubResponse_<std::allocator<void> > controller_pubResponse;

typedef boost::shared_ptr< ::simple::controller_pubResponse> controller_pubResponsePtr;
typedef boost::shared_ptr< ::simple::controller_pubResponse const> controller_pubResponseConstPtr;

struct controller_pub
{

typedef controller_pubRequest Request;
typedef controller_pubResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct controller_pub
} // namespace simple

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::simple::controller_pubRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::simple::controller_pubRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::simple::controller_pubRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e645823275114813776049e46258ef11";
  }

  static const char* value(const  ::simple::controller_pubRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe645823275114813ULL;
  static const uint64_t static_value2 = 0x776049e46258ef11ULL;
};

template<class ContainerAllocator>
struct DataType< ::simple::controller_pubRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "simple/controller_pubRequest";
  }

  static const char* value(const  ::simple::controller_pubRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::simple::controller_pubRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
bool publish_active\n\
\n\
\n\
";
  }

  static const char* value(const  ::simple::controller_pubRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::simple::controller_pubRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::simple::controller_pubResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::simple::controller_pubResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::simple::controller_pubResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::simple::controller_pubResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::simple::controller_pubResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "simple/controller_pubResponse";
  }

  static const char* value(const  ::simple::controller_pubResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::simple::controller_pubResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
\n\
";
  }

  static const char* value(const  ::simple::controller_pubResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::simple::controller_pubResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::simple::controller_pubRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.publish_active);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct controller_pubRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::simple::controller_pubResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct controller_pubResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<simple::controller_pub> {
  static const char* value() 
  {
    return "4d388af0f032c2121f3128ba7ec274c9";
  }

  static const char* value(const simple::controller_pub&) { return value(); } 
};

template<>
struct DataType<simple::controller_pub> {
  static const char* value() 
  {
    return "simple/controller_pub";
  }

  static const char* value(const simple::controller_pub&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<simple::controller_pubRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4d388af0f032c2121f3128ba7ec274c9";
  }

  static const char* value(const simple::controller_pubRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<simple::controller_pubRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "simple/controller_pub";
  }

  static const char* value(const simple::controller_pubRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<simple::controller_pubResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4d388af0f032c2121f3128ba7ec274c9";
  }

  static const char* value(const simple::controller_pubResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<simple::controller_pubResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "simple/controller_pub";
  }

  static const char* value(const simple::controller_pubResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // SIMPLE_SERVICE_CONTROLLER_PUB_H

