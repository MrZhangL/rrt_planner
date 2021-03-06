// Generated by gencpp from file robot_move/moveResult.msg
// DO NOT EDIT!


#ifndef ROBOT_MOVE_MESSAGE_MOVERESULT_H
#define ROBOT_MOVE_MESSAGE_MOVERESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_move
{
template <class ContainerAllocator>
struct moveResult_
{
  typedef moveResult_<ContainerAllocator> Type;

  moveResult_()
    : finish(false)  {
    }
  moveResult_(const ContainerAllocator& _alloc)
    : finish(false)  {
  (void)_alloc;
    }



   typedef uint8_t _finish_type;
  _finish_type finish;





  typedef boost::shared_ptr< ::robot_move::moveResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_move::moveResult_<ContainerAllocator> const> ConstPtr;

}; // struct moveResult_

typedef ::robot_move::moveResult_<std::allocator<void> > moveResult;

typedef boost::shared_ptr< ::robot_move::moveResult > moveResultPtr;
typedef boost::shared_ptr< ::robot_move::moveResult const> moveResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_move::moveResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_move::moveResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_move

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'robot_move': ['/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_move::moveResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_move::moveResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_move::moveResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_move::moveResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_move::moveResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_move::moveResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_move::moveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "474a58dbb494a45bb1ca99544cd64e45";
  }

  static const char* value(const ::robot_move::moveResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x474a58dbb494a45bULL;
  static const uint64_t static_value2 = 0xb1ca99544cd64e45ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_move::moveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_move/moveResult";
  }

  static const char* value(const ::robot_move::moveResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_move::moveResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define the result\n\
bool finish\n\
";
  }

  static const char* value(const ::robot_move::moveResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_move::moveResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.finish);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct moveResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_move::moveResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_move::moveResult_<ContainerAllocator>& v)
  {
    s << indent << "finish: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.finish);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MOVE_MESSAGE_MOVERESULT_H
