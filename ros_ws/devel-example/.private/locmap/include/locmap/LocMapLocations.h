// Generated by gencpp from file locmap/LocMapLocations.msg
// DO NOT EDIT!


#ifndef LOCMAP_MESSAGE_LOCMAPLOCATIONS_H
#define LOCMAP_MESSAGE_LOCMAPLOCATIONS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <locmap/LocMapLocation.h>

namespace locmap
{
template <class ContainerAllocator>
struct LocMapLocations_
{
  typedef LocMapLocations_<ContainerAllocator> Type;

  LocMapLocations_()
    : header()
    , locations()
    , groups()  {
    }
  LocMapLocations_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , locations(_alloc)
    , groups(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::locmap::LocMapLocation_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::locmap::LocMapLocation_<ContainerAllocator> >> _locations_type;
  _locations_type locations;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _groups_type;
  _groups_type groups;





  typedef boost::shared_ptr< ::locmap::LocMapLocations_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::locmap::LocMapLocations_<ContainerAllocator> const> ConstPtr;

}; // struct LocMapLocations_

typedef ::locmap::LocMapLocations_<std::allocator<void> > LocMapLocations;

typedef boost::shared_ptr< ::locmap::LocMapLocations > LocMapLocationsPtr;
typedef boost::shared_ptr< ::locmap::LocMapLocations const> LocMapLocationsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::locmap::LocMapLocations_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::locmap::LocMapLocations_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::locmap::LocMapLocations_<ContainerAllocator1> & lhs, const ::locmap::LocMapLocations_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.locations == rhs.locations &&
    lhs.groups == rhs.groups;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::locmap::LocMapLocations_<ContainerAllocator1> & lhs, const ::locmap::LocMapLocations_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace locmap

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::locmap::LocMapLocations_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::locmap::LocMapLocations_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::locmap::LocMapLocations_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::locmap::LocMapLocations_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::locmap::LocMapLocations_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::locmap::LocMapLocations_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::locmap::LocMapLocations_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3bd3045c7f0b78ca25575427c64ec7d5";
  }

  static const char* value(const ::locmap::LocMapLocations_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3bd3045c7f0b78caULL;
  static const uint64_t static_value2 = 0x25575427c64ec7d5ULL;
};

template<class ContainerAllocator>
struct DataType< ::locmap::LocMapLocations_<ContainerAllocator> >
{
  static const char* value()
  {
    return "locmap/LocMapLocations";
  }

  static const char* value(const ::locmap::LocMapLocations_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::locmap::LocMapLocations_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"LocMapLocation[] locations\n"
"string[] groups\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: locmap/LocMapLocation\n"
"Header header\n"
"\n"
"geometry_msgs/PoseStamped pose\n"
"string name\n"
"string group\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::locmap::LocMapLocations_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::locmap::LocMapLocations_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.locations);
      stream.next(m.groups);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocMapLocations_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::locmap::LocMapLocations_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::locmap::LocMapLocations_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "locations[]" << std::endl;
    for (size_t i = 0; i < v.locations.size(); ++i)
    {
      s << indent << "  locations[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::locmap::LocMapLocation_<ContainerAllocator> >::stream(s, indent + "    ", v.locations[i]);
    }
    s << indent << "groups[]" << std::endl;
    for (size_t i = 0; i < v.groups.size(); ++i)
    {
      s << indent << "  groups[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.groups[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LOCMAP_MESSAGE_LOCMAPLOCATIONS_H
