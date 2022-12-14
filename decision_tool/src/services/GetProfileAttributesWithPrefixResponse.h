/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/foukas/hobbit/src/decision_service/srv/GetProfileAttributesWithPrefix.srv
 *
 */


#ifndef DECISION_SERVICE_MESSAGE_GETPROFILEATTRIBUTESWITHPREFIXRESPONSE_H
#define DECISION_SERVICE_MESSAGE_GETPROFILEATTRIBUTESWITHPREFIXRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace decision_service
{
template <class ContainerAllocator>
struct GetProfileAttributesWithPrefixResponse_
{
  typedef GetProfileAttributesWithPrefixResponse_<ContainerAllocator> Type;

  GetProfileAttributesWithPrefixResponse_()
    : names_values()  {
    }
  GetProfileAttributesWithPrefixResponse_(const ContainerAllocator& _alloc)
    : names_values(_alloc)  {
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _names_values_type;
  _names_values_type names_values;




  typedef boost::shared_ptr< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct GetProfileAttributesWithPrefixResponse_

typedef ::decision_service::GetProfileAttributesWithPrefixResponse_<std::allocator<void> > GetProfileAttributesWithPrefixResponse;

typedef boost::shared_ptr< ::decision_service::GetProfileAttributesWithPrefixResponse > GetProfileAttributesWithPrefixResponsePtr;
typedef boost::shared_ptr< ::decision_service::GetProfileAttributesWithPrefixResponse const> GetProfileAttributesWithPrefixResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace decision_service

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b62ac2def01d0e2027ee2fa0f3912a71";
  }

  static const char* value(const ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb62ac2def01d0e20ULL;
  static const uint64_t static_value2 = 0x27ee2fa0f3912a71ULL;
};

template<class ContainerAllocator>
struct DataType< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "decision_service/GetProfileAttributesWithPrefixResponse";
  }

  static const char* value(const ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
string[] names_values\n\
\n\
\n\
";
  }

  static const char* value(const ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.names_values);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GetProfileAttributesWithPrefixResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::decision_service::GetProfileAttributesWithPrefixResponse_<ContainerAllocator>& v)
  {
    s << indent << "names_values[]" << std::endl;
    for (size_t i = 0; i < v.names_values.size(); ++i)
    {
      s << indent << "  names_values[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.names_values[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DECISION_SERVICE_MESSAGE_GETPROFILEATTRIBUTESWITHPREFIXRESPONSE_H
