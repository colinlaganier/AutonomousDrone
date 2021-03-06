// Generated by gencpp from file rgbdslam/rgbdslam_ros_ui_f.msg
// DO NOT EDIT!


#ifndef RGBDSLAM_MESSAGE_RGBDSLAM_ROS_UI_F_H
#define RGBDSLAM_MESSAGE_RGBDSLAM_ROS_UI_F_H

#include <ros/service_traits.h>


#include <rgbdslam/rgbdslam_ros_ui_fRequest.h>
#include <rgbdslam/rgbdslam_ros_ui_fResponse.h>


namespace rgbdslam
{

struct rgbdslam_ros_ui_f
{

typedef rgbdslam_ros_ui_fRequest Request;
typedef rgbdslam_ros_ui_fResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct rgbdslam_ros_ui_f
} // namespace rgbdslam


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_f > {
  static const char* value()
  {
    return "d8f674e014809463d0a122a49c328a89";
  }

  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_f&) { return value(); }
};

template<>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_f > {
  static const char* value()
  {
    return "rgbdslam/rgbdslam_ros_ui_f";
  }

  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_f&) { return value(); }
};


// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_fRequest> should match
// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_f >
template<>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_fRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rgbdslam::rgbdslam_ros_ui_f >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_fRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_fRequest> should match
// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_f >
template<>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_fRequest>
{
  static const char* value()
  {
    return DataType< ::rgbdslam::rgbdslam_ros_ui_f >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_fRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_fResponse> should match
// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_f >
template<>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_fResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rgbdslam::rgbdslam_ros_ui_f >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_fResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_fResponse> should match
// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_f >
template<>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_fResponse>
{
  static const char* value()
  {
    return DataType< ::rgbdslam::rgbdslam_ros_ui_f >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_fResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RGBDSLAM_MESSAGE_RGBDSLAM_ROS_UI_F_H
