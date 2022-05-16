// Generated by gencpp from file rgbdslam/rgbdslam_ros_ui_b.msg
// DO NOT EDIT!


#ifndef RGBDSLAM_MESSAGE_RGBDSLAM_ROS_UI_B_H
#define RGBDSLAM_MESSAGE_RGBDSLAM_ROS_UI_B_H

#include <ros/service_traits.h>


#include <rgbdslam/rgbdslam_ros_ui_bRequest.h>
#include <rgbdslam/rgbdslam_ros_ui_bResponse.h>


namespace rgbdslam
{

struct rgbdslam_ros_ui_b
{

typedef rgbdslam_ros_ui_bRequest Request;
typedef rgbdslam_ros_ui_bResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct rgbdslam_ros_ui_b
} // namespace rgbdslam


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_b > {
  static const char* value()
  {
    return "95aa0151a35e3de365041ffa089ce8c7";
  }

  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_b&) { return value(); }
};

template<>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_b > {
  static const char* value()
  {
    return "rgbdslam/rgbdslam_ros_ui_b";
  }

  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_b&) { return value(); }
};


// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_bRequest> should match
// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_b >
template<>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_bRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rgbdslam::rgbdslam_ros_ui_b >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_bRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_bRequest> should match
// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_b >
template<>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_bRequest>
{
  static const char* value()
  {
    return DataType< ::rgbdslam::rgbdslam_ros_ui_b >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_bRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_bResponse> should match
// service_traits::MD5Sum< ::rgbdslam::rgbdslam_ros_ui_b >
template<>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_bResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rgbdslam::rgbdslam_ros_ui_b >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_bResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_bResponse> should match
// service_traits::DataType< ::rgbdslam::rgbdslam_ros_ui_b >
template<>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_bResponse>
{
  static const char* value()
  {
    return DataType< ::rgbdslam::rgbdslam_ros_ui_b >::value();
  }
  static const char* value(const ::rgbdslam::rgbdslam_ros_ui_bResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RGBDSLAM_MESSAGE_RGBDSLAM_ROS_UI_B_H
