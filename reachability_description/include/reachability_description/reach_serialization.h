#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <fstream>
#include <vector>
#include <cstring>

#include <reachability_msgs/msg/reach_data.hpp>


/**
 * @function writeToDisk 
 */
namespace reachability_description
{

template<typename MessageT>
class ReachSerial
{
    public:
    ReachSerial() {}
    
bool writeToDisk(const MessageT &_msg,
                 const std::string &_filename)
{
  rclcpp::Serialization<MessageT> serializer;
  rclcpp::SerializedMessage serialized_msg;

  serializer.serialize_message(&_msg, &serialized_msg);
  rcl_serialized_message_t sm;
  
  sm = serialized_msg.get_rcl_serialized_message();
  /*
  uint8_t buffer[sm.buffer_length];
  uint8_t* bi = &sm.buffer[0];

  for(int i = 0; i < sm.buffer_length; i++)
  {
    buffer[i] = *bi;
    bi++;
  }*/

  std::ofstream output;
  int n = sm.buffer_length;
  output.open(_filename, std::ios::out | std::ios::binary);
  if(!output.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("writeToDisk"), "Error attempting to open file %s ", _filename.c_str());
    return false;
  }

  output.write( reinterpret_cast<char*>(&sm.buffer[0]), std::streamsize(n*sizeof(uint8_t)));
  output.close();
  return  true;
}

/**
 * @function readFromDisk 
 */
bool readFromDisk(const std::string &_filename,
                  MessageT &_msg)
{
  std::ifstream ifs(_filename, std::ios::binary | std::ios::ate);
  if (ifs.fail())
  {
    RCLCPP_ERROR(rclcpp::get_logger("readFromDisk"), "Error attempting to open file %s ", _filename.c_str());
    return false;
  }

  std::ifstream::pos_type pos = ifs.tellg();

  std::vector<uint8_t> file_contents(static_cast<size_t>(pos));

  ifs.seekg(0, std::ios::beg);
  ifs.read(reinterpret_cast<std::ifstream::char_type*>(&file_contents[0]), pos);  // NOLINT
  
  // Reconstruct
  int n = file_contents.size();

  rclcpp::Serialization<MessageT> serializer;
  rclcpp::SerializedMessage serialized_msg(n);

  auto &rcl_handle = serialized_msg.get_rcl_serialized_message();
  
  uint8_t* start_point = &file_contents[0];
  std::memcpy(rcl_handle.buffer, start_point, n);
  
  rcl_handle.buffer_length = n;
  rcl_handle.buffer_capacity = n;
 
  MessageT msg;
  serializer.deserialize_message(&serialized_msg, &msg);

  //Return
  _msg = msg;
  return true;
}

}; // class


} // namespace reachability_description