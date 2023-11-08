/* 
MIT License

Copyright (c) 2023 Abhijeet M. Kulkarni

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ar_perception_client_pkg/nolhmann_json.hpp"
#include "ar_perception_client_pkg/perception_stream_ws.h"
#include "ar_perception_client_pkg/perception_stream_socket.h"
#include "ros/ros.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <iostream>
#include <thread>




int main(int argc, char **argv) {
  ros::init(argc, argv, "ar_perception_client_node");
  ROS_INFO("Starting ar_perception_client_node");
  ros::NodeHandle nh;


  std::string flow_control = "none";
  std::string ip_address = "10.10.1.1";
  std::string json_api_port = "8080";


  //TODO: Not working cpu affinity
  cpu_set_t my_set;        /* Define your cpu_set bit mask. */
  CPU_ZERO(&my_set);       /* Initialize it all to 0, i.e. no CPUs selected. */
  CPU_SET(1, &my_set);     /* set the bit that represents core 1. */
  sched_setaffinity(0, sizeof(cpu_set_t), &my_set); /* Set affinity of tihs process to */
                                                    /* the defined mask, i.e. only 1. */


  std::string stream_name;
  // ros::param::set("stream_name", "forward-tis-dfm27up/color/image-raw");  
  ros::param::get("~stream_name", stream_name);
  // std::cout<<"stream_name: "<<stream_name<<"\n";
  std::cout<<"stream_name: "<<stream_name<<"\n";
  PerceptionStreamWS perception_stream_ws(stream_name, flow_control, ip_address, json_api_port);
  int stream_port = perception_stream_ws.GetStreamPort();
  PerceptionStreamSocket perception_stream_socket(stream_name, flow_control, ip_address, stream_port, &nh);
  perception_stream_socket.ProcessStreamAndPublish();


  return 0;
}