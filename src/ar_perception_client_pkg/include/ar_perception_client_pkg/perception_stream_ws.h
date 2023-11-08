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
#ifndef AR_PERCEPTION_CLIENT_PKG_PERCEPTION_STREAM_WS_H
#define AR_PERCEPTION_CLIENT_PKG_PERCEPTION_STREAM_WS_H

#include "ar_perception_client_pkg/nolhmann_json.hpp"
#include <iostream>
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ros/ros.h>

enum class PerceptionStreamStatus {
  INITIALIZED,
  STOPPED,
  STARTED,
  ERROR,
  DOES_NOT_EXIST
};

class PerceptionStreamWS {
 private:
  ix::WebSocket kwebSocket;
  const std::string ksubprotocol = "json-v1-agility";
  std::string kstream_name;
  std::string kflow_control;
  PerceptionStreamStatus kstatus = PerceptionStreamStatus::INITIALIZED;
  const float kretry_time = 0.5; // seconds
  int kstream_port = -1;

 public:
  PerceptionStreamWS(std::string stream_name, std::string flow_control, std::string ip_address,
                       std::string json_api_port){
    this->kstream_name = stream_name;
    this->kflow_control = flow_control;
    std::string url = "ws://" + ip_address + ":" + json_api_port + "/";
    ROS_INFO("perception_stream_ws: Setting url to %s", url.c_str());
    kwebSocket.setUrl(url);
    kwebSocket.addSubProtocol(this->ksubprotocol);
    kwebSocket.disablePerMessageDeflate();

    this->CheckIfStreamExists();
    this->StartStream();
  };

  ~PerceptionStreamWS() {
    this->StopStream();
  };

  int GetStreamPort() {
    if (this->kstatus == PerceptionStreamStatus::STARTED) return this->kstream_port;
    if (this->kstatus == PerceptionStreamStatus::STOPPED || this->kstatus == PerceptionStreamStatus::INITIALIZED) {
      // attempt to start stream
      this->StartStream();
      if (this->kstatus == PerceptionStreamStatus::STARTED) return this->kstream_port;
    }
    return -1; // error
  };

  void StopStream() {
    kwebSocket.setOnMessageCallback(
      [this](const ix::WebSocketMessagePtr &msg){
        if (this->kstatus == PerceptionStreamStatus::STOPPED) return; // if stream is stopped, do nothing
        // check if message is a message
        if (msg->type == ix::WebSocketMessageType::Message) {
          ROS_INFO("%s StopStream: Received message from server", this->kstream_name.c_str());
          ROS_INFO("Message: %s", msg->str.c_str());

          // assume message is a json and parse it
          nlohmann::json j = nlohmann::json::parse(msg->str);

          // check if message is a response to our request
          if (j.at(0) == "perception-stream-stop") {
            this->kstatus = PerceptionStreamStatus::STOPPED;
            ROS_INFO("%s StopStream: Stream stopped", this->kstream_name.c_str());
          }else if (j.at(0) == "error") {
            // check if error message contains 'Does not exist'
            std::string error_message = j.at(1)["info"];
            std::string search_string = "Does not exist";
            if (error_message.find(search_string) != std::string::npos) {
              this->kstatus = PerceptionStreamStatus::DOES_NOT_EXIST;
              ROS_WARN("%s StopStream: Stream does not exist", this->kstream_name.c_str());
            }else{
              this->kstatus = PerceptionStreamStatus::ERROR;
              ROS_ERROR("%s StopStream: Got error", this->kstream_name.c_str());
            }
          }
        }
      }
    );

    kwebSocket.start();
    std::string stop_string_message = "[\"perception-stream-stop\", {\"stream\": "
                                      "\"" +
                                      this->kstream_name + "\"}]";
    while (this->kstatus != PerceptionStreamStatus::STOPPED and 
           this->kstatus != PerceptionStreamStatus::ERROR and 
           this->kstatus != PerceptionStreamStatus::DOES_NOT_EXIST and
           ros::ok()) {
      kwebSocket.send(stop_string_message);
      ros::Duration(this->kretry_time).sleep();
    }
    kwebSocket.stop();
  };

  void StartStream() {
    kwebSocket.setOnMessageCallback(
    [this](const ix::WebSocketMessagePtr &msg) {
      if (this->kstatus == PerceptionStreamStatus::STARTED) return; // if stream is started, do nothing
      // check if message is a message
      if (msg->type == ix::WebSocketMessageType::Message) {
        ROS_INFO("%s StartStream: Received message from server", this->kstream_name.c_str());
        ROS_INFO("Message: %s", msg->str.c_str());

        // assume message is a json and parse it
        nlohmann::json j = nlohmann::json::parse(msg->str);

        // check if message is a response to our request
        if (j.at(0) == "perception-stream-response") {
          this->kstatus = PerceptionStreamStatus::STARTED;
          ROS_INFO("%s StartStream: Stream started at port %s", this->kstream_name.c_str(), j.at(1)["port"].dump().c_str());
          this->kstream_port = std::stoi(j.at(1)["port"].dump());
        }else if (j.at(0) == "error") {
          // check if error message contains 'Duplicate stream'
          std::string error_message = j.at(1)["info"];
          std::string search_string = "Duplicate stream";
          if (error_message.find(search_string) != std::string::npos) {
            this->kstatus = PerceptionStreamStatus::STARTED;
            ROS_WARN("%s StartStream: Stream already started", this->kstream_name.c_str());
          } else{
            this->kstatus = PerceptionStreamStatus::ERROR;
            ROS_ERROR("%s StartStream: Got error", this->kstream_name.c_str());
          }
        }
      }
    }
    );

    kwebSocket.start();
    std::string start_string_message = "[\"perception-stream-start\", {\"stream\": "
                                       "\"" +
                                       this->kstream_name + "\", \"flow-control\": \"" +
                                       this->kflow_control + "\"}]";
    while (this->kstatus != PerceptionStreamStatus::STARTED and 
           this->kstatus != PerceptionStreamStatus::ERROR and 
           ros::ok()) {
      kwebSocket.send(start_string_message);
      ros::Duration(this->kretry_time).sleep();
    }
    kwebSocket.stop();
  };

  void CheckIfStreamExists()
  {
    kwebSocket.setOnMessageCallback(
      [this](const ix::WebSocketMessagePtr &msg) {
        if (this->kstatus == PerceptionStreamStatus::STARTED) return; // if stream is started, do nothing
        // check if message is a message
        if (msg->type == ix::WebSocketMessageType::Message) {
          ROS_INFO("%s CheckIfStreamExists: Received message from server", this->kstream_name.c_str());
          ROS_INFO("Message: %s", msg->str.c_str());
          // assume message is a json and parse it
          nlohmann::json j = nlohmann::json::parse(msg->str);
          // check if message is a response to our request
          if (j.at(0) == "perception-streams") {
            // get list of active streams 
            std::vector<std::string> active_streams = j.at(1)["active-streams"];
            // get list of stream ports
            std::vector<int> stream_ports = j.at(1)["used-ports"];
            // get list of available streams
            std::vector<std::string> available_streams = j.at(1)["available-streams"];
            // check if stream is in list of available streams
            if (std::find(available_streams.begin(), available_streams.end(), this->kstream_name) != available_streams.end()) {
              // stream exists
              this->kstatus = PerceptionStreamStatus::STOPPED;
              ROS_INFO("%s CheckIfStreamExists: Stream exists", this->kstream_name.c_str());
              // get port of stream if it is active
              if (std::find(active_streams.begin(), active_streams.end(), this->kstream_name) != active_streams.end()) {
                // stream is active
                this->kstatus = PerceptionStreamStatus::STARTED;
                // get port of stream
                int stream_index = std::distance(active_streams.begin(), std::find(active_streams.begin(), active_streams.end(), this->kstream_name));
                std::cout<<"stream_index: "<<stream_index<<std::endl;
                this->kstream_port = stream_ports[stream_index];
                ROS_INFO("%s CheckIfStreamExists: Stream is active at port %d", this->kstream_name.c_str(), this->kstream_port);
              }
          } else {
              // stream does not exist
              this->kstatus = PerceptionStreamStatus::ERROR;
              ROS_ERROR("%s CheckIfStreamExists: Stream does not exist", this->kstream_name.c_str());
            }
          }
        }
      }
    );

    kwebSocket.start();
    std::string check_if_stream_exists_string_message = "[\"get-perception-streams\", {}]";
    while (this->kstatus != PerceptionStreamStatus::STARTED and 
           this->kstatus != PerceptionStreamStatus::STOPPED and 
           this->kstatus != PerceptionStreamStatus::ERROR and
           ros::ok()) {
      kwebSocket.send(check_if_stream_exists_string_message);
      ros::Duration(this->kretry_time).sleep();
    }
    kwebSocket.stop();
  }


  
  

};

#endif // AR_PERCEPTION_CLIENT_PKG_PERCEPTION_STREAM_WS_H
