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
#ifndef AR_PERCEPTION_CLIENT_PKG_PERCEPTION_STREAM_SOCKET_H
#define AR_PERCEPTION_CLIENT_PKG_PERCEPTION_STREAM_SOCKET_H

#include <ros/ros.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <iostream>
#include "ar_perception_client_pkg/nolhmann_json.hpp"
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include <iomanip>
#include <chrono>
#include <tf/transform_broadcaster.h>

#define BUFFER_SIZE 8192  // 8KB
#define MIN_DATA_SIZE 500 // to check if we have enough data to read json

enum class ReadState
{
    FIND_JSON,
    READ_DATA,
    PROCESS_DATA
};

struct FrameInfo
{
    std::string format;
    int channels = 0;
    int bit_depth = 0;
    int height = 0;
    int width = 0;
    int size = 0;
    double timestamp_mono;
    double timestamp_utc;
    int count;
    // Eigen::Matrix4d T_world_to_base;
    // Eigen::Matrix4d T_base_to_stream;
    tf::Transform transform_world_to_base;
    tf::Transform transform_base_to_stream;
        // Eigen::Matrix3f camera_matrix;
        // Eigen::Matrix<float,8,1> distortion_coefficients;
        // float pixel_to_depth_scale;
    std::vector<float> camera_matrix;
    std::vector<float> distortion_coefficients;
    float pixel_to_depth_scale;    
    enum DataType
    {
        IMG_TYPE,
        PT_CLOUD_TYPE,
        ERROR_TYPE
    } data_type;
    FrameInfo(){}; // default constructor
    FrameInfo(nlohmann::json &json_msg)
    {
        format = json_msg["format"];
        // ROS_INFO("ar_perception_client_node: Frame format: %s", format.c_str());
        if (format == "RGB8")
        {
            this->channels = 3;
            this->bit_depth = sizeof(uint8_t);
            this->data_type = IMG_TYPE;
        }
        else if (format == "Gray8")
        {
            this->channels = 1;
            this->bit_depth = sizeof(uint8_t);
            this->data_type = IMG_TYPE;
        }
        else if (format == "Depth16")
        {
            this->channels = 1;
            this->bit_depth = sizeof(uint16_t);
            this->data_type = IMG_TYPE;
        }
        else if (format == "XYZ")
        {
            this->channels = 3;
            this->bit_depth = sizeof(float);
            this->data_type = PT_CLOUD_TYPE;
        }
        else if (format == "XYZI")
        {
            this->channels = 4;
            this->bit_depth = sizeof(float);
            this->data_type = PT_CLOUD_TYPE;
        }
        else if (format == "XYZIRT")
        {
            this->channels = 6;
            this->bit_depth = sizeof(float);
            this->data_type = PT_CLOUD_TYPE;
        }
        else
        {
            ROS_ERROR("ar_perception_client_node: Unknown frame format: %s", format.c_str());
            this->data_type = ERROR_TYPE;
            ros::shutdown();
        }

        if (this->data_type == IMG_TYPE)
        {
            // get height, width, size
            this->height = (int)json_msg["height"];
            this->width = (int)json_msg["width"];
            this->size = this->height * this->width * this->channels * this->bit_depth;

            //clear
            this->camera_matrix.clear();
            this->distortion_coefficients.clear();
            // copy camera matrix with for loop where json_msg["camera-matrix"] is a 3x3 array
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    this->camera_matrix.push_back((float)json_msg["camera-matrix"][i][j]);
                }
            }

            // copy distortion coefficients with for loop
            for (int i = 0; i < 8; i++)
            {
                this->distortion_coefficients.push_back((float)json_msg["distortion-coefficients"][i]);
            }

            this->pixel_to_depth_scale = (float)json_msg["pixel-to-depth-scale"];

        }
        else if (this->data_type == PT_CLOUD_TYPE)
        {
            this->size = (int)json_msg["size"] * this->channels * this->bit_depth;
        }
        else
        {
            ROS_ERROR("ar_perception_client_node: Unknown frame type");
            ros::shutdown();
        }

        this->timestamp_mono = (double)json_msg["timestamp-mono"];
        this->timestamp_utc = (double)json_msg["timestamp-utc"];
        this->count = (int)json_msg["count"];

        // TODO: Find a better way to convert json to transform
        // T_world_to_base and T_base_to_stream are have R and t 
        Eigen::Matrix4f T_world_to_base, T_base_to_stream;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                T_world_to_base(i, j) = (float)json_msg["T-world-to-base"][j][i];
                T_base_to_stream(i, j) = (float)json_msg["T-base-to-stream"][j][i];
            }
        }
        this->transform_world_to_base = tf::Transform(tf::Matrix3x3(T_world_to_base(0, 0), T_world_to_base(0, 1), T_world_to_base(0, 2),
                                                                    T_world_to_base(1, 0), T_world_to_base(1, 1), T_world_to_base(1, 2),
                                                                    T_world_to_base(2, 0), T_world_to_base(2, 1), T_world_to_base(2, 2)),
                                                      tf::Vector3(T_world_to_base(0, 3), T_world_to_base(1, 3), T_world_to_base(2, 3)));
        this->transform_base_to_stream = tf::Transform(tf::Matrix3x3(T_base_to_stream(0, 0), T_base_to_stream(0, 1), T_base_to_stream(0, 2),
                                                                     T_base_to_stream(1, 0), T_base_to_stream(1, 1), T_base_to_stream(1, 2),
                                                                     T_base_to_stream(2, 0), T_base_to_stream(2, 1), T_base_to_stream(2, 2)),
                                                       tf::Vector3(T_base_to_stream(0, 3), T_base_to_stream(1, 3), T_base_to_stream(2, 3)));

    }

    void PrintInfo()
    {
        ROS_INFO("ar_perception_client_node: Frame format: %s", format.c_str());
        ROS_INFO("ar_perception_client_node: Frame channels: %d", channels);
        ROS_INFO("ar_perception_client_node: Frame bit depth: %d", bit_depth);
        ROS_INFO("ar_perception_client_node: Frame height: %d", height);
        ROS_INFO("ar_perception_client_node: Frame width: %d", width);
        ROS_INFO("ar_perception_client_node: Frame size: %d", size);
        ROS_INFO("ar_perception_client_node: Frame timestamp mono: %f", timestamp_mono);
        ROS_INFO("ar_perception_client_node: Frame timestamp utc: %f", timestamp_utc);
        ROS_INFO("ar_perception_client_node: Frame count: %d", count);

    }
};

class PerceptionStreamSocket
{
private:
    int ksockfd;
    struct sockaddr_in kserv_addr;
    std::string kstream_name;
    std::string kflow_control;
    std::string kip_address;
    int kstream_port;
    ReadState kread_state = ReadState::FIND_JSON;
    ros::NodeHandle *kptr_nh;

public:
    PerceptionStreamSocket(){}; // default constructor

    PerceptionStreamSocket(std::string stream_name,
                           std::string flow_control,
                           std::string ip_address,
                           int stream_port,
                           ros::NodeHandle *ptr_nh)
    {
        this->kstream_name = stream_name;
        this->kflow_control = flow_control;
        this->kip_address = ip_address;
        this->kstream_port = stream_port;
        this->kptr_nh = ptr_nh;

        // connect to stream
        this->ksockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (this->ksockfd < 0)
        {
            ROS_ERROR("ar_perception_client_node: Error opening socket");
            return;
        }

        this->kserv_addr.sin_family = AF_INET;
        this->kserv_addr.sin_port = htons(this->kstream_port);
        this->kserv_addr.sin_addr.s_addr = inet_addr(this->kip_address.c_str());

        if (connect(this->ksockfd, (struct sockaddr *)&this->kserv_addr, sizeof(this->kserv_addr)) < 0)
        {
            ROS_ERROR("ar_perception_client_node: Error connecting to stream");
            return;
        }
    };

    ~PerceptionStreamSocket()
    {
        close(this->ksockfd);
    };

    // Function that finds json message in the char vector and removes it
    // Returns empty string if no json message is found
    void FindJsonAndRemoveIt(std::vector<char> &data, std::string &json)
    {

        std::string buffer_str(data.begin(), data.end());
        int start = buffer_str.find("{");
        int end = buffer_str.find("}]");
        if (start != std::string::npos && end != std::string::npos)
        {
            json = buffer_str.substr(start, end - start + 1);
        }
        else
        {
            json = "";
        }
        data.erase(data.begin(), data.begin() + end + 2); // +2 to remove the "}]"
    }

    void ProcessStreamAndPublish()
    {
        int read_chunk_size = BUFFER_SIZE;
        char buffer[BUFFER_SIZE];
        std::vector<char> data;
        ReadState read_state = ReadState::FIND_JSON; // reset read state
        FrameInfo frame_info;
        bool is_first_frame = true;

        FrameInfo::DataType this_stream_data_type;

        // create publisher
        ros::Publisher pub;
        // tf broadcaster
        tf::TransformBroadcaster br;



        while (ros::ok())
        {
            // read from stream
            int n = read(this->ksockfd, buffer, read_chunk_size);
            if (n < 0)
            {
                ROS_ERROR("ar_perception_client_node: Error reading from stream");
                return;
            }

            // add read data to data vector
            data.insert(data.end(), buffer, buffer + n);

            if (read_state == ReadState::FIND_JSON)
            {
                std::string json_str;
                if (data.size() < MIN_DATA_SIZE)
                { //
                    // wait for some time
                    // ros::Duration(0.01).sleep();
                    continue;
                }
                this->FindJsonAndRemoveIt(data, json_str);
                if (json_str == "")
                {
                    continue;
                }
                // ROS_INFO("ar_perception_client_node: Found json: %s", json_str.c_str());
                nlohmann::json json_msg = nlohmann::json::parse(json_str);
                frame_info = FrameInfo(json_msg);
                // frame_info.PrintInfo();
                read_state = ReadState::READ_DATA;

                if (is_first_frame)
                {
                    is_first_frame = false;
                    // convert '-' to '_' in stream name
                    std::string stream_name = this->kstream_name;
                    std::replace(stream_name.begin(), stream_name.end(), '-', '_');
                    // create publisher
                    if (frame_info.format == "RGB8" or frame_info.format == "Gray8" or frame_info.format == "Depth16") {
                        pub = this->kptr_nh->advertise<sensor_msgs::Image>(stream_name, 1);
                        // ros param for camera matrix
                        ros::param::set(stream_name + "/camera_matrix_colm", frame_info.camera_matrix);
                        // ros param for distortion coefficients
                        ros::param::set(stream_name + "/distortion_coefficients", frame_info.distortion_coefficients);
                        // ros param for pixel to depth scale
                        ros::param::set(stream_name + "/pixel_to_depth_scale", frame_info.pixel_to_depth_scale);



                    } else if (frame_info.format == "XYZ" or frame_info.format == "XYZI" or frame_info.format == "XYZIRT") {
                        pub = this->kptr_nh->advertise<sensor_msgs::PointCloud2>(stream_name, 1);
                    }

                    this_stream_data_type = frame_info.data_type;


                }

                // Check if data type is same as this stream
                if (frame_info.data_type != this_stream_data_type)
                {
                    ROS_ERROR("ar_perception_client_node: Data type of stream %s is not same as this stream", this->kstream_name.c_str());
                    ros::shutdown();
                }
                
            }

            if (read_state == ReadState::READ_DATA)
            {
                // read data until we match the size of frame_info.size
                if (frame_info.size <= data.size())
                {
                    read_state = ReadState::PROCESS_DATA;
                }
                else if (frame_info.size - data.size() < read_chunk_size)
                {
                    read_chunk_size = frame_info.size - data.size();
                }
            }

            if (read_state == ReadState::PROCESS_DATA)
            {

                read_chunk_size = BUFFER_SIZE;     // reset read chunk size
                read_state = ReadState::FIND_JSON; // reset read state

                // create timestamp from frame_info.timestamp_mono and frame_info.timestamp_utc
                ros::Time timestamp;
                timestamp = ros::Time(frame_info.timestamp_utc); 

                // transform sensor to base frame
                

                br.sendTransform(tf::StampedTransform(frame_info.transform_base_to_stream,
                                                      timestamp,
                                                      this->kstream_name + "/base_frame",
                                                      this->kstream_name + "/sensor_frame"));
                br.sendTransform(tf::StampedTransform(frame_info.transform_world_to_base,
                                                      timestamp,
                                                      "world",
                                                      this->kstream_name + "/base_frame"));

                if (frame_info.format == "XYZIRT")
                {
                    // convert data to Eigen matrix assume XYZIRT and float by mapping
                    const int num_points = frame_info.size / (frame_info.channels * frame_info.bit_depth);

                    // create point cloud 2 message
                    sensor_msgs::PointCloud2 pt_cloud_msg;
                    pt_cloud_msg.header.frame_id = this->kstream_name +"/sensor_frame";
                    pt_cloud_msg.header.stamp = timestamp;
                    pt_cloud_msg.height = 1;
                    pt_cloud_msg.width = num_points;
                    pt_cloud_msg.fields.resize(6);
                    pt_cloud_msg.fields[0].name = "x";
                    pt_cloud_msg.fields[0].offset = 0;
                    pt_cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[0].count = 1;
                    pt_cloud_msg.fields[1].name = "y";
                    pt_cloud_msg.fields[1].offset = 4;
                    pt_cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[1].count = 1;
                    pt_cloud_msg.fields[2].name = "z";
                    pt_cloud_msg.fields[2].offset = 8;
                    pt_cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[2].count = 1;
                    pt_cloud_msg.fields[3].name = "intensity";
                    pt_cloud_msg.fields[3].offset = 12;
                    pt_cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[3].count = 1;
                    pt_cloud_msg.fields[4].name = "ring";
                    pt_cloud_msg.fields[4].offset = 16;
                    pt_cloud_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[4].count = 1;
                    pt_cloud_msg.fields[5].name = "time";
                    pt_cloud_msg.fields[5].offset = 20;
                    pt_cloud_msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[5].count = 1;
                    pt_cloud_msg.is_bigendian = false;
                    pt_cloud_msg.point_step = 24;
                    pt_cloud_msg.row_step = pt_cloud_msg.point_step * pt_cloud_msg.width;
                    pt_cloud_msg.is_dense = true;
                    pt_cloud_msg.data.resize(pt_cloud_msg.row_step * pt_cloud_msg.height);

                    // directly use data
                    memcpy(&pt_cloud_msg.data[0], data.data(), pt_cloud_msg.data.size());

                    pub.publish(pt_cloud_msg);
                } else if (frame_info.format == "XYZI") {
                    // convert data to Eigen matrix assume XYZI and float by mapping
                    const int num_points = frame_info.size / (frame_info.channels * frame_info.bit_depth);

                    // create point cloud 2 message
                    sensor_msgs::PointCloud2 pt_cloud_msg;
                    pt_cloud_msg.header.frame_id = this->kstream_name +"/sensor_frame";
                    pt_cloud_msg.header.stamp = timestamp;
                    pt_cloud_msg.height = 1;
                    pt_cloud_msg.width = num_points;
                    pt_cloud_msg.fields.resize(4);
                    pt_cloud_msg.fields[0].name = "x";
                    pt_cloud_msg.fields[0].offset = 0;
                    pt_cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[0].count = 1;
                    pt_cloud_msg.fields[1].name = "y";
                    pt_cloud_msg.fields[1].offset = 4;
                    pt_cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[1].count = 1;
                    pt_cloud_msg.fields[2].name = "z";
                    pt_cloud_msg.fields[2].offset = 8;
                    pt_cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[2].count = 1;
                    pt_cloud_msg.fields[3].name = "intensity";
                    pt_cloud_msg.fields[3].offset = 12;
                    pt_cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[3].count = 1;
                    pt_cloud_msg.is_bigendian = false;
                    pt_cloud_msg.point_step = 16;
                    pt_cloud_msg.row_step = pt_cloud_msg.point_step * pt_cloud_msg.width;
                    pt_cloud_msg.is_dense = true;
                    pt_cloud_msg.data.resize(pt_cloud_msg.row_step * pt_cloud_msg.height);

                    // directly use data
                    memcpy(&pt_cloud_msg.data[0], data.data(), pt_cloud_msg.data.size());

                    pub.publish(pt_cloud_msg);
                } else if (frame_info.format == "XYZ") {
                    // convert data to Eigen matrix assume XYZ and float by mapping
                    const int num_points = frame_info.size / (frame_info.channels * frame_info.bit_depth);

                    // create point cloud
                    sensor_msgs::PointCloud2 pt_cloud_msg;
                    pt_cloud_msg.header.frame_id = this->kstream_name +"/sensor_frame";
                    pt_cloud_msg.header.stamp = timestamp;
                    pt_cloud_msg.height = 1;
                    pt_cloud_msg.width = num_points;
                    pt_cloud_msg.fields.resize(3);
                    pt_cloud_msg.fields[0].name = "x";
                    pt_cloud_msg.fields[0].offset = 0;
                    pt_cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[0].count = 1;
                    pt_cloud_msg.fields[1].name = "y";
                    pt_cloud_msg.fields[1].offset = 4;
                    pt_cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[1].count = 1;
                    pt_cloud_msg.fields[2].name = "z";
                    pt_cloud_msg.fields[2].offset = 8;
                    pt_cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
                    pt_cloud_msg.fields[2].count = 1;
                    pt_cloud_msg.is_bigendian = false;
                    pt_cloud_msg.point_step = 12;
                    pt_cloud_msg.row_step = pt_cloud_msg.point_step * pt_cloud_msg.width;
                    pt_cloud_msg.is_dense = true;
                    pt_cloud_msg.data.resize(pt_cloud_msg.row_step * pt_cloud_msg.height);

                    // directly use data
                    memcpy(&pt_cloud_msg.data[0], data.data(), pt_cloud_msg.data.size());

                    pub.publish(pt_cloud_msg);
                } else if (frame_info.format == "RGB8") {
                    sensor_msgs::Image img_msg;
                    img_msg.header.frame_id = this->kstream_name +"/sensor_frame";
                    img_msg.header.stamp = timestamp;
                    img_msg.height = frame_info.height;
                    img_msg.width = frame_info.width;
                    img_msg.encoding = sensor_msgs::image_encodings::RGB8;
                    img_msg.is_bigendian = false;
                    img_msg.step = frame_info.width * frame_info.channels * frame_info.bit_depth;
                    img_msg.data.resize(img_msg.step * img_msg.height);
                    
                    // directly use data
                    memcpy(&img_msg.data[0], data.data(), img_msg.data.size());
                    
                    pub.publish(img_msg);
                
                } else if (frame_info.format == "Gray8") {
                    sensor_msgs::Image img_msg;
                    img_msg.header.frame_id = this->kstream_name +"/sensor_frame";
                    img_msg.header.stamp = timestamp;
                    img_msg.height = frame_info.height;
                    img_msg.width = frame_info.width;
                    img_msg.encoding = sensor_msgs::image_encodings::MONO8;
                    img_msg.is_bigendian = false;
                    img_msg.step = frame_info.width * frame_info.channels * frame_info.bit_depth;
                    img_msg.data.resize(img_msg.step * img_msg.height);

                    // directly use data
                    memcpy(&img_msg.data[0], data.data(), img_msg.data.size());

                    pub.publish(img_msg);
                } else if (frame_info.format == "Depth16") {
                    sensor_msgs::Image img_msg;
                    img_msg.header.frame_id = this->kstream_name +"/sensor_frame";
                    img_msg.header.stamp = timestamp;
                    img_msg.height = frame_info.height;
                    img_msg.width = frame_info.width;
                    img_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                    img_msg.is_bigendian = false;
                    img_msg.step = frame_info.width * frame_info.channels * frame_info.bit_depth;
                    img_msg.data.resize(img_msg.step * img_msg.height);

                    // directly use data
                    memcpy(&img_msg.data[0], data.data(), img_msg.data.size());

                    pub.publish(img_msg);
                }

                data.clear(); // VERY IMPORTANT
            }
        }
    };
};

#endif // AR_PERCEPTION_CLIENT_PKG_PERCEPTION_STREAM_SOCKET_H