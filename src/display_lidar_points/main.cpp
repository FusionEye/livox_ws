//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <vector>

#include "livox_sdk.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#define BUFFER_POINTS                   (32*1024) // must be 2^n
#define POINTS_PER_FRAME                5000      // must < BUFFER_POINTS
#define PACKET_GAP_MISS_TIME            (1500000) // 1.5ms

#define BD_ARGC_NUM                     (4)
#define BD_ARGV_POS                     (1)
#define COMMANDLINE_BD_SIZE             (15)



typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

struct PointCloudQueue {
  LivoxPoint buffer[BUFFER_POINTS];
  volatile uint32_t rd_idx;
  volatile uint32_t wr_idx;
  uint32_t mask;
  uint32_t size;  // must be 2^n
};

typedef struct {
  uint32_t receive_packet_count;
  uint32_t loss_packet_count;
  uint64_t last_timestamp;
} LidarPacketStatistic;

PointCloudQueue point_cloud_queue_pool[kMaxLidarCount];
/* for global publisher use */
ros::Publisher cloud_pub;


/* for device connect use ----------------------------------------------------------------------- */
typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
  LidarPacketStatistic statistic_info;
} DeviceItem;

DeviceItem lidars[kMaxLidarCount];

/* user add broadcast code here */
const char* broadcast_code_list[] = {
  "000000000000001",
};

#define BROADCAST_CODE_LIST_SIZE    (sizeof(broadcast_code_list) / sizeof(intptr_t))

/* total broadcast code, include broadcast_code_list and commandline input */
std::vector<std::string > total_broadcast_code;

/* for pointcloud queue process */
void PointCloudPoolInit(void) {
  for (int i=0; i<kMaxLidarCount; i++) {
    point_cloud_queue_pool[i].rd_idx = 0;
    point_cloud_queue_pool[i].wr_idx = 0;
    point_cloud_queue_pool[i].size = BUFFER_POINTS;
    point_cloud_queue_pool[i].mask = BUFFER_POINTS - 1;
  }
}

uint32_t QueuePop(PointCloudQueue* queue, LivoxPoint* out_point) {
  uint32_t mask = queue->mask;
  uint32_t rd_idx = queue->rd_idx & mask;

  out_point->x = queue->buffer[rd_idx].x;
  out_point->y = queue->buffer[rd_idx].y;
  out_point->z = queue->buffer[rd_idx].z;
  out_point->reflectivity = queue->buffer[rd_idx].reflectivity;

  queue->rd_idx++;
}

uint32_t QueuePush(PointCloudQueue *queue, LivoxPoint * in_point) {
  uint32_t mask = queue->mask;
  uint32_t wr_idx = queue->wr_idx & mask;

  queue->buffer[wr_idx].x = in_point->x;
  queue->buffer[wr_idx].y = in_point->y;
  queue->buffer[wr_idx].z = in_point->z;
  queue->buffer[wr_idx].reflectivity = in_point->reflectivity;

  queue->wr_idx++;
}

uint32_t QueueUsedSize(PointCloudQueue *queue) {
  return (queue->wr_idx - queue->rd_idx) & queue->mask;
}

uint32_t QueueIsFull(PointCloudQueue *queue) {
  return ((queue->wr_idx + 1) == queue->rd_idx);
}

uint32_t QueueIsEmpty(PointCloudQueue *queue) {
  return (queue->rd_idx == queue->wr_idx);
}

/* for pointcloud convert process */
static uint32_t PublishPointcloudData(PointCloudQueue *queue, uint32_t num) {
  /* init point cloud data struct */
  PointCloud::Ptr cloud (new PointCloud);
  cloud->header.frame_id = "livox_frame";
  cloud->height = 1;
  cloud->width = num;

  LivoxPoint points;
  for (unsigned int i = 0; i < num; i++) {
    QueuePop(queue, &points);

    pcl::PointXYZI point;
    point.x = points.x;
    point.y = points.y;
    point.z = points.z;
    point.intensity = (float) points.reflectivity;
    cloud->points.push_back(point);
  }

  cloud_pub.publish(cloud);
}

static void PointCloudConvert(LivoxPoint *p_dpoint, LivoxRawPoint *p_raw_point) {
  p_dpoint->x = p_raw_point->x/1000.0f;
  p_dpoint->y = p_raw_point->y/1000.0f;
  p_dpoint->z = p_raw_point->z/1000.0f;
  p_dpoint->reflectivity = p_raw_point->reflectivity;
}

void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num) {

  LivoxEthPacket *lidar_pack = data;

  if (!data || !data_num) {
    return;
  }

  if (handle >= kMaxLidarCount) {
    return;
  }

  if ((lidar_pack->timestamp_type == kTimestampTypeNoSync) || \
      (lidar_pack->timestamp_type == kTimestampTypePtp) ||\
      (lidar_pack->timestamp_type == kTimestampTypePps)) {
    LidarPacketStatistic *packet_statistic = &lidars[handle].statistic_info;
    uint64_t cur_timestamp = *((uint64_t *)(lidar_pack->timestamp));
    int64_t packet_gap    = cur_timestamp - packet_statistic->last_timestamp;

    packet_statistic->receive_packet_count++;
    if (packet_statistic->last_timestamp) {
      if (packet_gap > PACKET_GAP_MISS_TIME) {
        packet_statistic->loss_packet_count++;
        ROS_INFO("%d miss count : %ld %lu %lu %d", \
               handle, packet_gap,\
               cur_timestamp, packet_statistic->last_timestamp,\
               packet_statistic->loss_packet_count);
      }
    }

    packet_statistic->last_timestamp = cur_timestamp;
  }

  LivoxRawPoint *p_point_data = (LivoxRawPoint *)lidar_pack->data;
  PointCloudQueue *p_queue    = &point_cloud_queue_pool[handle];
  LivoxPoint tmp_point;
  while (data_num) {
    PointCloudConvert(&tmp_point, p_point_data);
    if (QueueIsFull(p_queue)) {
      break;
    }

    QueuePush(p_queue, &tmp_point);
    --data_num;
    p_point_data++;
  }

  return;
}

void PollPointcloudData(void) {
  for (int i = 0; i < kMaxLidarCount; i++) {
    PointCloudQueue *p_queue  = &point_cloud_queue_pool[i];
    if (QueueUsedSize(p_queue) > POINTS_PER_FRAME) {
      //ROS_DEBUG("%d %d %d %d\r\n", i, p_queue->rd_idx, p_queue->wr_idx, QueueUsedSize(p_queue));
      PublishPointcloudData(p_queue, POINTS_PER_FRAME);
    }
  }
}

/** add bd to total_broadcast_code */
void add_broadcast_code(const char* bd_str) {
  total_broadcast_code.push_back(bd_str);
}


/** add bd in broadcast_code_list to total_broadcast_code */
void add_local_broadcast_code(void) {
  for (int i = 0; i < BROADCAST_CODE_LIST_SIZE; ++i) {
    add_broadcast_code(broadcast_code_list[i]);
  }
}

/** add commandline bd to total_broadcast_code */
void add_commandline_broadcast_code(const char* cammandline_str) {
  char* strs = new char[strlen(cammandline_str) + 1];
  strcpy(strs, cammandline_str);

  std::string pattern = "&";
  char* bd_str  = strtok(strs, pattern.c_str());
  while (bd_str != NULL) {
    ROS_INFO("commandline input bd:%s", bd_str);
    if (COMMANDLINE_BD_SIZE == strlen(bd_str)) {
      add_broadcast_code(bd_str);
    } else {
      ROS_INFO("Invalid bd:%s", bd_str);
    }
    bd_str = strtok(NULL, pattern.c_str());
  }

  delete [] strs;
}


/** Callback function of starting sampling. */
void OnSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data) {
  ROS_INFO("OnSampleCallback statue %d handle %d response %d", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      lidars[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    lidars[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data) {
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(uint8_t status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
  if (status != kStatusSuccess) {
    ROS_INFO("Device Query Informations Failed %d", status);
  }
  if (ack) {
    ROS_INFO("firm ver: %d.%d.%d.%d",
             ack->firmware_version[0],
             ack->firmware_version[1],
             ack->firmware_version[2],
             ack->firmware_version[3]);
  }
}

/** Callback function of changing of device state. */
void OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }

  ROS_INFO("OnDeviceChange broadcast code %s update type %d", info->broadcast_code, type);

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, OnDeviceInformation, NULL);
    if (lidars[handle].device_state == kDeviceStateDisconnect) {
      lidars[handle].device_state = kDeviceStateConnect;
      lidars[handle].info = *info;
    }
  } else if (type == kEventDisconnect) {
    lidars[handle].device_state = kDeviceStateDisconnect;
  } else if (type == kEventStateChange) {
    lidars[handle].info = *info;
  }

  if (lidars[handle].device_state == kDeviceStateConnect) {
    ROS_INFO("Device State error_code %d", lidars[handle].info.status.status_code);
    ROS_INFO("Device State working state %d", lidars[handle].info.state);
    ROS_INFO("Device feature %d", lidars[handle].info.feature);
    if (lidars[handle].info.state == kLidarStateNormal) {
      if (lidars[handle].info.type == kDeviceTypeHub) {
        HubStartSampling(OnSampleCallback, NULL);
      } else {
        LidarStartSampling(handle, OnSampleCallback, NULL);
      }
      lidars[handle].device_state = kDeviceStateSampling;
    }
  }
}


void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL) {
    return;
  }

  ROS_INFO("Receive Broadcast Code %s", info->broadcast_code);
  bool found = false;

  for (int i = 0; i < total_broadcast_code.size(); ++i) {
    if (strncmp(info->broadcast_code, total_broadcast_code[i].c_str(), kBroadcastCodeSize) == 0) {
      found = true;
      break;
    }
  }
  if (!found) {
    ROS_INFO("Not in the broacast_code_list, please add it to if want to connect!");
    return;
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, GetLidarData);
    lidars[handle].handle = handle;
    lidars[handle].device_state = kDeviceStateDisconnect;
  }
}


int main(int argc, char **argv) {
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_INFO("Livox-SDK ros demo");

  PointCloudPoolInit();
  if (!Init()) {
    ROS_FATAL("Livox-SDK init fail!");
    return -1;
  }

  add_local_broadcast_code();
  if (argc >= BD_ARGC_NUM) {
    ROS_INFO("Commandline input %s", argv[BD_ARGV_POS]);
    add_commandline_broadcast_code(argv[BD_ARGV_POS]);
  }

  memset(lidars, 0, sizeof(lidars));
  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);

  if (!Start()) {
    Uninit();
    return -1;
  }

  /* ros related */
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle livox_node;
  cloud_pub = livox_node.advertise<PointCloud>("livox/lidar", POINTS_PER_FRAME);

  ros::Time::init();
  ros::Rate r(500); // 500 hz
  while (ros::ok()) {
    PollPointcloudData();
    r.sleep();
  }

  Uninit();
}


