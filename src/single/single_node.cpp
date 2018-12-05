#include "bluefox2/single_node.h"
#include "bluefox2/bluefox2_ros.h"

namespace bluefox2 {


SingleNode::SingleNode(ros::NodeHandle& pnh)
    : CameraNodeBase(pnh),
      bluefox2_ros_(boost::make_shared<Bluefox2Ros>(pnh)) { 
  fifoReadPos = fifoWritePos = 0;
  nextTriggerCounter = 0;
  outOfSyncCounter = 0;

  pnh.param("ctm", ctm, 1);
  ROS_INFO( "trigger mode = %d", ctm);
  pnh.param("calibration_offset", calibration_offset, 0.0);
  ROS_INFO( "calibration time offset = %fseconds", ros::Duration(calibration_offset).toSec());

  // hardware-trigger enabled
  if (ctm == 3){
    subTimeRef = pnh.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 1000, &bluefox2::SingleNode::callback, this);
  }
}

void SingleNode::callback(const mavros_msgs::CamIMUStamp &time_stamp) {
  //  ros::Duration(0.001).sleep();
  ROS_INFO("received trigger. time: %f", time_stamp.frame_stamp.toSec());
  bluefox2::TriggerPacket_t pkt;
  pkt.triggerTime = time_stamp.frame_stamp;
  pkt.triggerCounter = time_stamp.frame_seq_id;     
  fifoWrite(pkt);
}

void SingleNode::fifoWrite(TriggerPacket_t pkt){
  fifo[fifoWritePos]=pkt;
  fifoWritePos = (fifoWritePos + 1) % FIFO_SIZE;
  if (fifoWritePos == fifoReadPos){
    ROS_WARN("FIFO overflow!");
  }
}

bool SingleNode::fifoRead(TriggerPacket_t &pkt){
  if (fifoReadPos == fifoWritePos) return false;
  pkt = fifo[fifoReadPos];
  fifoReadPos = (fifoReadPos + 1) % FIFO_SIZE;
  return true;
}

bool SingleNode::fifoLook(TriggerPacket_t &pkt){
  if (fifoReadPos == fifoWritePos) return false;
  pkt = fifo[fifoReadPos];
  return true;
}

void SingleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    const auto expose_us = bluefox2_ros_->camera().GetExposeUs();
    const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const auto old_stamp = ros::Time::now() + expose_duration;
    if (ctm != 3){
      // no hardware trigger;
      bluefox2_ros_->RequestSingle();
      bluefox2_ros_->PublishCamera(old_stamp);
      Sleep();
    } else { 
      //  hardware trigger
      bluefox2_ros_->RequestSingle();
      // wait for new trigger
      TriggerPacket_t pkt;
      while (!fifoLook(pkt)) {    
        ros::Duration(0.001).sleep();
        ROS_DEBUG("no trigger received");
      }
      ROS_DEBUG("received frame.   time: %f", ros::Time::now().toSec());
      // a new video frame was captured - check if we need to skip it if one trigger packet was lost
      if (pkt.triggerCounter == nextTriggerCounter) {
        fifoRead(pkt);
        const auto new_stamp = pkt.triggerTime + expose_duration + ros::Duration(calibration_offset);
        bluefox2_ros_->PublishCamera(new_stamp);
        ROS_INFO("publish re-stamped frame. delay pulse to arrival %fms", (old_stamp.toSec() - pkt.triggerTime.toSec()) * 1000.0);
        nextTriggerCounter++;
      } else { 
        ROS_WARN("Sync counters mavros %d -> bluefox %d", pkt.triggerCounter, nextTriggerCounter);
        // determine counter offset
        nextTriggerCounter = pkt.triggerCounter;
      } 
      Sleep();
    }
  }
}

void SingleNode::AcquireOnce() {
  ROS_WARN("AcquireOnce called");
  if (is_acquire() && ros::ok()) {
    bluefox2_ros_->RequestSingle();
    const auto expose_us = bluefox2_ros_->camera().GetExposeUs();
    const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const auto time = ros::Time::now() + expose_duration;
    bluefox2_ros_->PublishCamera(time);
  }
}

void SingleNode::Setup(Bluefox2DynConfig& config) {
  bluefox2_ros_->set_fps(config.fps);
  bluefox2_ros_->camera().Configure(config);

  const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
  const std::string mavros_trig_interval_srv = "/mavros/cmd/trigger_interval";
  if (ros::service::exists(mavros_trig_control_srv, false) && 
          ros::service::exists(mavros_trig_interval_srv, false)) {
      
      // disable trigger until triggering is started
      mavros_msgs::CommandTriggerControl req_disable;
      req_disable.request.trigger_enable = false;
      req_disable.request.sequence_reset = true;
      req_disable.request.trigger_pause = false;
      ros::service::call(mavros_trig_control_srv, req_disable);
      ros::spinOnce();

      // set trigger cycle time
      mavros_msgs::CommandTriggerInterval req_interval;
      req_interval.request.cycle_time = 1000.0/config.fps;
      req_interval.request.integration_time = -1.0;
      ros::service::call(mavros_trig_interval_srv, req_interval);
      ros::spinOnce();

      // enable trigger until triggering is started
      mavros_msgs::CommandTriggerControl req_enable;
      req_enable.request.trigger_enable = true;
      req_enable.request.sequence_reset = false;
      req_enable.request.trigger_pause = false;
      ros::service::call(mavros_trig_control_srv, req_enable);
      ros::spinOnce();

      ROS_INFO("set mavros trigger interval to %f! Success? %d Result? %d",
                       1000.0/config.fps, req_interval.response.success, req_interval.response.result);
  } else {
      ROS_ERROR("Mavros service not available!");
  }
}

}  // namepace bluefox2