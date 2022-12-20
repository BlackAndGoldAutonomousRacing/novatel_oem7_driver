////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>

#include <driver_parameter.hpp>



#include <novatel_oem7_driver/oem7_ros_messages.hpp>
#include "novatel_oem7_driver/oem7_messages.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/imu.hpp"

#include "novatel_oem7_msgs/msg/corrimu.hpp"
#include "novatel_oem7_msgs/msg/imuratecorrimu.hpp"
#include "novatel_oem7_msgs/msg/insstdev.hpp"
#include "novatel_oem7_msgs/msg/insconfig.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"

#include <oem7_ros_publisher.hpp>
#include <driver_parameter.hpp>

#include <math.h>
#include <map>

namespace
{
  typedef unsigned int imu_type_t; ///< Type of IMU used
  typedef int imu_rate_t; ///< IMU message rate

  const imu_type_t IMU_TYPE_UNKNOWN = 0;
}



namespace novatel_oem7_driver
{
  /***
   * Converts degrees to Radians
   *
   * @return radians
   */
  inline double degreesToRadians(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  const double DATA_NOT_AVAILABLE = -1.0; ///< Used to initialized unpopulated fields.

  class INSHandler: public Oem7MessageHandlerIf
  {
    rclcpp::Node* node_;

    std::unique_ptr<Oem7RosPublisher<sensor_msgs::msg::Imu>>                   imu_pub_;
    std::unique_ptr<Oem7RosPublisher<sensor_msgs::msg::Imu>>                   raw_imu_pub_;
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::CORRIMU>>         corrimu_pub_;
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::INSSTDEV>>        insstdev_pub_;
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::INSPVAX>>         inspvax_pub_;
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::INSCONFIG>>       insconfig_pub_;


    std::shared_ptr<novatel_oem7_msgs::msg::INSPVA>   inspva_;
    std::shared_ptr<novatel_oem7_msgs::msg::CORRIMU>  corrimu_;
    std::shared_ptr<novatel_oem7_msgs::msg::INSSTDEV> insstdev_;

    int imu_rate_ = 0;
    double imu_linear_scale_ = 1.0;
    double imu_angular_scale_ = 1.0;
    std::vector<std::vector<double>> static_meas_{};
    double bias_raw_imu_rot_[3] = {0.0, 0.0, 0.0};
    double bias_raw_imu_acc_[3] = {0.0, 0.0, 0.0};
    bool init_raw_calibration_ = DriverParameter<bool>("ins_raw_static_init_calib", false, *node_).value();

    std::string frame_id_;

    typedef std::map<std::string, std::string> imu_config_map_t;
    imu_config_map_t imu_config_map;

    std::unique_ptr<DriverParameter<std::string>> imu_rate_p_;
    std::unique_ptr<DriverParameter<std::string>> imu_desc_p_;

    int getImuRate(imu_type_t imu_type)
    {
      static DriverParameter<int> rate_p("supported_imus." + std::to_string(imu_type) + ".rate", 0, *node_);
      return rate_p.value();
    }

    double getImuLinearScale(imu_type_t imu_type)
    {
      static DriverParameter<double> scale_p("supported_imus." + std::to_string(imu_type) + ".linear_scale", 1.0, *node_);
      return scale_p.value();
    }

    double getImuAngularScale(imu_type_t imu_type)
    {
      static DriverParameter<double> scale_p("supported_imus." + std::to_string(imu_type) + ".angular_scale", 1.0, *node_);
      return scale_p.value();
    }

    void getImuDescription(imu_type_t imu_type, std::string& desc)
    {
      static DriverParameter<std::string> desc_p("supported_imus." + std::to_string(imu_type) + ".name", "", *node_);
      desc = desc_p.value();
    }

    void do_init_raw_calibration(const RAWIMUSXMem& raw)
    {
      const unsigned int calib_len = 200;
      if (static_meas_.size() < calib_len){
        static_meas_.push_back(std::vector<double>{
          static_cast<double>(raw.x_acc),
          static_cast<double>(raw.y_acc),
          static_cast<double>(raw.z_acc),
          static_cast<double>(raw.x_gyro),
          static_cast<double>(raw.y_gyro),
          static_cast<double>(raw.z_gyro),
        });
      } else {
        std::vector<double> avg = std::reduce(static_meas_.begin(), static_meas_.end(),
            std::vector<double>{0.,0.,0.,0.,0.,0.},
            // define the vector add function
            [](std::vector<double> &x, std::vector<double> &y){
              std::vector<double> result;
              result.reserve(x.size());
              std::transform(x.begin(), x.end(), y.begin(), 
                   std::back_inserter(result), std::plus<double>());
              return result;
            }
        );
        std::for_each(avg.begin(), avg.end(), [](double &v){ v /= static_cast<double>(calib_len); });
        bias_raw_imu_acc_[0] = avg[0];
        bias_raw_imu_acc_[1] = avg[1];
        bias_raw_imu_acc_[2] = avg[2];
        bias_raw_imu_rot_[0] = avg[3];
        bias_raw_imu_rot_[1] = avg[4];
        bias_raw_imu_rot_[5] = avg[5];
        init_raw_calibration_ = false;
      }
    }


    void processInsConfigMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      std::shared_ptr<novatel_oem7_msgs::msg::INSCONFIG> insconfig;
      MakeROSMessage(msg, insconfig);
      insconfig_pub_->publish(insconfig);

      if(imu_rate_ == 0)
      {
        std::string imu_desc;
        getImuDescription(insconfig->imu_type, imu_desc);

        imu_rate_ = getImuRate(insconfig->imu_type);
        imu_linear_scale_ = getImuLinearScale(insconfig->imu_type);
        imu_angular_scale_ = getImuAngularScale(insconfig->imu_type);

        RCLCPP_INFO_STREAM(node_->get_logger(),  "IMU[" << insconfig->imu_type << "]: '" << imu_desc << "', rate= " << imu_rate_);
      }
    }

    void publishInsPVAXMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      std::shared_ptr<novatel_oem7_msgs::msg::INSPVAX> inspvax;
      MakeROSMessage(msg, inspvax);

      inspvax_pub_->publish(inspvax);
    }

    void publishCorrImuMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, corrimu_);
      corrimu_pub_->publish(corrimu_);
    }


    void publishImuMsg()
    {
      if(!imu_pub_->isEnabled())
      {
        return;
      }

      std::shared_ptr<sensor_msgs::msg::Imu> imu(new sensor_msgs::msg::Imu);

      if(inspva_)
      {
        // Azimuth: Oem7 (North=0) to ROS (East=0), using Oem7 LH rule
        static const double ZERO_DEGREES_AZIMUTH_OFFSET = 90.0;
        double azimuth = inspva_->azimuth - ZERO_DEGREES_AZIMUTH_OFFSET;

        static const double AZIMUTH_ROLLOVER = 360 - ZERO_DEGREES_AZIMUTH_OFFSET;
        if(azimuth < -AZIMUTH_ROLLOVER)
        {
          azimuth += AZIMUTH_ROLLOVER;
        }

        tf2::Quaternion tf_orientation;
        tf_orientation.setRPY(
                          degreesToRadians(inspva_->roll),
                         -degreesToRadians(inspva_->pitch),
                         -degreesToRadians(azimuth)); // Oem7 LH to ROS RH rule

        imu->orientation = tf2::toMsg(tf_orientation);
      }
      else
      {
        return;
      }

      if(corrimu_ && corrimu_->imu_data_count > 0 && imu_rate_ > 0)
      {
        double instantaneous_rate_factor = imu_rate_ / corrimu_->imu_data_count;

        imu->angular_velocity.x =  corrimu_->roll_rate  * instantaneous_rate_factor;
        imu->angular_velocity.y = -corrimu_->pitch_rate * instantaneous_rate_factor;
        imu->angular_velocity.z =  corrimu_->yaw_rate   * instantaneous_rate_factor;

        imu->linear_acceleration.x =  corrimu_->longitudinal_acc * instantaneous_rate_factor;
        imu->linear_acceleration.y = -corrimu_->lateral_acc      * instantaneous_rate_factor;
        imu->linear_acceleration.z =  corrimu_->vertical_acc     * instantaneous_rate_factor;
      }

      if(insstdev_)
        {
        imu->orientation_covariance[0] = std::pow(insstdev_->pitch_stdev,   2);
        imu->orientation_covariance[4] = std::pow(insstdev_->roll_stdev,    2);
        imu->orientation_covariance[8] = std::pow(insstdev_->azimuth_stdev, 2);
      }

      imu->angular_velocity_covariance[0]    = DATA_NOT_AVAILABLE;
      imu->linear_acceleration_covariance[0] = DATA_NOT_AVAILABLE;

      imu_pub_->publish(imu);
    }

    void publishInsStDevMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, insstdev_);
      insstdev_pub_->publish(insstdev_);
    }

    void processRawImuMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      const RAWIMUSXMem* raw;
      
      if (init_raw_calibration_) {
        raw = reinterpret_cast<const RAWIMUSXMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
        do_init_raw_calibration(*raw);
        if (!raw_imu_pub_->isEnabled())
          return;
      } else if (!raw_imu_pub_->isEnabled()) {
        return;
      } else {
        raw = reinterpret_cast<const RAWIMUSXMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
      }
        
      std::shared_ptr<sensor_msgs::msg::Imu> imu = std::make_shared<sensor_msgs::msg::Imu>();
      imu->angular_velocity.x = (raw->x_gyro - bias_raw_imu_rot_[0]) / imu_angular_scale_;
      imu->angular_velocity.y = (raw->y_gyro - bias_raw_imu_rot_[1]) / imu_angular_scale_;
      imu->angular_velocity.z = (raw->z_gyro - bias_raw_imu_rot_[2]) / imu_angular_scale_;

      imu->linear_acceleration.x = raw->x_acc / imu_linear_scale_;
      imu->linear_acceleration.y = raw->y_acc / imu_linear_scale_;
      imu->linear_acceleration.z = raw->z_acc / imu_linear_scale_;

      imu->angular_velocity_covariance[0]    = DATA_NOT_AVAILABLE;
      imu->linear_acceleration_covariance[0] = DATA_NOT_AVAILABLE;

      raw_imu_pub_->publish(imu);
    }


  public:
    INSHandler():
      imu_rate_(0)
    {
    }

    ~INSHandler()
    {
    }

    void initialize(rclcpp::Node& node)
    {
      node_ = &node;

      imu_pub_       = std::make_unique<Oem7RosPublisher<sensor_msgs::msg::Imu>>(            "IMU",       node);
      raw_imu_pub_   = std::make_unique<Oem7RosPublisher<sensor_msgs::msg::Imu>>(            "RAWIMU",    node);
      corrimu_pub_   = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::CORRIMU>>(  "CORRIMU",   node);
      insstdev_pub_  = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::INSSTDEV>>( "INSSTDEV",  node);
      inspvax_pub_   = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::INSPVAX>>(  "INSPVAX",   node);
      insconfig_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::INSCONFIG>>("INSCONFIG", node);

      DriverParameter<int> imu_rate_p("imu_rate", 0, *node_);
      imu_rate_ = imu_rate_p.value();
      if(imu_rate_ > 0)
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "INS: IMU rate overriden to " << imu_rate_);
      }
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS(
                                      {
                                        RAWIMUSX_OEM7_MSGID,
                                        CORRIMUS_OEM7_MSGID,
                                        IMURATECORRIMUS_OEM7_MSGID,
                                        INSPVAS_OEM7_MSGID,
                                        INSPVAX_OEM7_MSGID,
                                        INSSTDEV_OEM7_MSGID,
                                        INSCONFIG_OEM7_MSGID
                                      }
                                    );
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      switch(msg->getMessageId()){
        case INSPVAS_OEM7_MSGID:
          MakeROSMessage(msg, inspva_); // Cache
          break;
        case INSSTDEV_OEM7_MSGID:
          publishInsStDevMsg(msg);
          break;
        case CORRIMUS_OEM7_MSGID:
        case IMURATECORRIMUS_OEM7_MSGID:
          publishCorrImuMsg(msg);
          publishImuMsg();
          break;
        case INSCONFIG_OEM7_MSGID:
          processInsConfigMsg(msg);
          break;
        case INSPVAX_OEM7_MSGID:
          publishInsPVAXMsg(msg);
          break;
        case RAWIMUSX_OEM7_MSGID:
          processRawImuMsg(msg);
          break;
        default:
          assert(false);
          break;
      }
    }
  };

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::INSHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
