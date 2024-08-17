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
#include <novatel_oem7_driver/oem7_messages.h>
#include <novatel_oem7_driver/oem7_imu.hpp>

#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "novatel_oem7_msgs/msg/corrimu.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/imuratecorrimu.hpp"
#include "novatel_oem7_msgs/msg/inertial_solution_status.hpp"
#include "novatel_oem7_msgs/msg/solution_status.hpp"
#include "novatel_oem7_msgs/msg/insconfig.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "novatel_oem7_msgs/srv/oem7_abascii_cmd.hpp"

#include <oem7_ros_publisher.hpp>
#include <driver_parameter.hpp>

#include <math.h>
#include <map>

namespace
{
  /***
   * Converts degrees to Radians
   *
   * @return radians
   */
  inline double degreesToRadians(double degrees)
  {
    return degrees * M_PIf64 / 180.0;
  }

  /**
   * @brief Rotates a 3x3 covariance matrix
   *
   * @return rotated cov matrix in tf2::Matrix3x3 form.
   */
  inline tf2::Matrix3x3 rotateCovMatrix(const tf2::Transform& tf,
                                                 const std::array<double, 9UL>& covIn)
  {
    tf2::Matrix3x3 covMat(covIn[0], covIn[1], covIn[2],
                            covIn[3], covIn[4], covIn[5],
                            covIn[6], covIn[7], covIn[8]);
    tf2::Quaternion q = tf.getRotation().normalized();
    if (!std::isnan(q.w())){
      return covMat;
    } else {
      tf2::Matrix3x3 tfRot = tf.getBasis();
      covMat = tfRot * covMat * (tfRot.transpose());
    }
    return covMat;
  }

  const double DATA_NOT_AVAILABLE = -1.0; ///< Used to initialized unpopulated fields.

}

namespace novatel_oem7_driver
{
  using namespace novatel_oem7_msgs::msg;

  class INSHandler: public Oem7MessageHandlerIf
  {
    rclcpp::Node* node_;

    std::unique_ptr<Oem7RosPublisher<sensor_msgs::msg::Imu>>  imu_pub_;
    std::unique_ptr<Oem7RosPublisher<sensor_msgs::msg::Imu>>  raw_imu_pub_;
    std::unique_ptr<Oem7RosPublisher<CORRIMU>>                corrimu_pub_;
    std::unique_ptr<Oem7RosPublisher
            <geometry_msgs::msg::TwistWithCovarianceStamped>> ins_vel_pub_;
    std::unique_ptr<Oem7RosPublisher<INSPVA>>                 inspva_pub_;
    std::unique_ptr<Oem7RosPublisher<INSPVAX>>                inspvax_pub_;
    std::unique_ptr<Oem7RosPublisher<INSCONFIG>>              insconfig_pub_;

    rclcpp::Subscription<HEADING2>::SharedPtr                 align_sub_;

    rclcpp::Client<novatel_oem7_msgs::srv::Oem7AbasciiCmd>::SharedPtr oem7CmdCli_;

    std::shared_ptr<HEADING2> align_sol_;
    INSPVAX  inspvax_;

    bool inspva_present_;
    bool ins_sol_good_ = false;
    tf2::Transform enu_to_local_rotation_;
    geometry_msgs::msg::TwistWithCovarianceStamped twist_w_cov_;

    oem7_imu_rate_t imu_rate_;                ///< IMU output rate
    double imu_raw_gyro_scale_factor_;        ///< IMU-specific raw gyroscope scaling
    double imu_raw_accel_scale_factor_;       ///< IMU-specific raw acceleration scaling.
    std::vector<std::vector<double>> static_meas_{};
    double bias_raw_imu_rot_[3] = {0.0, 0.0, 0.0};
    double bias_raw_imu_acc_[3] = {0.0, 0.0, 0.0};
    bool init_raw_calibration_lin_;
    bool init_raw_calibration_ang_;

    std::string frame_id_;

    typedef std::map<std::string, std::string> imu_config_map_t;
    imu_config_map_t imu_config_map;

    std::unique_ptr<DriverParameter<std::string>> imu_rate_p_;
    std::unique_ptr<DriverParameter<std::string>> imu_desc_p_;

    oem7_imu_rate_t getImuRate(oem7_imu_type_t imu_type)
    {
      static DriverParameter<int> rate_p("supported_imus." + std::to_string(imu_type) + ".rate", 0, *node_);
      return rate_p.value();
    }

    void getImuDescription(oem7_imu_type_t imu_type, std::string& desc)
    {
      static DriverParameter<std::string> desc_p("supported_imus." + std::to_string(imu_type) + ".name", "", *node_);
      desc = desc_p.value();
    }

    void doInitRawCalibration(const RAWIMUSXMem& raw)
    {
      // intercept the first N messages for calibration.
      // This formulation allows co-calibration across multiple units running this same piece of driver and the same N,
      // TODO: although the actual implementation is not there yet.
      const unsigned int calib_len = 200;
      // single-unit calibration. The unit should be stationary and upright during this period.
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
        // TODO: maybe we can also calculate variance?
        if (init_raw_calibration_lin_){
          bias_raw_imu_acc_[0] = avg[0];
          bias_raw_imu_acc_[1] = avg[1];
          bias_raw_imu_acc_[2] = avg[2] - ONE_G / imu_raw_accel_scale_factor_;
          init_raw_calibration_lin_ = false;
        }
        if (init_raw_calibration_ang_){
          bias_raw_imu_rot_[0] = avg[3];
          bias_raw_imu_rot_[1] = avg[4];
          bias_raw_imu_rot_[2] = avg[5];
          init_raw_calibration_ang_ = false;
        }
        static_meas_.clear(); // frees up memory.
      }
    }


    void processInsConfigMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
      auto insconfig = std::make_unique<INSCONFIG>();
      MakeROSMessage(msg, *insconfig);

      const oem7_imu_type_t imu_type = static_cast<oem7_imu_type_t>(insconfig->imu_type);

      std::string imu_desc;
      getImuDescription(imu_type, imu_desc);

      if(imu_rate_ == 0) // No override; this is normal.
      {
        imu_rate_ = getImuRate(imu_type);
      }

      if(imu_rate_ == 0) // No rate configured at all.
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                    "IMU type = '" << imu_type  << "': IMU rate unavailable. IMU output disabled.");
        return;
      }

      if(imu_raw_gyro_scale_factor_  == 0.0 ||
         imu_raw_accel_scale_factor_ == 0.0) // No override, this is normal.
      {
        if(!getImuRawScaleFactors(
            imu_type,
            imu_raw_gyro_scale_factor_,
            imu_raw_accel_scale_factor_))
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
            "IMU type= '" << insconfig->imu_type << "'; Scale factors unavilable. Raw IMU output disabled");
          return;
        }
      }

      insconfig_pub_->publish(std::move(insconfig));

      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "IMU: "          << imu_type  << " '"  << imu_desc << "' "
                      << "rate= "         << imu_rate_                      << "' "
                      << "gyro scale= "   << imu_raw_gyro_scale_factor_     << "' "
                      << "accel scale= "  << imu_raw_accel_scale_factor_);
    }

    void publishInsPVAMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
      auto inspva = std::make_unique<INSPVA>();
      MakeROSMessage(msg, *inspva);
      updateEnuOrientation(*inspva);
      inspva_present_ = true;
      publishInsTwistMsg(*inspva);
      inspva_pub_->publish(std::move(inspva));
    }

    void publishInsPVAXMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
      MakeROSMessage(msg, inspvax_);
      inspvax_pub_->publish(inspvax_);
    }

    void publishCorrImuMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
      auto corrimu = std::make_unique<CORRIMU>();
      MakeROSMessage(msg, *corrimu);
      publishImuMsg(*corrimu);
      corrimu_pub_->publish(std::move(corrimu));
    }

    void updateEnuOrientation(const INSPVA& inspva)
    {
      static double pitch = 0., azimuth = 0.;
      // Azimuth: Oem7 (North=0) to ROS (East=0)
      static const double ZERO_DEGREES_AZIMUTH_OFFSET = 90.0;

      if (inspva.status.status == InertialSolutionStatus::INS_SOLUTION_GOOD ||
          inspva.status.status == InertialSolutionStatus::INS_SOLUTION_FREE ||
          inspva.status.status == InertialSolutionStatus::INS_ALIGNMENT_COMPLETE) {
        ins_sol_good_ = true;
        // a converging solution, use INSPVA solution. Need to convert INSPVA frame to ROS frame
        // FIXME: NEED TO DOUBLE-CHECK SETINSROTATION USER 0 0 0 1.0 1.0 1.0
        // (i.e. use Novatel Vehicle frame)
      } else {
        ins_sol_good_ = false;
      }/*else if (align_sol_ && align_sol_->sol_status.status == SolutionStatus::SOL_COMPUTED) {
        // INS initial alignment is incomplete. Substitute orientation with
        // the one obtained from (post-offset) HEADING2 instead.
        pitch = -align_sol_->pitch;                                   // ALIGN has pitch UP positive
        azimuth = ZERO_DEGREES_AZIMUTH_OFFSET - align_sol_->heading;  // ALIGN has North=0, CW positive
        // call initialization command service
        // Wait for the ASCII config interface service to be activated
        if (oem7CmdCli_->wait_for_service(std::chrono::seconds(1))) {
          static unsigned long last_init_ins_from_heading2 = 0;
          unsigned long init_timestamp = node_->now().nanoseconds();
          // TODO: hard-coded: re-init no more frequently than once every two seconds
          if (init_timestamp - last_init_ins_from_heading2 > 2000000000UL) {
            auto request = std::make_unique<novatel_oem7_msgs::srv::Oem7AbasciiCmd::Request>();
            request->cmd = "SETINITAZIMUTH " +
                          std::to_string(align_sol_->heading - ZERO_DEGREES_AZIMUTH_OFFSET) + " " +
                          std::to_string(align_sol_->heading_stdev);
            auto result = oem7CmdCli_->async_send_request(std::move(request));
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS)
              last_init_ins_from_heading2 = init_timestamp;
          }
        }
      }*/

      pitch = inspva.roll;        // INSPVA frame: left/front/down
      azimuth = -inspva.azimuth;  // INSPVA has x: ROS East = INSPVA North = 0, z: CW positive

      // Conversion to quaternion addresses rollover.
      // Pitch and azimuth are adjusted from Y-forward, RH to X-forward, RH.
      tf2::Quaternion orientation;
      orientation.setRPY(degreesToRadians(inspva.pitch),  // roll is INSPVA pitch (INS is Y forward)
                         degreesToRadians(pitch),
                         degreesToRadians(azimuth));      // East=0, CCW positive

      enu_to_local_rotation_.setRotation(orientation);
    }

    void publishInsTwistMsg(const INSPVA& inspva)
    {
      if (!imu_pub_->isEnabled() || !inspva_present_ || imu_rate_ == 0)
        return;

      auto local_to_enu_rotation = enu_to_local_rotation_.inverse();
      // update twist with covariace stamped using inspva_ and inspvax_

      tf2::Vector3 local_linear_velocity = local_to_enu_rotation(tf2::Vector3(
                                                                inspva.east_velocity,
                                                                inspva.north_velocity,
                                                                inspva.up_velocity));
      twist_w_cov_.twist.twist.linear.x = local_linear_velocity.x();
      twist_w_cov_.twist.twist.linear.y = local_linear_velocity.y();
      twist_w_cov_.twist.twist.linear.z = local_linear_velocity.z();
      if(inspvax_.header.stamp.sec)
      {
        double stdev_scalar = 1.0;
        if(!ins_sol_good_) {
          stdev_scalar = 10.0; // arbitrary value to indicate a poor solution
        }
        auto local_linear_vel_cov = rotateCovMatrix(local_to_enu_rotation,
                                            {std::pow(inspvax_.east_velocity_stdev * stdev_scalar, 2), 0., 0.,
                                             0., std::pow(inspvax_.north_velocity_stdev * stdev_scalar, 2), 0.,
                                             0., 0., std::pow(inspvax_.up_velocity_stdev * stdev_scalar, 2)});
        twist_w_cov_.twist.covariance[ 0] = local_linear_vel_cov[0][0];
        //twist_w_cov_.twist.covariance[ 1] = local_linear_vel_cov[0][1];
        //twist_w_cov_.twist.covariance[ 2] = local_linear_vel_cov[0][2];
        //twist_w_cov_.twist.covariance[ 6] = local_linear_vel_cov[1][0];
        twist_w_cov_.twist.covariance[ 7] = local_linear_vel_cov[1][1];
        //twist_w_cov_.twist.covariance[ 8] = local_linear_vel_cov[1][2];
        //twist_w_cov_.twist.covariance[12] = local_linear_vel_cov[2][0];
        //twist_w_cov_.twist.covariance[13] = local_linear_vel_cov[2][1];
        twist_w_cov_.twist.covariance[14] = local_linear_vel_cov[2][2];
      }

      ins_vel_pub_->publish(twist_w_cov_);
    }

    void publishImuMsg(const CORRIMU& corrimu)
    {
      if(!imu_pub_->isEnabled() || !inspva_present_ || imu_rate_ == 0)
        return;

      auto imu = std::make_unique<sensor_msgs::msg::Imu>();
      imu->orientation = tf2::toMsg(enu_to_local_rotation_.getRotation());

      if(corrimu.imu_data_count > 0)
      {
        double instantaneous_rate_factor = imu_rate_ / corrimu.imu_data_count;

        // CORRIMU frame: left/front/up (left-handed). Convert to front/left/up (right-handed)
        imu->angular_velocity.x = corrimu.pitch_rate * instantaneous_rate_factor;
        imu->angular_velocity.y = corrimu.roll_rate  * instantaneous_rate_factor;
        imu->angular_velocity.z = corrimu.yaw_rate   * instantaneous_rate_factor;

        imu->linear_acceleration.x = corrimu.lateral_acc      * instantaneous_rate_factor;
        imu->linear_acceleration.y = corrimu.longitudinal_acc * instantaneous_rate_factor;
        imu->linear_acceleration.z = corrimu.vertical_acc     * instantaneous_rate_factor;

        twist_w_cov_.twist.twist.angular = imu->angular_velocity;
      }

      if(inspvax_.header.stamp.sec)
      {
        double stdev_scalar = 1.0;
        if(!ins_sol_good_) {
          stdev_scalar = 1.5; // arbitrary value to indicate a poor solution
        }
        imu->orientation_covariance[0] = std::pow(inspvax_.pitch_stdev * stdev_scalar,   2);
        imu->orientation_covariance[4] = std::pow(inspvax_.roll_stdev * stdev_scalar,    2);
        imu->orientation_covariance[8] = std::pow(inspvax_.azimuth_stdev * stdev_scalar, 2);
      }

      // TODO hard-coded for now. Should be set in the oem7_imu.cpp as product-specific values.
      const double LINEAR_COV = 0.0009;
      const double ANGULAR_COV = 0.00035;
      // these are 3x3 matrices, so in 0, 4, 8 spots
      // C 0 0
      // 0 C 0
      // 0 0 C
      imu->angular_velocity_covariance[0] = ANGULAR_COV;
      imu->angular_velocity_covariance[4] = ANGULAR_COV;
      imu->angular_velocity_covariance[8] = ANGULAR_COV;
      imu->linear_acceleration_covariance[0] = LINEAR_COV;
      imu->linear_acceleration_covariance[4] = LINEAR_COV;
      imu->linear_acceleration_covariance[8] = LINEAR_COV;

      imu_pub_->publish(std::move(imu));

      twist_w_cov_.twist.covariance[21] = ANGULAR_COV;
      twist_w_cov_.twist.covariance[28] = ANGULAR_COV;
      twist_w_cov_.twist.covariance[35] = ANGULAR_COV;
    }

    /**
     * @return angular velocity, rad / sec
     */
    inline double computeAngularVelocityFromRaw(const double& raw_gyro, const double& bias_iu = 0.0)
    {
      return (raw_gyro - bias_iu) * imu_raw_gyro_scale_factor_;
    }

    /**
     * @return linear acceleration, m / sec^2
     */
    inline double computeLinearAccelerationFromRaw(const double& raw_acc, const double& bias_iu = 0.0)
    {
      return (raw_acc - bias_iu) * imu_raw_accel_scale_factor_;
    }

    void processRawImuMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
      // since RAWIMUS has reduced content without benefits in size,
      // we should always use the eXtended version (RAWIMUSX).
      const RAWIMUSXMem* raw;

      if (init_raw_calibration_lin_ || init_raw_calibration_ang_) {
        raw = reinterpret_cast<const RAWIMUSXMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
        doInitRawCalibration(*raw);
        if (!raw_imu_pub_->isEnabled()          ||
            imu_rate_                   == 0    ||
            imu_raw_gyro_scale_factor_  == 0.0  ||
            imu_raw_accel_scale_factor_ == 0.0)
          return;
        // upon calibration done, the values should see a small jump.
      } else if (!raw_imu_pub_->isEnabled()   ||
          imu_rate_                   == 0    ||
          imu_raw_gyro_scale_factor_  == 0.0  ||
          imu_raw_accel_scale_factor_ == 0.0) {
        return;
      } else {
        raw = reinterpret_cast<const RAWIMUSXMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
      }

      auto imu = std::make_unique<sensor_msgs::msg::Imu>();
      // All measurements are in sensor frame, uncorrected for gravity. There is no up, forward, left;
      // x, y, z are nominal references to enclsoure housing.
      imu->angular_velocity.x =  computeAngularVelocityFromRaw(raw->x_gyro, bias_raw_imu_rot_[0]);
      imu->angular_velocity.y = -computeAngularVelocityFromRaw(raw->y_gyro, bias_raw_imu_rot_[1]); // Refer to RAWIMUSX documentation
      imu->angular_velocity.z =  computeAngularVelocityFromRaw(raw->z_gyro, bias_raw_imu_rot_[2]);

      imu->linear_acceleration.x =  computeLinearAccelerationFromRaw(raw->x_acc, bias_raw_imu_acc_[0]);
      imu->linear_acceleration.y = -computeLinearAccelerationFromRaw(raw->y_acc, bias_raw_imu_acc_[1]);  // Refer to RAWIMUSX documentation
      imu->linear_acceleration.z =  computeLinearAccelerationFromRaw(raw->z_acc, bias_raw_imu_acc_[2]);

      imu->angular_velocity_covariance[0]    = DATA_NOT_AVAILABLE;
      imu->linear_acceleration_covariance[0] = DATA_NOT_AVAILABLE;
      imu->orientation_covariance[0] = DATA_NOT_AVAILABLE;

      raw_imu_pub_->publish(std::move(imu));
    }

    std::string topic(const std::string& publisher)
    {
      std::string topic;
      node_->get_parameter(publisher + ".topic", topic);
      return std::string(node_->get_namespace()) +
                        (node_->get_namespace() == std::string("/") ? topic : "/" + topic);
    }

  public:
    INSHandler():
      inspva_present_(false),
      enu_to_local_rotation_(tf2::Quaternion(0.,0.,0.,1.)),
      imu_rate_(0),
      imu_raw_gyro_scale_factor_ (0.0),
      imu_raw_accel_scale_factor_(0.0)
    {
    }

    ~INSHandler()
    {
    }

    void initialize(rclcpp::Node& node)
    {
      node_ = &node;

      imu_pub_       = std::make_unique<Oem7RosPublisher<sensor_msgs::msg::Imu>>("IMU",       node);
      raw_imu_pub_   = std::make_unique<Oem7RosPublisher<sensor_msgs::msg::Imu>>("RAWIMU",    node);
      corrimu_pub_   = std::make_unique<Oem7RosPublisher<CORRIMU>>(              "CORRIMU",   node);
      ins_vel_pub_   = std::make_unique<Oem7RosPublisher
                              <geometry_msgs::msg::TwistWithCovarianceStamped>>( "INS_Twist", node);
      inspva_pub_    = std::make_unique<Oem7RosPublisher<INSPVA>>(               "INSPVA",    node);
      inspvax_pub_   = std::make_unique<Oem7RosPublisher<INSPVAX>>(              "INSPVAX",   node);
      insconfig_pub_ = std::make_unique<Oem7RosPublisher<INSCONFIG>>(            "INSCONFIG", node);

      DriverParameter<int> imu_rate_p("oem7_imu_rate", 0, *node_);
      imu_rate_ = imu_rate_p.value();
      if(imu_rate_ > 0)
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "INS: IMU rate overriden to " << imu_rate_);
      }

      init_raw_calibration_lin_ = node_->declare_parameter<bool>("ins_raw_static_init_linear_calib", false);
      init_raw_calibration_ang_ = node_->declare_parameter<bool>("ins_raw_static_init_angular_calib", false);

      align_sub_  = node.create_subscription<HEADING2>( topic("HEADING2"),  10,
        [&](const HEADING2::SharedPtr msg){
          align_sol_ = msg;
        }
      );

      static rmw_qos_profile_t qos = rmw_qos_profile_default;
      qos.depth = 20;
      oem7CmdCli_ = node_->create_client<novatel_oem7_msgs::srv::Oem7AbasciiCmd>("Oem7Cmd", qos);
    }

    const MessageIdRecords& getMessageIds()
    {
      static const MessageIdRecords MSG_IDS(
                                      {
                                        {RAWIMUSX_OEM7_MSGID,            MSGFLAG_NONE},
                                        {CORRIMUS_OEM7_MSGID,            MSGFLAG_NONE},
                                        {IMURATECORRIMUS_OEM7_MSGID,     MSGFLAG_NONE},
                                        {INSPVAS_OEM7_MSGID,             MSGFLAG_NONE},
                                        {INSPVAX_OEM7_MSGID,             MSGFLAG_NONE},
                                        {INSPVAS_OEM7_MSGID,             MSGFLAG_NONE},
                                        {INSCONFIG_OEM7_MSGID,           MSGFLAG_STATUS_OR_CONFIG},
                                      }
                                    );
      return MSG_IDS;
    }

    void handleMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
      switch(msg->getMessageId()){
        case INSPVAS_OEM7_MSGID:
          publishInsPVAMsg(msg);
          break;
        case CORRIMUS_OEM7_MSGID:
        case IMURATECORRIMUS_OEM7_MSGID:
          publishCorrImuMsg(msg);
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
