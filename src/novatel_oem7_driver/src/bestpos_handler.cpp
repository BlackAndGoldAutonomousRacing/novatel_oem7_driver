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
#include <oem7_driver_util.hpp>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>
#include <oem7_ros_publisher.hpp>
#include <driver_parameter.hpp>

#include "novatel_oem7_msgs/msg/solution_status.hpp"
#include "novatel_oem7_msgs/msg/position_or_velocity_type.hpp"
#include "novatel_oem7_msgs/msg/ppppos.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestutm.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "novatel_oem7_msgs/msg/trackstat.hpp"

#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "GeographicLib/UTMUPS.hpp"

#include <cmath>
#include <stdint.h>


using gps_msgs::msg::GPSFix;
using gps_msgs::msg::GPSStatus;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

using novatel_oem7_msgs::msg::PPPPOS;
using novatel_oem7_msgs::msg::BESTPOS;
using novatel_oem7_msgs::msg::BESTVEL;
using novatel_oem7_msgs::msg::BESTUTM;
using novatel_oem7_msgs::msg::BESTGNSSPOS;
using novatel_oem7_msgs::msg::INSPVA;
using novatel_oem7_msgs::msg::INSPVAX;
using novatel_oem7_msgs::msg::TRACKSTAT;


namespace novatel_oem7_driver
{
  /***
   * Converts radians to degrees
   *
   * @return degrees
   */
  inline double radiansToDegrees(double radians)
  {
    return radians * 180.0 / M_PIf64;
  }

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
   * Compute a single 3D standard deviation from individual deviations.
   * This can be used as a '3D error'.
   */
  inline double Get3DPositionError(double lat_stdev, double lon_stdev, double hgt_stdev)
  {
    // Sum stdevs:take square root of sum of variances
    return std::sqrt(
        std::pow(lat_stdev, 2) +
        std::pow(lon_stdev, 2) +
        std::pow(hgt_stdev, 2)
    );
  }


  /*
   * Refer to NovAtel APN029
   */
  inline double computeHorizontalError(double lat_stdev, double lon_stdev)
  {
    //95%:  2 * DRMS:

    return 2.0 * std::sqrt(
        std::pow(lat_stdev, 2) +
        std::pow(lon_stdev, 2));
  }

  /*
   * Refer to NovAtel APN029
   */
  inline double computeVerticalError(double hgt_stdev)
  {
    //95%
    return 2.0 * hgt_stdev;
  }

  /*
   * Refer to NovAtel APN029
   */
  inline double computeSphericalError(double lat_stdev, double lon_stdev, double hgt_stdev)
  {
    // 90% spherical accuracy
    return 0.833 * (lat_stdev + lon_stdev + hgt_stdev);
  };


  /***
   * Convert GPS time to seconds
   *
   * @return seconds
   */
  inline double MakeGpsTime_Seconds(const uint16_t& gps_week, const uint32_t& gps_milliseconds)
  {
    static const double SECONDS_IN_GPS_WEEK = 604800.0;
    static const double MILLISECONDS_IN_SECOND = 1000.0;

    return gps_week * SECONDS_IN_GPS_WEEK +
           gps_milliseconds / MILLISECONDS_IN_SECOND;
  }

  /***
   * Convert GPS time (double) to UTC
   */
  inline double MakeGpsTime_UTC(const double& gps_time_seconds) {
    // https://novatel.com/support/knowledge-and-learning/unit-conversions
    // seconds between january 1, 1970 and january 6, 1980
    const double epoch_to_gnss = 3657.*24.*60.*60.;
    // subtract 18 seconds to convert to UTC
    return epoch_to_gnss + gps_time_seconds - 18.0;
  }


  /*
   * Relation: greater than, less than, equal
   */
  enum ValueRelation
  {
    REL_GT,
    REL_LT,
    REL_EQ
  };

  /*
   * Determine the time relation between two message headers
   * @return REL_GT when msg_hdr_1 was generated by Oem7 later than msg_hdr_2
   */
  inline ValueRelation
  GetOem7MessageTimeRelation(
      const novatel_oem7_msgs::msg::Oem7Header& msg_hdr_1,
      const novatel_oem7_msgs::msg::Oem7Header& msg_hdr_2)
  {
    if(msg_hdr_1.gps_week_number > msg_hdr_2.gps_week_number)
      return REL_GT;

    if(msg_hdr_1.gps_week_number == msg_hdr_2.gps_week_number)
    {
      if(msg_hdr_1.gps_week_milliseconds > msg_hdr_2.gps_week_milliseconds)
        return REL_GT;

      if(msg_hdr_1.gps_week_milliseconds == msg_hdr_1.gps_week_milliseconds)
        return REL_EQ;
    }

    return REL_LT;
  }


  /**
   * Derive sensor_msgs::Navstatus::service from BESTPOS signal masks
   */
  inline uint16_t
  NavSatStatusService(const BESTPOS::SharedPtr& bestpos)
  {
    uint16_t service = 0;

    if(bestpos->gps_glonass_sig_mask & 0x07)
    {
      service |= NavSatStatus::SERVICE_GPS;
    }

    if(bestpos->gps_glonass_sig_mask & 0x70)
    {
      service |= NavSatStatus::SERVICE_GLONASS;
    }

    if(bestpos->galileo_beidou_sig_mask & 0x0F)
    {
      service |= NavSatStatus::SERVICE_GALILEO;
    }

    if(bestpos->galileo_beidou_sig_mask & 0x70)
    {
      service |= NavSatStatus::SERVICE_COMPASS;
    }

    return service;
  }


  /***
   * Handler of position-related messages. Synthesizes ROS messages GPSFix and NavSatFix from native Oem7 Messages.
   */
  class BESTPOSHandler: public Oem7MessageHandlerIf
  {
    rclcpp::Node* node_;

    std::unique_ptr<Oem7RosPublisher<PPPPOS>>       PPPPOS_pub_;
    std::unique_ptr<Oem7RosPublisher<BESTPOS>>      BESTPOS_pub_;
    std::unique_ptr<Oem7RosPublisher<BESTVEL>>      BESTVEL_pub_;
    std::unique_ptr<Oem7RosPublisher<BESTUTM>>      BESTUTM_pub_;
    std::unique_ptr<Oem7RosPublisher<BESTGNSSPOS>>  BESTGNSSPOS_pub_;
    std::unique_ptr<Oem7RosPublisher<TRACKSTAT>>    TRACKSTAT_pub_;

    std::unique_ptr<Oem7RosPublisher<GPSFix>>       GPSFix_pub_;
    std::unique_ptr<Oem7RosPublisher<NavSatFix>>    NavSatFix_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_offset_pub_;

    rclcpp::Subscription<INSPVA>::SharedPtr   inspva_sub_;
    rclcpp::Subscription<INSPVAX>::SharedPtr  inspvax_sub_;

    std::shared_ptr<BESTPOS> bestpos_ = std::make_shared<BESTPOS>();
    std::shared_ptr<BESTVEL> bestvel_ = std::make_shared<BESTVEL>();
    std::shared_ptr<GPSFix>  gpsfix_ = std::make_shared<GPSFix>();

    Oem7RawMessageIf::ConstPtr psrdop2_;
    INSPVA::SharedPtr inspva_;
    INSPVAX::SharedPtr inspvax_;

    int64_t last_bestpos_;
    int64_t last_bestvel_;
    // int64_t last_bestgnsspos_;
    int64_t last_inspva_;

    int32_t bestpos_period_;
    int32_t bestvel_period_;
    // int32_t bestgnsspos_period_;
    int32_t inspva_period_;

    bool position_source_BESTPOS_; //< User override: always use BESTPOS
    bool position_source_INS_; ///< User override: always use INS

    /***
     * @return true if the specified period is the shortest in all messages.
     */
    bool isShortestPeriod(const int32_t& period) const
    {
      return period <= bestpos_period_ &&
             period <= bestvel_period_ &&
             //period <= bestgnsspos_period_ &&
             period <= inspva_period_;
    }

    /***
     * Updates message period
     */
    template<typename T>
    void updatePeriod(
        const T& msg,
        int64_t& last_msg_msec,
        int32_t& msg_period)
    {
      int64_t cur_msg_msec = GPSTimeToMsec(msg->nov_header);
      if(last_msg_msec > 0)
      {
        int32_t period = cur_msg_msec - last_msg_msec;
        if(period >= 0)
        {
          msg_period = period;
        }
        else // Could be input corruption; do not update anything.
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
                             "updatePeriod: msg= " << msg->nov_header.message_id << "; per= " << period << "; ignored.");
        }
      }

      last_msg_msec = cur_msg_msec;
    }

    void publishBESTPOS(const Oem7RawMessageIf::ConstPtr& msg)
    {
      MakeROSMessage(msg, *bestpos_);
      updatePeriod(bestpos_, last_bestpos_, bestpos_period_);

      BESTPOS_pub_->publish(bestpos_);
    }

    void publishBESTVEL(const Oem7RawMessageIf::ConstPtr& msg)
    {
      MakeROSMessage(msg, *bestvel_);
      updatePeriod(bestvel_, last_bestvel_, bestvel_period_);
      BESTVEL_pub_->publish(bestvel_);
    }

    void publishBESTUTM(const Oem7RawMessageIf::ConstPtr& msg)
    {
      auto bestutm = std::make_unique<BESTUTM>();
      MakeROSMessage(msg, *bestutm);
      BESTUTM_pub_->publish(std::move(bestutm));
    }

    void publishBESTGNSSPOS(const Oem7RawMessageIf::ConstPtr& msg)
    {
      auto bestgnsspos = std::make_unique<BESTGNSSPOS>();
      MakeROSMessage(msg, *bestgnsspos);
      BESTGNSSPOS_pub_->publish(std::move(bestgnsspos));
    }

    void publishTRACKSTAT(const Oem7RawMessageIf::ConstPtr& msg)
    {
      auto trackstat = std::make_unique<TRACKSTAT>();
      MakeROSMessage(msg, *trackstat);
      TRACKSTAT_pub_->publish(std::move(trackstat));
    }

    void publishPPPPOS(const Oem7RawMessageIf::ConstPtr& msg)
    {
      auto ppppos = std::make_unique<PPPPOS>();
      MakeROSMessage(msg, *ppppos);
      PPPPOS_pub_->publish(std::move(ppppos));
    }

    void processPositionAndPublishGPSFix()
    {
      double timestamp_now = this->node_->now().nanoseconds() * 1.e-9;

      gpsfix_->status.position_source     = GPSStatus::SOURCE_NONE;
      gpsfix_->status.orientation_source  = GPSStatus::SOURCE_NONE;
      gpsfix_->status.motion_source       = GPSStatus::SOURCE_NONE;
      gpsfix_->position_covariance_type   = GPSFix::COVARIANCE_TYPE_UNKNOWN;

      // BESTPOS has the highest quality values, use them by default. They may be overriden later.
      // This is deliberately not optimized for clarity.

      // messages have been received at least once
      if(bestpos_->header.stamp.sec)
      {
        gpsfix_->latitude   = bestpos_->lat;
        gpsfix_->longitude  = bestpos_->lon;
        gpsfix_->altitude   = bestpos_->hgt;

        // Convert stdev to diagonal covariance
        gpsfix_->position_covariance[0] = std::pow(bestpos_->lon_stdev, 2);
        gpsfix_->position_covariance[4] = std::pow(bestpos_->lat_stdev, 2);
        gpsfix_->position_covariance[8] = std::pow(bestpos_->hgt_stdev, 2);
        gpsfix_->position_covariance_type = GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gpsfix_->err_horz = computeHorizontalError(bestpos_->lon_stdev, bestpos_->lat_stdev);
        gpsfix_->err_vert = computeVerticalError(  bestpos_->hgt_stdev);
        gpsfix_->err      = computeSphericalError( bestpos_->lon_stdev, bestpos_->lat_stdev, bestpos_->hgt_stdev);

        gpsfix_->time = MakeGpsTime_Seconds(
                          bestpos_->nov_header.gps_week_number,
                          bestpos_->nov_header.gps_week_milliseconds);


        gpsfix_->status.satellites_visible = bestpos_->num_svs;
        gpsfix_->status.satellites_used    = bestpos_->num_sol_svs;
        gpsfix_->status.status             = ToROSGPSStatus(bestpos_);

        gpsfix_->status.position_source = GPSStatus::SOURCE_GPS;
      }

      if(bestvel_->header.stamp.sec)
      {
        // FIXME: this does not get through.
        gpsfix_->track = bestvel_->trk_gnd;
        gpsfix_->speed = bestvel_->hor_speed;
        gpsfix_->climb = bestvel_->ver_speed;

        if(gpsfix_->time == 0.0) // Not populated yet
        {
          gpsfix_->time = MakeGpsTime_Seconds(
                            bestvel_->nov_header.gps_week_number,
                            bestvel_->nov_header.gps_week_milliseconds);
        }

        if(bestvel_->vel_type.type == novatel_oem7_msgs::msg::PositionOrVelocityType::DOPPLER_VELOCITY)
        {
          gpsfix_->status.motion_source = GPSStatus::SOURCE_DOPPLER;
        }
        else
        {
          gpsfix_->status.motion_source = GPSStatus::SOURCE_POINTS;
        }
      }

      if(inspva_)
      {
        // Populate INS data
        gpsfix_->pitch  = -inspva_->pitch;
        gpsfix_->roll   =  inspva_->roll;
        //gpsfix->dip: not populated.

        // BESTPOS/BESTVEL take INS into account
        gpsfix_->status.orientation_source = (GPSStatus::SOURCE_GYRO | GPSStatus::SOURCE_ACCEL);

        // Use most recent timestamp
        gpsfix_->time = MakeGpsTime_Seconds(
                          inspva_->nov_header.gps_week_number,
                          inspva_->nov_header.gps_week_milliseconds);

        // For normal installations, INSPVA messages are sent at much higher rate than BESTPOS/BESTVEL.
        // More recent INSPVAS are preferred, unless they report inferior accuracy.
        // This takes effect, unless explicitly overriden:
        assert(position_source_BESTPOS_ != position_source_INS_ || !position_source_BESTPOS_); // Can't both be true, both can be false.
        bool prefer_INS = position_source_INS_; // Init to override value
        if(!position_source_INS_ && !position_source_BESTPOS_) // Not overriden: determine source on-the-fly based on quality
        {
          if(bestpos_->header.stamp.sec && inspvax_)
          {
            ValueRelation time_rel = GetOem7MessageTimeRelation(inspva_->nov_header, bestpos_->nov_header);
            if(time_rel == REL_GT || time_rel == REL_EQ)
            {
              const double ins_3d_pos_err = Get3DPositionError(
                                              inspvax_->latitude_stdev,
                                              inspvax_->longitude_stdev,
                                              inspvax_->height_stdev);
              if(ins_3d_pos_err != 0.0) // Never exactly 0.0 for a valid position
              {
                static const float ACCURACY_MARGIN_FACTOR = 1.1; // Avoid shifting rapidly between data sources.
                prefer_INS = ins_3d_pos_err < Get3DPositionError(
                                                bestpos_->lat_stdev,
                                                bestpos_->lon_stdev,
                                                bestpos_->hgt_stdev) * ACCURACY_MARGIN_FACTOR;
              }
            }
          }
        }
        //-------------------------------------------------------------------------------------------------------
        // Log INS vs BESTPOS preference
        // This logic is not necessary for correct operation.
        static bool prev_prefer_INS = false;
        static bool initial_pref = true;
        if(prev_prefer_INS != prefer_INS || initial_pref)
        {
          initial_pref = false;

          RCLCPP_INFO_STREAM(node_->get_logger(), "GPSFix position source= INSPVA: " << prev_prefer_INS
                                                               << " --> " << prefer_INS
                                                               << " at GPSTime["
                                                               << inspva_->nov_header.gps_week_number         << " "
                                                               << inspva_->nov_header.gps_week_milliseconds   << "]"
                                                               );
        }
        prev_prefer_INS = prefer_INS;
        //--------------------------------------------------------------------------------------------------------

        if(!bestpos_->header.stamp.sec || prefer_INS)
        {
          gpsfix_->latitude   = inspva_->latitude;
          gpsfix_->longitude  = inspva_->longitude;
          
          // GPSFix needs MSL height; SPAN reports ellipsoidal; so it's computed.
          if(bestpos_->header.stamp.sec) {
            gpsfix_->altitude = inspva_->height - bestpos_->undulation;
          } else if (inspvax_) {
            gpsfix_->altitude = inspva_->height - inspvax_->undulation;
          } else {
            // Abnormal condition; likely receiver misconfiguration.
            RCLCPP_ERROR_STREAM(node_->get_logger(), "No BESTPOS or INSPVAX to get undulation");
            *gpsfix_ = GPSFix();
            return;
          }

          gpsfix_->status.position_source |= (GPSStatus::SOURCE_GYRO | GPSStatus::SOURCE_ACCEL);

          if(inspvax_)
          {
            // Convert stdev to diagonal covariance
            gpsfix_->position_covariance[0] = std::pow(inspvax_->longitude_stdev, 2);
            gpsfix_->position_covariance[4] = std::pow(inspvax_->latitude_stdev,  2);
            gpsfix_->position_covariance[8] = std::pow(inspvax_->height_stdev,    2);
            gpsfix_->position_covariance_type = GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
          }
        }

        /*
        // Considering different reference frames of the GNSS vs INS velocities,
        // switching of velocity sources is not recommended.
        // Ref: https://docs.novatel.com/OEM7/Content/PDFs/OEM7_SPAN_Installation_Operation_Manual.pdf, pp 80

        if(!bestvel_->header.stamp.sec || prefer_INS)
        {
          // Compute track and horizontal speed from north and east velocities

          gpsfix_->track = radiansToDegrees(
                              atan2(inspva_->north_velocity, inspva_->east_velocity));
          if(gpsfix_->track < 0.0)
          {
            gpsfix_->track + 360.0;
          }

          gpsfix_->speed = std::sqrt(std::pow(inspva_->north_velocity, 2.0) +
                                    std::pow(inspva_->east_velocity,  2.0));

          gpsfix_->climb = inspva_->up_velocity;
          gpsfix_->status.motion_source = (GPSStatus::SOURCE_GYRO | GPSStatus::SOURCE_ACCEL);
        }
        */

      } // if(inspva_)


      if(psrdop2_)
      {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), ">PSRDOP");
        GetDOPFromPSRDOP2(
            psrdop2_,
            0, // GPS
            gpsfix_->gdop,
            gpsfix_->pdop,
            gpsfix_->hdop,
            gpsfix_->vdop,
            gpsfix_->tdop);
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "<PSRDOP");
      }

      // publish time difference based on gpsfix_->time
      double diff = timestamp_now - MakeGpsTime_UTC(gpsfix_->time);
      if (std::fabs(diff) > 0.1) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *(node_->get_clock()), 500,
           "WARNING: Time difference high! difference is: " << diff);
      }

      auto time_offset_msg = std::make_unique<std_msgs::msg::Float64>();
      time_offset_msg->data = diff;
      time_offset_pub_->publish(std::move(time_offset_msg));

      GPSFix_pub_->publish(gpsfix_);
    }

    void publishNavSatFix()
    {
      if(!gpsfix_->header.stamp.sec || !bestpos_->header.stamp.sec) {
        // BESTPOS is needed for service status and undulation
        return;
      }

      std::unique_ptr<NavSatFix> navsatfix = std::make_unique<NavSatFix>();

      navsatfix->latitude    = gpsfix_->latitude;
      navsatfix->longitude   = gpsfix_->longitude;
      navsatfix->altitude    = gpsfix_->altitude + bestpos_->undulation;

      navsatfix->position_covariance[0]   = gpsfix_->position_covariance[0];
      navsatfix->position_covariance[4]   = gpsfix_->position_covariance[4];
      navsatfix->position_covariance[8]   = gpsfix_->position_covariance[8];
      navsatfix->position_covariance_type = GpsFixCovTypeToNavSatFixCovType(gpsfix_->position_covariance_type);

      navsatfix->status.status  = GpsStatusToNavSatStatus(gpsfix_->status.status);
      navsatfix->status.service = NavSatStatusService(bestpos_);
      NavSatFix_pub_->publish(std::move(navsatfix));
    }


    void publishROSMessages()
    {
      processPositionAndPublishGPSFix(); // Must be published first, since other message may be derived from it.
      publishNavSatFix();
    }


    /***
     * Converts covariance form GPSFix to NavSatFix
     * @return NavSatFix covariance
     */
    uint8_t GpsFixCovTypeToNavSatFixCovType(uint8_t covariance_type) const
    {
      switch(covariance_type)
      {
        case GPSFix::COVARIANCE_TYPE_APPROXIMATED:
          return NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

        case GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
          return NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        case GPSFix::COVARIANCE_TYPE_KNOWN:
          return NavSatFix::COVARIANCE_TYPE_KNOWN;

        case GPSFix::COVARIANCE_TYPE_UNKNOWN:
          return NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        default:
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown GPSFix covariance type: " << covariance_type);
          return NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      }
    }

    /***
     * Derive ROS GPS Status from Oem7 BESTPOS
     *
     * @return ROS status.
     */
    int16_t ToROSGPSStatus(const BESTPOS::SharedPtr& bestpos) const
    {
      // ROS does not support all necessary solution types to map Oem7 solution types correctly.
      // For consistency, OEM7 WAAS is reported as SBAS.


      switch(bestpos->pos_type.type)
      {
        case novatel_oem7_msgs::msg::PositionOrVelocityType::PSRDIFF:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_PSRDIFF:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::L1_FLOAT:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::NARROW_FLOAT:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::L1_INT:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::WIDE_INT:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::NARROW_INT:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::RTK_DIRECT_INS:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_RTKFLOAT:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_RTKFIXED:
          return GPSStatus::STATUS_DGPS_FIX;

        case novatel_oem7_msgs::msg::PositionOrVelocityType::FIXEDPOS:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::FIXEDHEIGHT:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::DOPPLER_VELOCITY:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::SINGLE:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_PSRSP:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::PROPAGATED:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::OPERATIONAL:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::WARNING:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::OUT_OF_BOUNDS:
          return GPSStatus::STATUS_FIX;

        case novatel_oem7_msgs::msg::PositionOrVelocityType::WAAS:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_SBAS:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::PPP_CONVERGING:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::PPP:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_PPP_CONVERGING:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_PPP:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::PPP_BASIC_CONVERGING:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::PPP_BASIC:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_PPP_BASIC_CONVERGING:
        case novatel_oem7_msgs::msg::PositionOrVelocityType::INS_PPP_BASIC:
          return GPSStatus::STATUS_SBAS_FIX;

        case novatel_oem7_msgs::msg::PositionOrVelocityType::NONE:
          return GPSStatus::STATUS_NO_FIX;

        default:
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown OEM7 PositionOrVelocityType: " << bestpos->pos_type.type);
          return GPSStatus::STATUS_NO_FIX;
      };
    }

    /*** Generates NavSatFix object from GpsFix
     */
    void GpsFixToNavSatFix(const GPSFix::SharedPtr& gpsfix, NavSatFix::SharedPtr& navsatfix) const
    {
      navsatfix->latitude    = gpsfix->latitude;
      navsatfix->longitude   = gpsfix->longitude;
      navsatfix->altitude    = gpsfix->altitude;

      navsatfix->position_covariance[0]   = gpsfix->position_covariance[0];
      navsatfix->position_covariance[4]   = gpsfix->position_covariance[4];
      navsatfix->position_covariance[8]   = gpsfix->position_covariance[8];
      navsatfix->position_covariance_type = GpsFixCovTypeToNavSatFixCovType(gpsfix->position_covariance_type);
    }

    /**
     * Generates NavSatStatus from GPSStatus::status
     */
    uint8_t GpsStatusToNavSatStatus(const int16_t gps_status) const
    {
      // Keep this in sync with the return values of ToROSGPSStatus
      switch(gps_status)
      {
        case GPSStatus::STATUS_NO_FIX:
          return NavSatStatus::STATUS_NO_FIX;

        case GPSStatus::STATUS_FIX:
          return NavSatStatus::STATUS_FIX;

        case GPSStatus::STATUS_SBAS_FIX:
        case GPSStatus::STATUS_WAAS_FIX:
          return NavSatStatus::STATUS_SBAS_FIX;

        case GPSStatus::STATUS_DGPS_FIX:
        case GPSStatus::STATUS_GBAS_FIX:
          return NavSatStatus::STATUS_GBAS_FIX;

        default:
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown gps status: " << gps_status);
          return NavSatStatus::STATUS_NO_FIX;
      }
    }

    std::string topic(const std::string& publisher) const
    {
      std::string topic;
      node_->get_parameter(publisher + ".topic", topic);
      return std::string(node_->get_namespace()) + 
                        (node_->get_namespace() == std::string("/") ? topic : "/" + topic);
    }

  public:
    BESTPOSHandler():
      last_bestpos_(0),
      last_bestvel_(0),
      last_inspva_(0),
      bestpos_period_(std::numeric_limits<int32_t>::max()),
      bestvel_period_(std::numeric_limits<int32_t>::max()),
      //bestgnsspos_period_(std::numeric_limits<int32_t>::max()),
      inspva_period_( std::numeric_limits<int32_t>::max()),
      position_source_BESTPOS_(false),
      position_source_INS_(false)
    {
    }

    ~BESTPOSHandler()
    {
    }

    void initialize(rclcpp::Node& node)
    {
      node_ = &node;

      PPPPOS_pub_  = std::make_unique<Oem7RosPublisher<PPPPOS>>(  "PPPPOS",       node);
      BESTPOS_pub_ = std::make_unique<Oem7RosPublisher<BESTPOS>>("BESTPOS",       node);
      BESTVEL_pub_ = std::make_unique<Oem7RosPublisher<BESTVEL>>("BESTVEL",       node);
      BESTUTM_pub_ = std::make_unique<Oem7RosPublisher<BESTUTM>>("BESTUTM",       node);
      BESTGNSSPOS_pub_ = std::make_unique<Oem7RosPublisher<BESTGNSSPOS>>("BESTGNSSPOS", node);
      TRACKSTAT_pub_ = std::make_unique<Oem7RosPublisher<TRACKSTAT>>("TRACKSTAT", node);

      GPSFix_pub_  = std::make_unique<Oem7RosPublisher<GPSFix>>(  "GPSFix",       node);
      NavSatFix_pub_ = std::make_unique<Oem7RosPublisher<NavSatFix>>("NavSatFix", node);

      inspva_sub_ = node.create_subscription<INSPVA>(topic("INSPVA"), 10,
        [&](const INSPVA::SharedPtr msg){
          inspva_ = msg;
          updatePeriod(inspva_, last_inspva_, inspva_period_);
          if(isShortestPeriod(inspva_period_))
          {
            publishROSMessages();
          }
        }
      );
      inspvax_sub_ = node.create_subscription<INSPVAX>(topic("INSPVAX"), 10,
        [&](const INSPVAX::SharedPtr msg){ inspvax_ = msg; }
      );

      time_offset_pub_ = node.create_publisher<std_msgs::msg::Float64>("time/offset", rclcpp::SensorDataQoS());

      DriverParameter<std::string> pos_source_p("oem7_position_source", "",       node);

      // Determine if position source is overriden by the user; otherwise it is determined dynamically.
      std::string position_source = pos_source_p.value();
      if(position_source == "BESTPOS")
      {
        position_source_BESTPOS_ = true;
      }
      else if(position_source == "INSPVAS")
      {
        position_source_INS_ = true;
      }
      else
      {
        position_source = "BESTPOS or INSPVAS based on quality";
      }
      RCLCPP_INFO_STREAM(node_->get_logger(), "GPSFix position source: " << position_source);
    }

    const MessageIdRecords& getMessageIds()
    {
      static const MessageIdRecords MSG_IDS(
                                    {
                                      {INSPVAS_OEM7_MSGID,      MSGFLAG_NONE},
                                      {BESTPOS_OEM7_MSGID,      MSGFLAG_NONE},
                                      {BESTVEL_OEM7_MSGID,      MSGFLAG_NONE},
                                      {BESTUTM_OEM7_MSGID,      MSGFLAG_NONE},
                                      {BESTGNSSPOS_OEM7_MSGID,  MSGFLAG_NONE},
                                      {PPPPOS_OEM7_MSGID,       MSGFLAG_NONE},
                                      {INSPVAX_OEM7_MSGID,      MSGFLAG_NONE},
                                      {PSRDOP2_OEM7_MSGID,      MSGFLAG_NONE},
                                      {TRACKSTAT_OEM7_MSGID,    MSGFLAG_NONE}
                                    });
      return MSG_IDS;
    }

    void handleMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "BESTPOS < [id=" << msg->getMessageId() << "] periods (BP BV BGP PVA):" <<
                        bestpos_period_ << " " <<
                        bestvel_period_ << " " <<
                        //bestgnsspos_period_ << " " <<
                        inspva_period_);

      // It is assumed all the messages are logged at reasonable rates.
      // BESTPOS and BESTVEL are always logged together.
      // On units with IMU, INSPVA would trigger publishing of ROS messages.
      // On non-IMU units, BESTVEL be.

      switch (msg->getMessageId()){
        case BESTPOS_OEM7_MSGID:
          publishBESTPOS(msg);

          if(isShortestPeriod(bestpos_period_))
          {
            publishROSMessages();
          }
          break;
        
        case BESTVEL_OEM7_MSGID:
          publishBESTVEL(msg);

          if(isShortestPeriod(bestvel_period_))
          {
            publishROSMessages();
          }
          break;
        
        case BESTUTM_OEM7_MSGID:
          publishBESTUTM(msg);
          break;
        
        case BESTGNSSPOS_OEM7_MSGID:
          publishBESTGNSSPOS(msg);
          break;

        case TRACKSTAT_OEM7_MSGID:
          publishTRACKSTAT(msg);
          break;

        case PPPPOS_OEM7_MSGID:
          publishPPPPOS(msg);
          break;

        case PSRDOP2_OEM7_MSGID:
          psrdop2_ = msg;
          break;

        default:
          break;
      }
    }
  };

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::BESTPOSHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
