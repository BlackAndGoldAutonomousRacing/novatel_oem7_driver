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

/*! \file
 * Binary format definitions for Oem7 messages.
 * In the future, this file will be autogenerated.
 * All Oem7 messages are 4-byte aligned, allowing simple casting into structs.
 * Consult Oem7 manual for details.
 */

#ifndef __OEM7_MESSAGES_H_
#define __OEM7_MESSAGES_H_

#include <stdint.h>


#define ASSERT_MSG "Consult Oem7 manual"

namespace novatel_oem7_driver
{
  typedef uint32_t oem7_enum_t;
  typedef uint32_t oem7_bool_t;
  typedef uint8_t  oem7_hex_t;
  typedef char     oem7_char_t;

  static_assert(sizeof(oem7_char_t) == 1, ASSERT_MSG);
  static_assert(sizeof(double)      == 8, ASSERT_MSG);
  static_assert(sizeof(float)       == 4, ASSERT_MSG);

  struct __attribute__((packed))
  Oem7MessageCommonHeaderMem
  {
    char sync1;
    char sync2;
    char sync3;

    uint8_t    message_length;
    uint16_t   message_id;
  };

  struct __attribute__((packed))
  Oem7MessageHeaderMem
  {
    char sync1;
    char sync2;
    char sync3;

    uint8_t     header_length;
    uint16_t    message_id;
    char        message_type;
    uint8_t     port_address;
    uint16_t    message_length;
    uint16_t    sequence;
    uint8_t     idle_time;
    uint8_t     time_status;
    uint16_t    gps_week;
    int32_t     gps_milliseconds;
    uint32_t    receiver_status;
    uint16_t    reserved;
    uint16_t    receiver_version;
  };

  struct __attribute__((packed))
  Oem7MessgeShortHeaderMem
  {
    char sync1;
    char sync2;
    char sync3;

    uint8_t    message_length;
    uint16_t   message_id;
    uint16_t   gps_week;
    int32_t    gps_milliseconds;
  };


  struct __attribute__((packed))
  BESTPOSMem
  {
    oem7_enum_t        sol_stat;
    oem7_enum_t        pos_type;
    double             lat;
    double             lon;
    double             hgt;
    float              undulation;
    oem7_enum_t        datum_id;
    float              lat_stdev;
    float              lon_stdev;
    float              hgt_stdev;
    oem7_char_t        stn_id[4];
    float              diff_age;
    float              sol_age;
    uint8_t            num_svs;
    uint8_t            num_sol_svs;
    uint8_t            num_sol_l1_svs;
    uint8_t            num_sol_multi_svs;
    oem7_hex_t         reserved;
    oem7_hex_t         ext_sol_stat;
    uint8_t            galileo_beidou_sig_mask;
    uint8_t            gps_glonass_sig_mask;
   };
   static_assert(sizeof(BESTPOSMem) == 72, ASSERT_MSG);


  struct __attribute__((packed))
  BESTVELMem
  {
    uint32_t           sol_stat;
    uint32_t           vel_type;
    float              latency;
    float              diff_age;
    double             hor_speed;
    double             track_gnd;
    double             ver_speed;
    float              reserved;
   };
  static_assert(sizeof(BESTVELMem) == 44, ASSERT_MSG);


  struct __attribute__((packed))
  BESTGNSSPOSMem
  {
    oem7_enum_t        sol_stat;
    oem7_enum_t        pos_type;
    double             lat;
    double             lon;
    double             hgt;
    float              undulation;
    oem7_enum_t        datum_id;
    float              lat_stdev;
    float              lon_stdev;
    float              hgt_stdev;
    oem7_char_t        stn_id[4];
    float              diff_age;
    float              sol_age;
    uint8_t            num_svs;
    uint8_t            num_sol_svs;
    uint8_t            num_sol_l1_svs;
    uint8_t            num_sol_multi_svs;
    oem7_hex_t         reserved;
    oem7_hex_t         ext_sol_stat;
    uint8_t            galileo_beidou_sig_mask;
    uint8_t            gps_glonass_sig_mask;
  };
  static_assert(sizeof(BESTGNSSPOSMem) == 72, ASSERT_MSG);

  struct __attribute__((packed))
  TRACKSTATMem
  {
    oem7_enum_t        sol_stat;
    oem7_enum_t        pos_type;
    float              cutoff;
    uint32_t           num_channels;
  };
  static_assert(sizeof(TRACKSTATMem) == 16, ASSERT_MSG);

  struct __attribute__((packed))
  TrackStatChannelMem
  {
    int16_t           prn;
    int16_t           glofreq;
    uint32_t          ch_tr_status;
    double            psr;
    float             doppler;
    float             c_no;
    float             locktime;
    float             psr_res;
    uint32_t          reject;
    float             psr_weight;
  };
  static_assert(sizeof(TrackStatChannelMem) == 40, ASSERT_MSG);

  struct __attribute__((packed))
  INSPVASmem
  {
    uint32_t gnss_week;
    double   seconds;
    double latitude;
    double longitude;
    double height;
    double north_velocity;
    double east_velocity;
    double up_velocity;
    double roll;
    double pitch;
    double azimuth;
    oem7_enum_t status;
  };
  static_assert(sizeof(INSPVASmem) == 88, ASSERT_MSG);

  struct __attribute__((packed))
  CORRIMUSMem
  {
    uint32_t        imu_data_count;
    double          pitch_rate;
    double          roll_rate;
    double          yaw_rate;
    double          lateral_acc;
    double          longitudinal_acc;
    double          vertical_acc;
    uint32_t        reserved1;
    uint32_t        reserved2;
  };
  static_assert(sizeof(CORRIMUSMem) == 60, ASSERT_MSG);

  struct __attribute__((packed))
  IMURATECORRIMUSMem
  {
    uint32_t        week;
    double          seconds;
    double          pitch_rate;
    double          roll_rate;
    double          yaw_rate;
    double          lateral_acc;
    double          longitudinal_acc;
    double          vertical_acc;
  };
  static_assert(sizeof(IMURATECORRIMUSMem) == 60, ASSERT_MSG);

  struct __attribute__((packed))
  INSSTDEVMem
  {
    float          latitude_stdev;
    float          longitude_stdev;
    float          height_stdev;
    float          north_velocity_stdev;
    float          east_velocity_stdev;
    float          up_velocity_stdev;
    float          roll_stdev;
    float          pitch_stdev;
    float          azimuth_stdev;
    uint32_t       ext_sol_status;
    uint16_t       time_since_last_update;
    uint16_t       reserved1;
    uint32_t       reserved2;
    uint32_t       reserved3;
  };
  static_assert(sizeof(INSSTDEVMem) == 52, ASSERT_MSG);


  struct __attribute__((packed))
  INSCONFIG_FixedMem
  {
    oem7_enum_t        imu_type;
    uint8_t            mapping;
    uint8_t            initial_alignment_velocity;
    uint16_t           heave_window;
    oem7_enum_t        profile;
    oem7_hex_t         enabled_updates[4];
    oem7_enum_t        alignment_mode;
    oem7_enum_t        relative_ins_output_frame;
    oem7_bool_t        relative_ins_output_direction;
    oem7_hex_t         ins_receiver_status[4];
    uint8_t            ins_seed_enabled;
    uint8_t            ins_seed_validation;
    uint16_t           reserved_1;
    uint32_t           reserved_2;
    uint32_t           reserved_3;
    uint32_t           reserved_4;
    uint32_t           reserved_5;
    uint32_t           reserved_6;
    uint32_t           reserved_7;
  };
  static_assert(sizeof(INSCONFIG_FixedMem) == 60, ASSERT_MSG);

  struct __attribute__((packed))
  INSCONFIG_TranslationMem
  {
    uint32_t       translation;
    uint32_t       frame;
    float          x_offset;
    float          y_offset;
    float          z_offset;
    float          x_uncertainty;
    float          y_uncertainty;
    float          z_uncertainty;
    uint32_t       translation_source;
  };


  struct __attribute__((packed))
  INSCONFIG_RotationMem
  {
    uint32_t       rotation;
    uint32_t       frame;
    float          x_rotation;
    float          y_rotation;
    float          z_rotation;
    float          x_rotation_stdev;
    float          y_rotation_stdev;
    float          z_rotation_stdev;
    uint32_t       rotation_source;
  };


  struct __attribute__((packed))
  INSPVAXMem
  {
    oem7_enum_t    ins_status;
    oem7_enum_t    pos_type;
    double         latitude;
    double         longitude;
    double         height;
    float          undulation;
    double         north_velocity;
    double         east_velocity;
    double         up_velocity;
    double         roll;
    double         pitch;
    double         azimuth;
    float          latitude_stdev;
    float          longitude_stdev;
    float          height_stdev;
    float          north_velocity_stdev;
    float          east_velocity_stdev;
    float          up_velocity_stdev;
    float          roll_stdev;
    float          pitch_stdev;
    float          azimuth_stdev;
    uint32_t       extended_status;
    uint16_t       time_since_update;
  };
  static_assert(sizeof(INSPVAXMem) == 126, ASSERT_MSG);

  struct __attribute__((packed))
  HEADING2Mem
  {
    oem7_enum_t        sol_status;
    oem7_enum_t        pos_type;
    float              length;
    float              heading;
    float              pitch;
    float              reserved;
    float              heading_stdev;
    float              pitch_stdev;
    oem7_char_t        rover_stn_id[4];
    oem7_char_t        master_stn_id[4];
    uint8_t            num_sv_tracked;
    uint8_t            num_sv_in_sol;
    uint8_t            num_sv_obs;
    uint8_t            num_sv_multi;
    uint8_t            sol_source;
    uint8_t            ext_sol_status;
    uint8_t            galileo_beidou_sig_mask;
    uint8_t            gps_glonass_sig_mask;
  };
  static_assert(sizeof(HEADING2Mem) == 48, ASSERT_MSG);

  struct __attribute__((packed))
  BESTUTMMem
  {
    oem7_enum_t        sol_stat;
    oem7_enum_t        pos_type;
    uint32_t           lon_zone_number;
    uint32_t           lat_zone_letter;
    double             northing;
    double             easting;
    double             height;
    float              undulation;
    uint32_t           datum_id;
    float              northing_stddev;
    float              easting_stddev;
    float              height_stddev;
    char               stn_id[4];
    float              diff_age;
    float              sol_age;
    uint8_t            num_svs;
    uint8_t            num_sol_svs;
    uint8_t            num_sol_ggl1_svs;
    uint8_t            num_sol_multi_svs;
    uint8_t            reserved;
    uint8_t            ext_sol_stat;
    uint8_t            galileo_beidou_sig_mask;
    uint8_t            gps_glonass_sig_mask;
  };
  static_assert(sizeof(BESTUTMMem) == 80, ASSERT_MSG);

  struct __attribute__((packed))
  RXSTATUSMem
  {
    uint32_t           error;
    uint32_t           num_status_codes;
    uint32_t           rxstat;
    uint32_t           rxstat_pri_mask;
    uint32_t           rxstat_set_mask;
    uint32_t           rxstat_clr_mask;
    uint32_t           aux1_stat;
    uint32_t           aux1_stat_pri;
    uint32_t           aux1_stat_set;
    uint32_t           aux1_stat_clr;
    uint32_t           aux2_stat;
    uint32_t           aux2_stat_pri;
    uint32_t           aux2_stat_set;
    uint32_t           aux2_stat_clr;
    uint32_t           aux3_stat;
    uint32_t           aux3_stat_pri;
    uint32_t           aux3_stat_set;
    uint32_t           aux3_stat_clr;
    uint32_t           aux4_stat;
    uint32_t           aux4_stat_pri;
    uint32_t           aux4_stat_set;
    uint32_t           aux4_stat_clr;
  };
  static_assert(sizeof(RXSTATUSMem) == 88, ASSERT_MSG);


  struct __attribute__((packed))
  TIMEMem
  {
    uint32_t           clock_status;
    double             offset;
    double             offset_std;
    double             utc_offset;
    uint32_t           utc_year;
    uint8_t            utc_month;
    uint8_t            utc_day;
    uint8_t            utc_hour;
    uint8_t            utc_min;
    uint32_t           utc_msec;
    uint32_t           utc_status;
  };
  static_assert(sizeof(TIMEMem) == 44, ASSERT_MSG);


  struct __attribute__((packed))
  PSRDOP2_FixedMem
  {
    float      gdop;
    float      pdop;
    float      hdop;
    float      vdop;
  };
  static_assert(sizeof(PSRDOP2_FixedMem) == 16, ASSERT_MSG);

  struct __attribute__((packed))
  PSRDOP2_SystemMem
  {
    uint32_t system;
    float    tdop;
  };

  // since RAWIMUS and RAWIMUSX have the same size,
  // we should by default using the eXtended version.
  struct __attribute__((packed))
  RAWIMUSMem
  {
    uint32_t       gnss_week;
    double         gnss_week_seconds;
    oem7_hex_t     imu_status[4];
    int32_t        z_accel;
    int32_t        y_accel;
    int32_t        x_accel;
    int32_t        z_gyro;
    int32_t        y_gyro;
    int32_t        x_gyro;
  };
  static_assert(sizeof(RAWIMUSMem) == 40, ASSERT_MSG);

  struct __attribute__((packed))
  RAWIMUSXMem
  {
    oem7_hex_t  imu_info;
    uint8_t     imu_type;
    uint16_t    gnss_week;
    double      gnss_week_seconds;
    oem7_hex_t  imu_status[4];
    int32_t     z_acc;
    int32_t     y_acc;
    int32_t     x_acc;
    int32_t     z_gyro;
    int32_t     y_gyro;
    int32_t     x_gyro;
  };
  static_assert(sizeof(RAWIMUSXMem) == 40, ASSERT_MSG);


  struct __attribute__((packed))
  PPPPOSMem
  {
    oem7_enum_t        sol_stat;
    oem7_enum_t        pos_type;
    double             lat;
    double             lon;
    double             hgt;
    float              undulation;
    oem7_enum_t        datum_id;
    float              lat_stdev;
    float              lon_stdev;
    float              hgt_stdev;
    oem7_char_t        stn_id[4];
    float              diff_age;
    float              sol_age;
    uint8_t            num_svs;
    uint8_t            num_sol_svs;
    uint8_t            num_sol_l1_svs;
    uint8_t            num_sol_multi_svs;
    oem7_hex_t         reserved;
    oem7_hex_t         ext_sol_stat;
    uint8_t            reserved2;
    uint8_t            gps_glonass_sig_mask;
  };
  static_assert(sizeof(PPPPOSMem) == 72, ASSERT_MSG);


  struct __attribute__((packed))
  TERRASTARINFOMem
  {
    oem7_char_t        product_activation_code[16];
    oem7_enum_t        sub_type;
    uint32_t           sub_permission;
    uint32_t           service_end_day_of_year;
    uint32_t           service_end_year;
    uint32_t           reserved;
    oem7_enum_t        region_restriction;
    float              center_point_latitude;
    float              center_point_longitude;
    uint32_t           radius;
  };
  static_assert(sizeof(TERRASTARINFOMem) == 52, ASSERT_MSG);


  struct __attribute__((packed))
  TERRASTARSTATUSMem
  {
    oem7_enum_t        access_status;
    oem7_enum_t        sync_state;
    uint32_t           reserved;
    oem7_enum_t        local_area_status;
    oem7_enum_t        geo_status;
  };
  static_assert(sizeof(TERRASTARSTATUSMem) == 20, ASSERT_MSG);


  const std::size_t OEM7_BINARY_MSG_HDR_LEN       = sizeof(Oem7MessageHeaderMem);
  const std::size_t OEM7_BINARY_MSG_SHORT_HDR_LEN = sizeof(Oem7MessgeShortHeaderMem);



}
#endif
