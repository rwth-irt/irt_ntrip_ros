// Copyright 2021 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author:  Thomas Konrad (t.korad@irt.rwth-aachen.de)
//          Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#ifndef NTRIP_ROS_UTILS_H
#define NTRIP_ROS_UTILS_H

#include "config.h"
#include <irt_msgs/msg/rtcml1_e1.hpp>

#pragma once

namespace ntrip
{
    const uint16_t CRC_POLY = 0x8005;
    const uint16_t CRC_START = 0xFFFF;
    const bool     USE_ROSTOCK_SETTINGS = false;
    const bool     CONSOLE_DEBUG = false;
    const unsigned int nmeaResendIntervalSec = 30;

    //Invert a uint8 Array, used by CRC_Generation
    void InvertUint8(uint8_t *dBuf, const uint8_t *srcBuf)
    {
      int i;
      unsigned char tmp[4];
      tmp[0] = 0;
      for(i=0;i< 8;i++)
      {
        if(srcBuf[0]& (1 << i))
          tmp[0]|=1<<(7-i);
      }
      dBuf[0] = tmp[0];
    }

    //Invert a uint16 Array, used by CRC_Generation
    void InvertUint16(uint16_t *dBuf, const uint16_t *srcBuf) {
      int i;
      unsigned short tmp[4];
      tmp[0] = 0;
      for(i=0;i< 16;i++)
      {
        if(srcBuf[0]& (1 << i))
          tmp[0]|=1<<(15 - i);
      }
      dBuf[0] = tmp[0];
    }

    //CRC_Generation in Modbus
    uint16_t CRC16_Modbus_Gen(uint8_t *t, size_t Size)
    {
      uint16_t CRC_In = CRC_START;
      uint8_t  CRC16_Temp = 0;

      while (Size--)
      {
        CRC16_Temp = *(t++);
        InvertUint8(&CRC16_Temp,&CRC16_Temp);
        CRC_In^= (CRC16_Temp << 8);
        for(int i = 0;i < 8;i++)
        {
          if(CRC_In & 0x8000)
            CRC_In = (CRC_In << 1) ^ CRC_POLY;
          else
            CRC_In = CRC_In << 1;
        }
      }
      InvertUint16(&CRC_In,&CRC_In);
      return CRC_In;
    }

    RTCM_3_3_L1_E1 Construction_RTCM3_3_L1E1(const boost::shared_ptr<RTCM_3_3_t>& t)
    {
      RTCM_3_3_L1_E1 d;
      memcpy(&d. TOW, &t-> TOW, sizeof(double));
      memcpy(d. base, t->base, 3*sizeof(double));
      memcpy(d. SVID, t->SVID, RTCM_MAX_SATS * sizeof(uint8_t));
      memcpy(&d.Reference_Station_ID, &t->Reference_Station_ID, sizeof(uint16_t));

      for (size_t i = 0; i < RTCM_MAX_SATS; i++)
      {
        d.Pseudorange[i] = t->Pseudorange[i * RTCM_MAX_FREQS];
        d.Type[i] = t->Type[i * RTCM_MAX_FREQS];
        d.Carrier[i] = t->Carrier[i * RTCM_MAX_FREQS];
        d.CN0[i] = t->CN0[i * RTCM_MAX_FREQS];
        d.locktime[i] = t->locktime[i * RTCM_MAX_FREQS];
      }
      return d;
    }

    irt_msgs::msg::RTCML1E1 ConstructRTCM33_L1E1Msg(const boost::shared_ptr<RTCM_3_3_t>& t)
    {
      irt_msgs::msg::RTCML1E1 msg;
      msg.header.frame_id = "sapos";
      msg.tow = t->TOW;
      std::copy(std::begin(t->base), std::end(t->base), msg.base.begin());
      std::copy(std::begin(t->SVID), std::end(t->SVID), msg.svid.begin());
      msg.reference_station_id = t->Reference_Station_ID;

      for (size_t i = 0; i < RTCM_MAX_SATS; i++)
      {
        msg.pseudorange[i] = t->Pseudorange[i * RTCM_MAX_FREQS];
        msg.type[i] = t->Type[i * RTCM_MAX_FREQS];
        msg.carrier[i] = t->Carrier[i * RTCM_MAX_FREQS];
        msg.cn0[i] = t->CN0[i * RTCM_MAX_FREQS];
        msg.locktime[i] = t->locktime[i * RTCM_MAX_FREQS];
        msg.lli[i] = t->lli[i * RTCM_MAX_FREQS];
      }
      return msg;
    }

    irt_msgs::msg::RTCMV3 constctRTCMV3Msg(const boost::shared_ptr<RTCM_3_3_t>& t)
    {
        irt_msgs::msg::RTCMV3 msg;
        msg.header.frame_id = "sapos";
        msg.tow = t->TOW;
        std::copy(std::begin(t->base), std::end(t->base), msg.base.begin());
        std::copy(std::begin(t->SVID), std::end(t->SVID), msg.svid.begin());
        msg.reference_station_id = t->Reference_Station_ID;

        for(size_t i = 0; i < RTCM_MAX_FREQS * RTCM_MAX_SATS; i++)
        {
            msg.pseudorange[i] = t->Pseudorange[i];
            msg.type[i] = t->Type[i];
            msg.carrier[i] = t->Carrier[i];
            msg.cn0[i] = t->CN0[i];
            msg.locktime[i] = t->locktime[i];
            msg.lli[i] = t->lli[i];
        }
        return msg;
    }

    irt_msgs::msg::RTCM1004 constructRTCM1004Msg(const boost::shared_ptr<RTCM_30_1004_t>& t)
    {
        irt_msgs::msg::RTCM1004 msg;
        msg.header.frame_id = "sapos";
        msg.msg_number = t->Message_Number;
        msg.ref_station_id = t->Reference_Station_ID;
        msg.gps_epoch_time_ms = t->GPS_Epoche_Time_ms;
        msg.flag_gnss_synchronous = t->Synchronous_GNSS_Flag;
        msg.no_gps_sat_signal_processed = t->No_of_GPS_Satellite_Signals_Processed;
        msg.divergency_free_smoothing_indicator = t->GPS_Divergence_free_Smoothing_Indicator;
        msg.smoothing_interval = t->GPS_Smoothing_Interval;
        msg.msg_counter = t->message_counter;
        std::copy(std::begin(t->GPS_Satellite_ID), std::end(t->GPS_Satellite_ID), msg.sat_id.begin());
        std::copy(std::begin(t->GPS_L1_Code_Indicator), std::end(t->GPS_L1_Code_Indicator), msg.l1_code_indicator.begin());
        std::copy(std::begin(t->GPS_L1_Pseudorange_mod1lightms_m), std::end(t->GPS_L1_Pseudorange_mod1lightms_m), msg.l1_pseudorange_mod1lightms_m.begin());
        std::copy(std::begin(t->GPS_L1_PhaseRange_L1_Pseudorange_m), std::end(t->GPS_L1_PhaseRange_L1_Pseudorange_m), msg.l1_phasenrange_l1_pseudorange_m.begin());
        std::copy(std::begin(t->GPS_L1_Lock_time_Indicator), std::end(t->GPS_L1_Lock_time_Indicator), msg.l1_locktime_indicator.begin());
        std::copy(std::begin(t->GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m), std::end(t->GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m), msg.l1_integer_pseudorange_modulus_ambiguity_m.begin());
        std::copy(std::begin(t->GPS_L1_CNR_dBHz), std::end(t->GPS_L1_CNR_dBHz), msg.l1_cnr_db_hz.begin());
        std::copy(std::begin(t->GPS_L2_Code_Indicator), std::end(t->GPS_L2_Code_Indicator), msg.l2_code_indicator.begin());
        std::copy(std::begin(t->GPS_L2_L1_Pseudorange_Difference_m), std::end(t->GPS_L2_L1_Pseudorange_Difference_m), msg.l2_l1_pseudorange_difference_m.begin());
        std::copy(std::begin(t->GPS_L2_PhaseRange_L1_Pseudorange_m), std::end(t->GPS_L2_PhaseRange_L1_Pseudorange_m), msg.l2_phaserange_l1_pseudorange_m.begin());
        std::copy(std::begin(t->GPS_L2_Lock_time_Indicator), std::end(t->GPS_L2_Lock_time_Indicator), msg.l2_locktime_indicator.begin());
        std::copy(std::begin(t->GPS_L2_CNR_dbHz), std::end(t->GPS_L2_CNR_dbHz), msg.l2_cnr_db_hz.begin());
        return msg;
    }

    irt_msgs::msg::RTCM1005 constructRTCM1005Msg(const boost::shared_ptr<RTCM_30_1005_t>& t)
    {
        irt_msgs::msg::RTCM1005 msg;
        msg.header.frame_id = "sapos";
        msg.msg_number = t->Message_Number;
        msg.ref_station_id = t->Reference_Station_ID;
        msg.reserved_itrf_realization_year = t->Reserved_for_ITRF_Realization_Year;
        msg.gps_indicator = t->GPS_Indicator;
        msg.glo_indicator = t->GLONASS_Indicator;
        msg.galileo_indicator = t->Reserved_for_Galileo_Indicator;
        msg.ref_station_id = t->Reference_Station_ID;
        msg.antenna_reference_point_ecef_x_m = t->Antenna_Reference_Point_ECEF_X_m;
        msg.antenna_reference_point_ecef_y_m = t->Antenna_Reference_Point_ECEF_Y_m;
        msg.antenna_reference_point_ecef_z_m = t->Antenna_Reference_Point_ECEF_Z_m;
        msg.signle_receiver_oscillator_indicator = t->Single_Receiver_Oscillator_Indicator;
        msg.reserved1 = t->Reserved1;
        msg.reserved2 = t->Reserved2;
        msg.msg_counter = t->message_counter;
        return msg;
    }

    Data_Dictionary_RTCM_3_3_L1_E1 ConvertRTCM3_3(const RTCM_3_3_L1_E1& t)
    {
      Data_Dictionary_RTCM_3_3_L1_E1 d;
      d.Preamble = 0x5555;
      d.CRC_16_Check_Sum = 0;
      d.Packet_Identifier = 111;
      d.Product_ID = t.Reference_Station_ID;
      d.Range[0] = t.TOW;
      for (size_t i = 0; i < 3; i++)
      {
        d.Range[i+1] = t.base[i];
      }
      for (size_t i = 0; i < 40; i++)
      {
        d.Number[i] = t.SVID[i];
      }
      for (size_t i = 0; i < 40; i++)
      {
        d.Range[i+4] = t.Pseudorange[i];
      }
      for (size_t i = 0; i < 40; i++)
      {
        d.Number[i+40] = t.Type[i];
      }
      for (size_t i = 0; i < 40; i++)
      {
        d.Range[i+44] = t.Carrier[i];
      }
      for (size_t i = 0; i < 40; i++)
      {
        d.Range[i+84] = t.CN0[i];
      }
      memcpy(&d.Number[80],&t.locktime[0],RTCM_MAX_SATS*sizeof(uint16_t));
      return d;
    }

    boost::shared_ptr<uint8_t>  TransDataDictionaryRTCM_3_3(Data_Dictionary_RTCM_3_3_L1_E1& t)
    {
      boost::shared_ptr<uint8_t> Array(new uint8_t[Length_DD_RTCM_3_3_L1E1], [](const uint8_t *Array) {delete[] Array; });
      auto * Array_No_CRC = new uint8_t[Length_DD_RTCM_3_3_L1E1-2];

      memcpy(&Array.get()[0], &t.Preamble, sizeof(uint16_t));
      memcpy(&Array.get()[2], &t.Packet_Identifier, sizeof(uint8_t));
      memcpy(&Array.get()[3], &(t.Range[0]), sizeof(double)*4);
      memcpy(&Array.get()[35], &(t.Number[0]), sizeof(uint8_t)*RTCM_MAX_SATS);
      memcpy(&Array.get()[75], &(t.Range[4]), sizeof(double)*RTCM_MAX_SATS);
      memcpy(&Array.get()[395], &(t.Number[40]), sizeof(uint8_t)*RTCM_MAX_SATS);
      memcpy(&Array.get()[435], &(t.Range[44]), sizeof(double)*RTCM_MAX_SATS*2);
      memcpy(&Array.get()[1075], &(t.Number[80]), sizeof(uint8_t)*RTCM_MAX_SATS*2);
      memcpy(&Array.get()[1155], &(t.Product_ID), sizeof(uint16_t));

      memcpy(Array_No_CRC,Array.get(),Length_DD_RTCM_3_3_L1E1-2);
      t.CRC_16_Check_Sum = CRC16_Modbus_Gen(Array_No_CRC,Length_DD_RTCM_3_3_L1E1-2);

      memcpy(&Array.get()[1157], &t.CRC_16_Check_Sum, sizeof(t.CRC_16_Check_Sum));

      delete [] Array_No_CRC;
      return Array;
    }
    //Convert Data in Struct<RTCM_30_1005> to Struct<Data_Dictionary_4_0_1005>
    Data_Dictionary_4_0_1005 Convert1005(const RTCM_30_1005_t& t)
    {
      Data_Dictionary_4_0_1005 d;
      d.Preamble = 0x5555;
      d.CRC_16_Check_Sum = 0;
      d.Packet_Identifier = 110;
      d.Product_ID[0] = t.Message_Number;
      d.Product_ID[1] = t.Reference_Station_ID;
      d.Time = t.message_counter;
      d.Number[0] = t.Reserved_for_ITRF_Realization_Year;
      d.Number[1] = t.GPS_Indicator;
      d.Number[2] = t.GLONASS_Indicator;
      d.Number[3] = t.Reserved_for_Galileo_Indicator;
      d.Number[4] = t.Reference_Station_Indicator;
      d.Number[5] = t.Single_Receiver_Oscillator_Indicator;
      d.Number[6] = t.Reserved1;
      d.Number[7] = t.Reserved2;
      d.Range[0] = t.Antenna_Reference_Point_ECEF_X_m;
      d.Range[1] = t.Antenna_Reference_Point_ECEF_Y_m;
      d.Range[2] = t.Antenna_Reference_Point_ECEF_Z_m;
      return d;
    }
    Data_Dictionary_4_0_1004 Convert1004(const RTCM_30_1004_t& t)
    {
      Data_Dictionary_4_0_1004 d;
      d.Preamble = 0x5555;
      d.CRC_16_Check_Sum = 0;
      d.Packet_Identifier = 109;
      d.Product_ID[0] = t.Message_Number;
      d.Product_ID[1] = t.Reference_Station_ID;
      d.Time[0]=t.GPS_Epoche_Time_ms;
      d.Time[1]=t.message_counter;
      d.Boolean_Flag[0] = t.Synchronous_GNSS_Flag;
      d.Boolean_Flag[1] = t.GPS_Divergence_free_Smoothing_Indicator;
      d.Number[0] = t.No_of_GPS_Satellite_Signals_Processed;
      d.Number[1] = t.GPS_Smoothing_Interval;

      for (int i = 0; i < 12; i++)
      {
        d.Number[i+2]=t.GPS_Satellite_ID[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Number[i+14]=t.GPS_L1_Code_Indicator[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Number[i+26]=t.GPS_L1_Lock_time_Indicator[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Number[i+38]=t.GPS_L2_Code_Indicator[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Number[i+50]=t.GPS_L2_Lock_time_Indicator[i];
      }

      for (int i = 0; i < 12; i++)
      {
        d.Range[i]=t.GPS_L1_Pseudorange_mod1lightms_m[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Range[i+12]=t.GPS_L1_PhaseRange_L1_Pseudorange_m[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Range[i+24]=t.GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Range[i+36]=t.GPS_L1_CNR_dBHz[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Range[i+48]=t.GPS_L2_L1_Pseudorange_Difference_m[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Range[i+60]=t.GPS_L2_PhaseRange_L1_Pseudorange_m[i];
      }
      for (int i = 0; i < 12; i++)
      {
        d.Range[i+72]=t.GPS_L2_CNR_dbHz[i];
      }
      return d;
    }
    boost::shared_ptr<uint8_t>  TransRTCM_1004(const RTCM_30_1004_t& t)
    {
      boost::shared_ptr<uint8_t> Array(new uint8_t[Length_1004], [](const uint8_t *Array) {delete[] Array; });
      memcpy(&Array.get()[0], &t.Message_Number, sizeof(t.Message_Number));
      memcpy(&Array.get()[2], &t.Reference_Station_ID, sizeof(t.Reference_Station_ID));
      memcpy(&Array.get()[4], &t.GPS_Epoche_Time_ms, sizeof(t.GPS_Epoche_Time_ms));
      memcpy(&Array.get()[8], &t.Synchronous_GNSS_Flag, sizeof(t.Synchronous_GNSS_Flag));
      memcpy(&Array.get()[9], &t.No_of_GPS_Satellite_Signals_Processed, sizeof(t.No_of_GPS_Satellite_Signals_Processed));
      memcpy(&Array.get()[10], &t.GPS_Divergence_free_Smoothing_Indicator, sizeof(t.GPS_Divergence_free_Smoothing_Indicator));
      memcpy(&Array.get()[11], &t.GPS_Smoothing_Interval, sizeof(t.GPS_Smoothing_Interval));
      memcpy(&Array.get()[12], &t.GPS_Satellite_ID, sizeof(t.GPS_Satellite_ID));
      memcpy(&Array.get()[24], &t.GPS_L1_Code_Indicator, sizeof(t.GPS_L1_Code_Indicator));
      memcpy(&Array.get()[36], &t.GPS_L1_Pseudorange_mod1lightms_m, sizeof(t.GPS_L1_Pseudorange_mod1lightms_m));
      memcpy(&Array.get()[132], &t.GPS_L1_PhaseRange_L1_Pseudorange_m, sizeof(t.GPS_L1_PhaseRange_L1_Pseudorange_m));
      memcpy(&Array.get()[228], &t.GPS_L1_Lock_time_Indicator, sizeof(t.GPS_L1_Lock_time_Indicator));
      memcpy(&Array.get()[240], &t.GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m, sizeof(t.GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m));
      memcpy(&Array.get()[336], &t.GPS_L1_CNR_dBHz, sizeof(t.GPS_L1_CNR_dBHz));
      memcpy(&Array.get()[432], &t.GPS_L2_Code_Indicator, sizeof(t.GPS_L2_Code_Indicator));
      memcpy(&Array.get()[444], &t.GPS_L2_L1_Pseudorange_Difference_m, sizeof(t.GPS_L2_L1_Pseudorange_Difference_m));
      memcpy(&Array.get()[540], &t.GPS_L2_PhaseRange_L1_Pseudorange_m, sizeof(t.GPS_L2_PhaseRange_L1_Pseudorange_m));
      memcpy(&Array.get()[636], &t.GPS_L2_Lock_time_Indicator, sizeof(t.GPS_L2_Lock_time_Indicator));
      memcpy(&Array.get()[648], &t.GPS_L2_CNR_dbHz, sizeof(t.GPS_L2_CNR_dbHz));
      memcpy(&Array.get()[744], &t.message_counter, sizeof(t.message_counter));
      return Array;
    }
    // Transform Data in Struct<RTCM_30_1005> to a uint8 Array
    boost::shared_ptr<uint8_t> TransRTCM_1005(const RTCM_30_1005_t& t)
    {
      boost::shared_ptr<uint8_t> Array(new uint8_t[Length_1005], [](const uint8_t *Array) {delete[] Array; });
      memcpy(&Array.get()[0], &t.Message_Number, sizeof(t.Message_Number));
      memcpy(&Array.get()[2], &t.Reference_Station_ID, sizeof(t.Reference_Station_ID));
      memcpy(&Array.get()[4], &t.Reserved_for_ITRF_Realization_Year, sizeof(t.Reserved_for_ITRF_Realization_Year));
      memcpy(&Array.get()[5], &t.GPS_Indicator, sizeof(t.GPS_Indicator));
      memcpy(&Array.get()[6], &t.GLONASS_Indicator, sizeof(t.GLONASS_Indicator));
      memcpy(&Array.get()[7], &t.Reserved_for_Galileo_Indicator, sizeof(t.Reserved_for_Galileo_Indicator));
      memcpy(&Array.get()[8], &t.Reference_Station_Indicator, sizeof(t.Reference_Station_Indicator));
      memcpy(&Array.get()[9], &t.Antenna_Reference_Point_ECEF_X_m, sizeof(t.Antenna_Reference_Point_ECEF_X_m));
      memcpy(&Array.get()[17], &t.Single_Receiver_Oscillator_Indicator, sizeof(t.Single_Receiver_Oscillator_Indicator));
      memcpy(&Array.get()[18], &t.Reserved1, sizeof(t.Reserved1));
      memcpy(&Array.get()[19], &t.Antenna_Reference_Point_ECEF_Y_m, sizeof(t.Antenna_Reference_Point_ECEF_Y_m));
      memcpy(&Array.get()[27], &t.Reserved2, sizeof(t.Reserved2));
      memcpy(&Array.get()[28], &t.Antenna_Reference_Point_ECEF_Z_m, sizeof(t.Antenna_Reference_Point_ECEF_Z_m));
      memcpy(&Array.get()[36], &t.message_counter, sizeof(t.message_counter));
      return Array;
    }
    boost::shared_ptr<uint8_t> TransDataDictionary_1004(Data_Dictionary_4_0_1004& t)
    {
      boost::shared_ptr<uint8_t> Array(new uint8_t[Length_DD_4_1004], [](const uint8_t *Array) {delete[] Array; });
      auto * Array_No_CRC = new uint8_t[Length_DD_4_1004-2];

      memcpy(&Array.get()[0], &t.Preamble, sizeof(uint16_t));
      memcpy(&Array.get()[2], &t.Packet_Identifier, sizeof(uint8_t));
      memcpy(&Array.get()[3], &t.Product_ID, sizeof(uint16_t)*2);
      memcpy(&Array.get()[7], &(t.Time[0]), sizeof(uint32_t));
      memcpy(&Array.get()[11], &(t.Boolean_Flag[0]), sizeof(uint8_t));
      memcpy(&Array.get()[12], &(t.Number[0]), sizeof(uint8_t));
      memcpy(&Array.get()[13], &(t.Boolean_Flag[1]), sizeof(uint8_t));
      memcpy(&Array.get()[14], &(t.Number[1]), sizeof(uint8_t));
      memcpy(&Array.get()[15], &(t.Number[2]), sizeof(uint8_t)*24);
      memcpy(&Array.get()[39], &(t.Range[0]), sizeof(double)*24);
      memcpy(&Array.get()[231], &(t.Number[26]), sizeof(uint8_t)*12);
      memcpy(&Array.get()[243], &(t.Range[24]), sizeof(double)*24);
      memcpy(&Array.get()[435], &(t.Number[38]), sizeof(uint8_t)*12);
      memcpy(&Array.get()[447], &(t.Range[48]), sizeof(double)*24);
      memcpy(&Array.get()[639], &(t.Number[50]), sizeof(uint8_t)*12);
      memcpy(&Array.get()[651], &(t.Range[72]), sizeof(double)*12);
      memcpy(&Array.get()[747], &(t.Time[1]), sizeof(uint32_t));

      memcpy(Array_No_CRC,Array.get(),Length_DD_4_1004-2);
      t.CRC_16_Check_Sum=CRC16_Modbus_Gen(Array_No_CRC,Length_DD_4_1004-2);

      memcpy(&Array.get()[751], &t.CRC_16_Check_Sum, sizeof(t.CRC_16_Check_Sum));

      delete [] Array_No_CRC;
      return Array;
    }
    // Transform Data in Struct<Data_Dictionary_4_0_1005> to a uint8 Array, last 2 positions(bytes) generate a CRC_Code
    boost::shared_ptr<uint8_t> TransDataDictionary_1005(Data_Dictionary_4_0_1005& t)
    {

      boost::shared_ptr<uint8_t> Array(new uint8_t[Length_DD_4_1005], [](const uint8_t *Array) {delete[] Array; });
      auto * Array_No_CRC = new uint8_t[Length_DD_4_1005-2];

      memcpy(&Array.get()[0], &t.Preamble, sizeof(uint16_t));
      memcpy(&Array.get()[2], &t.Packet_Identifier, sizeof(uint8_t));
      memcpy(&Array.get()[3], &t.Product_ID, sizeof(uint16_t)*2);
      memcpy(&Array.get()[7], &(t.Number[0]), sizeof(uint8_t)*5);
      memcpy(&Array.get()[12], &(t.Range[0]), sizeof(double));
      memcpy(&Array.get()[20], &(t.Number[5]), sizeof(uint8_t)*2);
      memcpy(&Array.get()[22], &(t.Range[1]), sizeof(double));
      memcpy(&Array.get()[30], &(t.Number[7]), sizeof(uint8_t));
      memcpy(&Array.get()[31], &(t.Range[2]), sizeof(double));
      memcpy(&Array.get()[39], &t.Time, sizeof(uint32_t));

      memcpy(Array_No_CRC,Array.get(),Length_DD_4_1005-2);
      t.CRC_16_Check_Sum=CRC16_Modbus_Gen(Array_No_CRC,Length_DD_4_1005-2);

      memcpy(&Array.get()[43], &t.CRC_16_Check_Sum, sizeof(t.CRC_16_Check_Sum));

      delete [] Array_No_CRC;
      return Array;
    }
}
#endif //NTRIP_ROS_UTILS_H
