/*
 * RTCMParser_types.h
 *
 *  Created on: 15.06.2018
 *      Author: Kd
 */

#pragma once

#pragma pack(push) // push current alignment to stack
#pragma pack(1)    // set alignment boundary to 1 byte

#include <stdint.h>

// SYS Constants
#define SYS_GPS 0
#define SYS_GLO 1
#define SYS_GAL 2
#define SYS_UNKNOWN 255

// SVID Offsets to SBF
#define SVID_OFFSET_GPS 0
#define SVID_OFFSET_GLO 37
#define SVID_OFFSET_GAL 70

// System Constants: ATTENTION: CHANGES HERE REQUIRE CHANGES TO THE TARGET DATA TYPE (e.g. Simulink)
#define RTCM_MAX_FREQS 5
#define RTCM_MAX_SATS 40
#define UINT8 uint8_t
#define UINT16 uint16_t
#define UINT32 uint32_t

// MSM header
typedef struct {
	uint16_t Message_Number;
	uint16_t Reference_Station_ID;
	uint32_t GNSS_Epoche_Time_ms;
	uint8_t multiple_message;
	uint8_t iod;              		/* issue of data station */
	uint8_t clk_str;          		/* clock steering indicator */
	uint8_t clk_ext;          		/* external clock indicator */
	uint8_t smooth;           		/* divergence free smoothing indicator */
	uint8_t tint_s;           		/* smoothing interval */
	uint8_t nsat,nsig;        		/* number of satellites/signals */
	uint8_t sats[64];         		/* satellites */
	uint8_t sigs[32];         		/* signals */
	uint8_t cellmask[64];     		/* cell mask */
} RTCM_MSM_h_t;

// RTCM V3 msg content
typedef struct {
	double TOW;
	double base[3];
	uint8_t SVID[RTCM_MAX_SATS];
	double Pseudorange[RTCM_MAX_FREQS*RTCM_MAX_SATS];
	double Doppler[RTCM_MAX_FREQS*RTCM_MAX_SATS];
	uint8_t Type[RTCM_MAX_FREQS*RTCM_MAX_SATS];
	double Carrier[RTCM_MAX_FREQS*RTCM_MAX_SATS];
	double CN0[RTCM_MAX_FREQS*RTCM_MAX_SATS];
	uint16_t locktime[RTCM_MAX_FREQS*RTCM_MAX_SATS];
	uint16_t lli[RTCM_MAX_FREQS*RTCM_MAX_SATS];
	uint16_t Reference_Station_ID;
} RTCM_3_3_t;

typedef struct {
	double TOW;
	double base[3];
	uint8_t SVID[RTCM_MAX_SATS];
	double Pseudorange[RTCM_MAX_SATS];
	uint8_t Type[RTCM_MAX_SATS];
	double Carrier[RTCM_MAX_SATS];
	double CN0[RTCM_MAX_SATS];
	uint16_t locktime[RTCM_MAX_SATS];
	uint16_t Reference_Station_ID;
} RTCM_3_3_L1_E1;


// SSR corrections for GPS
typedef struct {
  uint16_t Message_Number;
  uint16_t Reference_Station_ID;
  uint32_t GPS_Epoche_Time_ms;
  uint8_t Synchronous_GNSS_Flag;
  uint8_t No_of_GPS_Satellite_Signals_Processed;
  uint8_t GPS_Divergence_free_Smoothing_Indicator;
  uint8_t GPS_Smoothing_Interval;
  uint8_t GPS_Satellite_ID[12];
  uint8_t GPS_L1_Code_Indicator[12];
  double GPS_L1_Pseudorange_mod1lightms_m[12];
  double GPS_L1_PhaseRange_L1_Pseudorange_m[12];
  uint8_t GPS_L1_Lock_time_Indicator[12];
  double GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m[12];
  double GPS_L1_CNR_dBHz[12];
  uint8_t GPS_L2_Code_Indicator[12];
  double GPS_L2_L1_Pseudorange_Difference_m[12];
  double GPS_L2_PhaseRange_L1_Pseudorange_m[12];
  uint8_t GPS_L2_Lock_time_Indicator[12];
  double GPS_L2_CNR_dbHz[12];
  uint32_t message_counter;
} RTCM_30_1004_t;


// Reference Station Message
typedef struct {
  uint16_t Message_Number;
  uint16_t Reference_Station_ID;
  uint8_t Reserved_for_ITRF_Realization_Year;
  uint8_t GPS_Indicator;
  uint8_t GLONASS_Indicator;
  uint8_t Reserved_for_Galileo_Indicator;
  uint8_t Reference_Station_Indicator;
  double Antenna_Reference_Point_ECEF_X_m;
  uint8_t Single_Receiver_Oscillator_Indicator;
  uint8_t Reserved1;
  double Antenna_Reference_Point_ECEF_Y_m;
  uint8_t Reserved2;
  double Antenna_Reference_Point_ECEF_Z_m;
  uint32_t message_counter;
} RTCM_30_1005_t;

// Reference Station Message
typedef struct {
  uint16_t Message_Number;
  uint16_t Reference_Station_ID;
  uint8_t Reserved_for_ITRF_Realization_Year;
  uint8_t GPS_Indicator;
  uint8_t GLONASS_Indicator;
  uint8_t Reserved_for_Galileo_Indicator;
  uint8_t Reference_Station_Indicator;
  double Antenna_Reference_Point_ECEF_X_m;
  uint8_t Single_Receiver_Oscillator_Indicator;
  uint8_t Reserved1;
  double Antenna_Reference_Point_ECEF_Y_m;
  uint8_t Reserved2;
  double Antenna_Reference_Point_ECEF_Z_m;
  double Antenna_Height;
  uint32_t message_counter;
} RTCM_30_1006_t;

typedef struct {
  uint16_t Preamble;
  uint8_t  Packet_Identifier;
  uint16_t Product_ID[2];
  uint32_t Time;
  uint8_t  Number[8];
  double Range[3];
  uint16_t CRC_16_Check_Sum;
} Data_Dictionary_4_0_1005;

typedef struct {
  uint16_t Preamble;
  uint8_t  Packet_Identifier;
  uint16_t Product_ID[2];
  uint32_t Time[2];
  uint8_t  Boolean_Flag[2];
  uint8_t Number[62];
  double Range[84];
  uint16_t CRC_16_Check_Sum;
} Data_Dictionary_4_0_1004;


typedef struct {
  uint16_t Preamble;
  uint8_t  Packet_Identifier;
  uint16_t Product_ID;
  uint8_t  Number[160];
  double Range[124];
  uint16_t CRC_16_Check_Sum;
} Data_Dictionary_RTCM_3_3_L1_E1;

#pragma pack(pop) // zurück zur alten Fragmentierung
