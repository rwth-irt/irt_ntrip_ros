/*
 * RTCMParser.h
 *
 *  Created on: 15.06.2018
 *      Author: Kd
 */

#pragma once

#include <map>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <boost/shared_ptr.hpp>
#include <boost/signals2/signal.hpp>
#include "rtcm_types.h"

#define RTCM_HEADER_LENGTH 3
#define RTCM_HEADER_AND_CRC_LENGTH 6
#define RTCM_MINIMUM_MSG_LENGTH 5
#define RTCM_PREAMBLE 0xD3

#define RTCM_1004_HEADER_SIZE_BITS 64
#define RTCM_1004_PER_SAT_SIZE_BITS 125

#define RTCM_MSM_HEADER_SIZE 169

#define P2_10 0.0009765625 			/* 2^-10 */
#define P2_24 5.960464477539063E-08 /* 2^-24 */
#define P2_29 1.862645149230957E-09 /* 2^-29 */

#define CLIGHT 299792458.0 			/* speed of light [m/s]  */
#define RANGE_MS (CLIGHT*0.001) 	/* range in 1 ms 		 */

#define FREQ1 1.57542E9             /* L1/E1  frequency [Hz] */
#define FREQ2 1.22760E9             /* L2     frequency [Hz] */
#define FREQ5 1.17645E9             /* L5/E5a frequency [Hz] */
#define FREQ7 1.20714E9             /* E5b    frequency [Hz] */
#define FREQ8 1.191795E9 		    /* E5a+b  frequency [Hz] */

#define LAMBDA_1 CLIGHT/FREQ1 		/* L1 / E1  [m]			 */
#define LAMBDA_2 CLIGHT/FREQ2 		/* L2  	  	[m]			 */
#define LAMBDA_5 CLIGHT/FREQ5 		/* L5 / E5a [m]			 */
#define LAMBDA_7 CLIGHT/FREQ7 		/* E5b 	  	[m]			 */
#define LAMBDA_8 CLIGHT/FREQ8 		/* E5  	  	[m]			 */


struct RtcmStats {
	RtcmStats()
	: numOfMsgs(std::map<uint16_t, unsigned int>())
	, numCrcError(0)
	, numBufferOverrun(0)
	, numTruncatedBytes(0)
	{};

	std::map<uint16_t, unsigned int> numOfMsgs;
	unsigned int numCrcError;
	unsigned int numBufferOverrun;
	unsigned int numTruncatedBytes;
};

//extern boost::shared_ptr<uint> Mark;

class RtcmParser {
public:

	// typedefs for Boost signals
	typedef boost::signals2::signal<void(boost::shared_ptr<RTCM_30_1004_t>)> Msg1004Signal_t;
	typedef boost::signals2::signal<void(boost::shared_ptr<RTCM_30_1005_t>)> Msg1005Signal_t;
	typedef boost::signals2::signal<void(boost::shared_ptr<RTCM_3_3_t>)> RTCM_3_3_Signal_t;

	// constructor, destructor
	explicit RtcmParser(size_t bufferSize);
	~RtcmParser();
	
	// add bytes to parse buffer
	void addBytesToBuffer(boost::shared_ptr<uint8_t[]> data, size_t length);
	boost::shared_ptr<uint> Index_L1E1;
		
	// callback register functions
	boost::signals2::connection registerCbMsg1004(const Msg1004Signal_t::slot_type& subscriber);
	boost::signals2::connection registerCbMsg1005(const Msg1005Signal_t::slot_type& subscriber);
	boost::signals2::connection registerCbRTCM33(const RTCM_3_3_Signal_t::slot_type& subscriber);

	// get rtcm stats, only for debug
	[[nodiscard]] const RtcmStats& getStats() const;

private:

	// global byte parser, search for valid rtcm msgs
	void parseBytes();

	// global decode function
	void decodeMsg(uint8_t* msgPtr, size_t msgLength);

	// SSM functions, only GPS so far
	void decodeMsg1004(uint8_t* msgPtr, size_t msgLength);
	void decodeMsg1005(uint8_t* msgPtr, size_t msgLength);
	void decodeMsg1006(uint8_t* msgPtr, size_t msgLength);

	// MSM functions, GPS and Galileo so far
	static int decodeMSMheader(const uint8_t* msgPtr, size_t msgLength, uint8_t sys, RTCM_MSM_h_t &head);
	void save_msm_obs(uint8_t sys, RTCM_MSM_h_t &head, const double *r, const double *pr, const double *cp,
                    const double *rrf, const double *cnr, const uint16_t *lock, const uint8_t *ex,
                    const uint8_t *half);
	int decodeMSM4(const uint8_t* msgPtr, size_t msgLength, uint8_t sys);
	void add1005ToStorage(const boost::shared_ptr<RTCM_30_1005_t>&);
	void reset_rtcm_buffer();
	unsigned int getSVID_index(uint8_t svid);
	unsigned int getnextfree_index(uint8_t svid_index);

	// calculate Qualcomm CRC24
	static unsigned int crc24q(const uint8_t* buffer, size_t length);
	static void displayErrorMsg(const std::string& errorMsg);
	
	// bit level decode functions
	static unsigned int decodeUnsignedInteger(const uint8_t* bufferPtr, unsigned int pos, uint8_t length);
	static int decodeSignedInteger(const uint8_t* bufferPtr, unsigned int pos, uint8_t length);
	static uint64_t decodeUnsignedInteger64(const uint8_t* bufferPtr, unsigned int pos, uint8_t length);
	static int64_t decodeSignedInteger64(const uint8_t* bufferPtr, unsigned int pos, uint8_t length);
	
	// get SBF_SVID/SBF_Type from RTCM ids
	static uint8_t getSBF_SVID(uint8_t svid, uint8_t sys);
	static uint8_t getSBF_Type(uint8_t type, uint8_t sys);

	// get wavelength for signal type
	static double get_wavelength(uint8_t type);


	// signals for decoded msgs
	Msg1004Signal_t m_msg1004Signal;
	Msg1005Signal_t m_msg1005Signal;
	RTCM_3_3_Signal_t m_rtcm_3_3_Signal;

	// struct for stats
	RtcmStats m_stats;

	// RTCM 3.3 Speicher
	boost::shared_ptr<RTCM_3_3_t> rtcm_3_3_buff;
    unsigned short obs_locks[RTCM_MAX_SATS][RTCM_MAX_FREQS];
	// Byte Buffer
	size_t m_bufferSize;
	uint8_t* m_tmpBuffer;
	uint8_t* m_parseBuffer;
	size_t m_numBytesParseBuffer;

	// MSM signal mappings
	const char *msm_sig_gps[32] = {
		/* GPS: RTCM 3.3 table 3.5-91 for DF 395 */
		""  ,"1C","1P","1W",""  ,""  ,""  ,"2C","2P","2W",""  ,""  , /*  1-12 */
		""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
		""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
	};

	const char *msm_sig_gal[32] = {
		/* Galileo: RTCM 3.3 table 3.5-99 for DF 395 */
		""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z", /*  1-12 */
		""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X", /* 13-24 */
		""  ,""  ,""  ,""  ,""  ,""  ,""  ,""						 /* 25-32 */
	};

    const char *msm_sig_glo[32]={
        /* GLONASS: ref [13] table 3.5-93, ref [14][15] table 3.5-97 */
        ""  ,"1C","1P",""  ,""  ,""  ,""  ,"2C","2P",""  ,"3I","3Q",
        "3X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
        ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
    };

    const char *msm_sig_qzs[32]={
        /* QZSS: ref [15] table 3.5-103 */
        ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,"6S","6L","6X",""  ,
        ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X",
        ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"
    };
    const char *msm_sig_sbs[32]={
        /* SBAS: ref [13] table 3.5-T+005 */
        ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
        ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5I","5Q","5X",
        ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
    };
    const char *msm_sig_cmp[32]={
        /* BeiDou: ref [15] table 3.5-106 */
        ""  ,"1I","1Q","1X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,
        ""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
        ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
    };

	// SYS Types
	const char *msm_sys[3] = {
			"GPS", "GLO", "GAL"
	};
};

inline std::ostream& operator<< (std::ostream& stream, const RtcmStats& stats){
	for(auto numOfMsg : stats.numOfMsgs){
		stream << "# of Message " << numOfMsg.first << ":\t" << numOfMsg.second << std::endl;
	}
	stream << "CRC Errors:\t" << stats.numCrcError << std::endl;
	stream << "Buffer Overruns:\t" << stats.numBufferOverrun << std::endl;
	stream << "Truncated bytes:\t" << stats.numTruncatedBytes << std::endl;
	return stream;
}

inline std::ostream& operator<< (std::ostream& stream, const RTCM_30_1004_t& msg){
	stream << "Msg Number: " << static_cast<uint16_t>(msg.Message_Number) << std::endl;
	stream << "Counter: " << static_cast<uint32_t>(msg.message_counter) << std::endl;
	stream << "Reference Station ID: " << static_cast<uint16_t>(msg.Reference_Station_ID) << std::endl;
	stream << "GPS Epoch Time: " << static_cast<uint32_t>(msg.GPS_Epoche_Time_ms) << std::endl;
	stream << "Synchronous GNSS Flag: " << static_cast<unsigned int>(msg.Synchronous_GNSS_Flag) << std::endl;
	stream << "No. of GPS Satellite Signals Processed: " << static_cast<unsigned int>(msg.No_of_GPS_Satellite_Signals_Processed) << std::endl;
	stream << "GPS Divergence-free Smoothing Indicator: " << static_cast<unsigned int>(msg.GPS_Divergence_free_Smoothing_Indicator) << std::endl;
	stream << "GPS Smoothing Interval: " << static_cast<unsigned int>(msg.GPS_Smoothing_Interval) << std::endl;
	for(uint8_t i=0; i<msg.No_of_GPS_Satellite_Signals_Processed; i++){
		stream << std::endl;
		stream << "Satellite Number: " << static_cast<unsigned int>(i) << std::endl;
		stream << "GPS Satellite ID: " << static_cast<unsigned int>(msg.GPS_Satellite_ID[i]) << std::endl;
		stream << "GPS L1 Code Indicator: " << static_cast<unsigned int>(msg.GPS_L1_Code_Indicator[i]) << std::endl;
		stream << "GPS L1 Pseudorange: " << static_cast<double>(msg.GPS_L1_Pseudorange_mod1lightms_m[i]) << std::endl;
		stream << "GPS L1 Phaserange - L1 Pseudorange: " << static_cast<double>(msg.GPS_L1_PhaseRange_L1_Pseudorange_m[i]) << std::endl;
		stream << "GPS L1 Lock time Indicator: " << static_cast<unsigned int>(msg.GPS_L1_Lock_time_Indicator[i]) << std::endl;
		stream << "GPS Integer L1 Pseudorange Modulus Amb.: " << static_cast<double>(msg.GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m[i]) << std::endl;
		stream << "GPS L1 CNR: " << static_cast<double>(msg.GPS_L1_CNR_dBHz[i]) << std::endl;
		stream << "GPS L2 Code Indicator: " << static_cast<unsigned int>(msg.GPS_L2_Code_Indicator[i]) << std::endl;
		stream << "GPS L2-L1 Pseudorange Diff.: " << static_cast<double>(msg.GPS_L2_L1_Pseudorange_Difference_m[i]) << std::endl;
		stream << "GPS L2 Phaserange - L1 Pseudorange: " << static_cast<double>(msg.GPS_L2_PhaseRange_L1_Pseudorange_m[i]) << std::endl;
		stream << "GPS L2 Lock time Indicator: " << static_cast<unsigned int>(msg.GPS_L2_Lock_time_Indicator[i]) << std::endl;
		stream << "GPS L2 CNR: " << static_cast<double>(msg.GPS_L2_CNR_dbHz[i]) << std::endl;
	}
	return stream;
}

inline std::ostream& operator<< (std::ostream& stream, const RTCM_30_1005_t& msg){
	stream << "Msg Number: " << static_cast<uint16_t>(msg.Message_Number) << std::endl;
	stream << "Counter: " << static_cast<uint32_t>(msg.message_counter) << std::endl;
	stream << "Reference Station ID: " << static_cast<uint16_t>(msg.Reference_Station_ID) << std::endl;
	stream << "GPS Indicator: " << static_cast<unsigned int>(msg.GPS_Indicator) << std::endl;
	stream << "GLONASS Indicator: " << static_cast<unsigned int>(msg.GLONASS_Indicator) << std::endl;
	stream << "Reference Station Indicator: " << static_cast<unsigned int>(msg.Reference_Station_Indicator) << std::endl;
	stream << "Single Receiver Oscillator Indicator: " << static_cast<unsigned int>(msg.Single_Receiver_Oscillator_Indicator) << std::endl;
	stream << "Antenna Reference Point ECEF-X: " << static_cast<double>(msg.Antenna_Reference_Point_ECEF_X_m) << std::endl; 
	stream << "Antenna Reference Point ECEF-Y: " << static_cast<double>(msg.Antenna_Reference_Point_ECEF_Y_m) << std::endl; 
	stream << "Antenna Reference Point ECEF-Z: " << static_cast<double>(msg.Antenna_Reference_Point_ECEF_Z_m) << std::endl; 
	return stream;
}
