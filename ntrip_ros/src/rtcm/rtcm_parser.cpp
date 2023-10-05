/*
 * RTCMParser.cpp
 *
 *  Created on: 15.06.2018
 *      Author: Kd
 */

#include <iostream>
#include <cstring>
#include <cstdint>

#include "rtcm/rtcm_parser.h"

RtcmParser::RtcmParser(size_t bufferSize) :
  Index_L1E1(new uint[40], [](uint *Index_L1E1) {delete[] Index_L1E1; }),
  m_bufferSize(bufferSize),
  m_tmpBuffer(new uint8_t[bufferSize]),
  m_parseBuffer(new uint8_t[bufferSize]),
  m_numBytesParseBuffer(0)

{
	// create new buffer, init with do not use values
	reset_rtcm_buffer();
	
	// store 1005 (station reference position) in rtcm33_buffer
	registerCbMsg1005(boost::bind(&RtcmParser::add1005ToStorage, this, boost::placeholders::_1));
};

RtcmParser::~RtcmParser()
{
	delete[] m_tmpBuffer;
	delete[] m_parseBuffer;

};

/// Add new bytes to the parse buffer and trigger a parse. If a complete message is subsequently found in the buffer
/// all registered callbacks for the corresponding message type will be called
void RtcmParser::addBytesToBuffer(boost::shared_ptr<uint8_t[]> data, size_t length){
	// check if new bytes fit in current parse buffer
	if (m_numBytesParseBuffer + length > m_bufferSize)
	{ 
		// ok, we need to truncate bytes in the parse buffer...
		std::stringstream errorMsg;
		
		//check if only the new bytes in the read buffer fit in parse buffer
		if (length <= m_bufferSize)
		{ 
			m_stats.numBufferOverrun++;
			// truncate parseBuffer at start and make space at end
			size_t numTruncate = length - (m_bufferSize - m_numBytesParseBuffer);
			m_stats.numTruncatedBytes += numTruncate;
			size_t numKeep = m_numBytesParseBuffer - numTruncate;

			static unsigned int errorcount = 0;
			errorcount++;
			if (errorcount > 2)  
			{
				errorMsg << "parse buffer full, discarding " << numTruncate << " bytes";
				displayErrorMsg(errorMsg.str());
			}

			// reset tmpParseBuffer
			std::memset(m_tmpBuffer, 0, m_bufferSize);
			// copy bytes to keep to tmpBuffer
			std::memcpy(m_tmpBuffer, m_parseBuffer + numTruncate, numKeep);
			// reset ParseBuffer
			std::memset(m_parseBuffer, 0, m_bufferSize);
			// copy tmpBuffer back to start of parseBuffer
			std::memcpy(m_parseBuffer, m_tmpBuffer, numKeep);
			m_numBytesParseBuffer = numKeep;
			// now there a numberOfBytesInReadBuffer bytes space at the end of parseBuffer
		} else { //
			displayErrorMsg("Too many bytes transmitted at once, increase parseBuffer size or decrease packet size");
		}
	}
	// copy receiveBuffer Bytes to end of parseBuffer
	std::memcpy(m_parseBuffer + m_numBytesParseBuffer, data.get(), length);
	// update parseBuffer_numberOfBytes
	m_numBytesParseBuffer += length;
	// parse all complete messages:
	parseBytes();
}

/// Register a callback function that is called by the parser when a new message of type 1004 is decoded
boost::signals2::connection RtcmParser::registerCbMsg1004(const Msg1004Signal_t::slot_type& subscriber) 
{	
	return m_msg1004Signal.connect(subscriber);
}

/// Register a callback function that is called by the parser when a new message of type 1005 is decoded
boost::signals2::connection RtcmParser::registerCbMsg1005(const Msg1005Signal_t::slot_type& subscriber) 
{
	return m_msg1005Signal.connect(subscriber);
}

/// Register a callback function that is called by the parser when a complete MSM set is decoded
boost::signals2::connection RtcmParser::registerCbRTCM33(const RTCM_3_3_Signal_t::slot_type& subscriber)
{
	return m_rtcm_3_3_Signal.connect(subscriber);
}

const RtcmStats& RtcmParser::getStats() const {
	return m_stats;
}

/// Parse parseBuffer for complete RTCM Messages and call decodeMsg with complete messages
/// Clean up the parseBuffer afterwards
void RtcmParser::parseBytes() 
{
	size_t i = 0;
	uint8_t* bufferPtr;
	
	while (i < m_numBytesParseBuffer)
	{
		bufferPtr = m_parseBuffer + i;
		size_t numBytesRemaining = m_numBytesParseBuffer-i;

		// check, if remaining number of bytes is sufficient for header and chksum
		if (numBytesRemaining < RTCM_HEADER_AND_CRC_LENGTH)
		{
			// break while loop, buffer not big enough for header, wait for more bytes
			break;
		}

		// check for preamble
		if (bufferPtr[0] != RTCM_PREAMBLE)
		{
			// bytes are not a message start, continue with next byte
			i++;
			continue;
		}

		auto msgLength = static_cast<uint16_t>(decodeUnsignedInteger(bufferPtr, 14, 10));

		// check, if more than 1023 bytes of payload --> invalid msg
		if (msgLength > 1023)
		{
			// invalid msg length
			i++;
			continue;
		}

		// check, if enough bytes are in buffer
		if(numBytesRemaining < RTCM_HEADER_AND_CRC_LENGTH+msgLength)
		{
			// wait for more bytes
			break;
		}

		// do crc check
		unsigned int theirCrc = decodeUnsignedInteger(bufferPtr+RTCM_HEADER_LENGTH+msgLength, 0, 24);
		unsigned int ourCrc = crc24q(bufferPtr, RTCM_HEADER_LENGTH+msgLength);

		if(ourCrc == theirCrc)
		{
			// valid, now parse it!
			decodeMsg(bufferPtr+RTCM_HEADER_LENGTH, msgLength);
			i += RTCM_HEADER_AND_CRC_LENGTH + msgLength;
		}
		else
		{
			// invalid CRC
			m_stats.numCrcError++;
			i++;
		}
	
	}

	// copy not parsed numberOfMessageBytes into tmpBuffer
	size_t numNotParsedBytes = m_numBytesParseBuffer - i;
	if (numNotParsedBytes > 0) 
	{
		memcpy(m_tmpBuffer, m_parseBuffer + i, numNotParsedBytes); 	// copy unparsed bytes in tmpBuffer
	}

	// reset ParseBuffer
	memset(m_parseBuffer, 0, m_bufferSize);
	m_numBytesParseBuffer = 0;

	// copy not parsed bytes back in ParseBuffer
	if (numNotParsedBytes > 0) 
	{
		memcpy(m_parseBuffer, m_tmpBuffer, numNotParsedBytes);
		m_numBytesParseBuffer = numNotParsedBytes;
	}
}

/// Determine Message type and call appropriate decode function when possible
void RtcmParser::decodeMsg(uint8_t* msgPtr, size_t msgLength)
{
	int ret = 99;

	auto msgNumber = static_cast<uint16_t>(decodeUnsignedInteger(msgPtr, 0, 12));
	m_stats.numOfMsgs[msgNumber]++;

	switch (msgNumber)
	{
		// SSR Messages
		case 1004: decodeMsg1004(msgPtr, msgLength); break;
		case 1005: decodeMsg1005(msgPtr, msgLength); break;
		case 1006: decodeMsg1006(msgPtr, msgLength); break;

		// MSM Messages
		case 1074: ret = decodeMSM4(msgPtr, msgLength, SYS_GPS); break;
		//case 1084: decodeMSM4(msgPtr, msgLength, SYS_GLO); break;
		case 1094: ret = decodeMSM4(msgPtr, msgLength, SYS_GAL); break;

		default:

			// detect end of measurement for not parsed MSM messages
			if (msgNumber >= 1070 && msgNumber <= 1229)
			{
				// decode MSM header
				RTCM_MSM_h_t head = {0};
				int ncell = decodeMSMheader(msgPtr, msgLength, SYS_UNKNOWN, head);
				ret = head.multiple_message;

				//std::cout << "Unknown MSM msg. Multiple Message Bit is " << ret << std::endl;
			}
			break;
	}

	//check, if MSM struct is ready: 1) reference station id is correct and thus > 0; 2) ret (MMB) is set to 0
	if (ret == 0 && rtcm_3_3_buff->Reference_Station_ID > 0)
	{
		// call subscribers
		m_rtcm_3_3_Signal(rtcm_3_3_buff);
	}
}

// Decode MSM4
int RtcmParser::decodeMSM4(const uint8_t* msgPtr, size_t msgLength, uint8_t sys)
{
	// decode MSM header
	RTCM_MSM_h_t head = {};
	int ncell = decodeMSMheader(msgPtr, msgLength, sys, head);

	// bit ptr to begin of payload
	unsigned int payload_ptr = RTCM_MSM_HEADER_SIZE + head.nsat * head.nsig;

	// go out
	if (ncell < 0)
	{
		displayErrorMsg("error parsing MSM header");
		return -1;
	}

	if (msgLength*8 < (RTCM_MSM_HEADER_SIZE + head.nsat * head.nsig + head.nsat*18 + ncell*48))
	{	
		
		displayErrorMsg("size of msg too small for MSM4 payload");
		return -1;
	}

	// pre-allocate data and nullify
	double r[64], pr[64], cp[64], cnr[64];
	int temp;
	uint8_t half[64];
	uint16_t lock[64];
	for (unsigned int j = 0; j < head.nsat; j++)
		r[j] = 0.0;
	for (unsigned int j = 0; j < ncell; j++)
		pr[j]=cp[j]=-1E16;

	// decode satellite data: range
	for (unsigned int j = 0; j < head.nsat; j++)
	{
		temp = static_cast<int>(decodeUnsignedInteger(msgPtr, payload_ptr, 8));
		if (temp != 255)
			r[j] = static_cast<double>(temp) * RANGE_MS;
		payload_ptr += 8;
	}
	for (unsigned int j = 0; j < head.nsat; j++)
	{
		temp = static_cast<int>(decodeUnsignedInteger(msgPtr, payload_ptr, 10));
		if (r[j] != 0.0)
			r[j] += static_cast<double>(temp) * P2_10 * RANGE_MS;
		payload_ptr += 10;
	}


	// decode signal data: pseudorange
	for (unsigned int j = 0; j < ncell; j++)
	{
		temp = static_cast<int>(decodeSignedInteger(msgPtr, payload_ptr, 15));
		if (temp != -16384)
			pr[j] = static_cast<double>(temp) * P2_24 * RANGE_MS;
		payload_ptr += 15;
	}
	// phaserange
	for (unsigned int j = 0; j < ncell; j++)
	{
		temp = static_cast<int>(decodeSignedInteger(msgPtr, payload_ptr, 22));
		if (temp != -2097152)
			cp[j] = static_cast<double>(temp) * P2_29 * RANGE_MS;
		payload_ptr += 22;
	}
	// lock time
	for (unsigned int j = 0; j < ncell; j++)
	{
		lock[j] = static_cast<uint16_t>(decodeUnsignedInteger(msgPtr, payload_ptr, 4));
		payload_ptr += 4;
	}
	// half-cycle ambiguity
	for (unsigned int j = 0; j < ncell; j++)
	{
		half[j] = static_cast<uint8_t>(decodeUnsignedInteger(msgPtr, payload_ptr, 1));
		payload_ptr ++;
	}
	// cnr
	for (unsigned int j = 0; j < ncell; j++)
	{
		cnr[j] = static_cast<double>(decodeUnsignedInteger(msgPtr, payload_ptr, 6));
		payload_ptr += 6;
	}

	// save observation
    save_msm_obs(sys, head, r, pr, cp, nullptr, cnr, lock, nullptr, half);

	return head.multiple_message;
}

void RtcmParser::save_msm_obs(uint8_t sys, RTCM_MSM_h_t &head, const double *r, const double *pr, const double *cp,
                              const double *rrf, const double *cnr, const uint16_t *lock, const uint8_t *ex,
                              const uint8_t *half)
{
	uint8_t svid, type;
	unsigned int sv_index;
	unsigned int sig_index = 0;
	double wl, rt;

	// j: counter for total number of observations (ncell)
	unsigned int j = 0;
	unsigned int t =0;

	// i: counter for current satellite vehicle
	for (unsigned int i = 0; i < head.nsat; i++)
	{
		// get SBF SVID
		svid = getSBF_SVID(head.sats[i], sys);

		// check for timediff if change of Epoch happened --> reinit whole struct
		rt = static_cast<double>(head.GNSS_Epoche_Time_ms) / 1000;
		if (rt > (rtcm_3_3_buff->TOW + 1e-9) || rt < (rtcm_3_3_buff->TOW - 10000.0))
		{
			// copy out current reference station
			double tmp_base[3];
			for (unsigned int iz = 0; iz < 3; iz++)
				tmp_base[iz] = rtcm_3_3_buff->base[iz];
			uint16_t tmp_ref_station = rtcm_3_3_buff->Reference_Station_ID;

			// reset whole RTCM 3.3 buffer
			reset_rtcm_buffer();

			// copy back reference station
			for (unsigned int iz = 0; iz < 3; iz++)
				rtcm_3_3_buff->base[iz] = tmp_base[iz];
			rtcm_3_3_buff->Reference_Station_ID = tmp_ref_station;
		}
		rtcm_3_3_buff->TOW = rt;

		// find SVID index if already in array
		sv_index = getSVID_index(svid);
		if (sv_index >= RTCM_MAX_SATS)
		{
			displayErrorMsg("Error: too many sats received, cannot store further sats.");
			return;
		}
		rtcm_3_3_buff->SVID[sv_index] = svid;

		// k: counter for current signal type
		for (unsigned int k = 0; k < head.nsig; k++)
		{
			
			// check if current sat / sig combo is set to 1 in cell mask
			if (!head.cellmask[k + i*head.nsig]) continue;

			// get next sat/sig index
			sig_index = getnextfree_index(sv_index);

			if (sig_index >= RTCM_MAX_SATS * RTCM_MAX_FREQS)
			{
				displayErrorMsg("Error: too many signals received, cannot store further signals.");
				return;
			}

			/* satellite carrier wave length */
			wl = get_wavelength(head.sigs[k]);

			// get SBF signal type
			type = getSBF_Type(head.sigs[k], sys);
			rtcm_3_3_buff->Type[sig_index] = type;
			//std::cout << "Type["<<sig_index<<"]:"<<int(rtcm_3_3_buff->Type[sig_index])<< std::endl;
			if (type == 1 || type == 0 || type == 17)
			{
				Index_L1E1.get()[t] = sig_index;
				t++;
			}
			/* pseudorange [m] */
			if (r[i] != 0.0 && pr[j] > -1E12)
			{
				rtcm_3_3_buff->Pseudorange[sig_index] = r[i] + pr[j];
			}

			/* carrier-phase [cycles] */
			if (r[i] != 0.0 && cp[j] > -1E12 && wl > 0.0)
			{
				rtcm_3_3_buff->Carrier[sig_index] = (r[i] + cp[j]) / wl;
			}

			/* Doppler [Hz] */
            if (r && rrf && rrf[j] > -1E12 && wl > 0.0)
			{
				rtcm_3_3_buff->Doppler[sig_index] = -(r[i] + rrf[j]) / wl; // negative sign here, as an approaching satellite should be positive for Doppler, but the transmitted phaserangerate is negative in this case
			}

			rtcm_3_3_buff->locktime[sig_index] = lock[j];
            try {
                //int lli=(!lock&&!rtcm->lock[sat-1][freq])||lock<rtcm->lock[sat-1][freq];

                rtcm_3_3_buff->lli[sig_index] = (!lock[j]&&!obs_locks[sv_index][sig_index])|| (lock[j]<obs_locks[sv_index][sig_index]) +(half[j]?3:0);
                obs_locks[sv_index][sig_index]=(unsigned short)lock[j];
            } catch (std::exception &ex)
            {
                std::cout << "start parsing2222" << std::endl;
                rtcm_3_3_buff->lli[sig_index] = 0;
            }

			/* C_N0 [0.25 dbHz] */
			rtcm_3_3_buff->CN0[sig_index] = static_cast<double>(cnr[j]) * 4.0; // times 4 to correspond to SBF format with unit 0.25 dbHz
			j++;

		} // loop over signals
	} // loop over sats
	//std::cout << "RTCM3_3_BUFF: "<<rtcm_3_3_buff << std::endl;
	
	/*for (size_t im= 0; im < 40; im++)
	{
		std::cout<<"Index_L1E1: "<<Index_L1E1.get()[im] <<std::endl;
	}
	for (size_t iv = 0; iv < 40; iv++)
	{
		std::cout<<"SVID: "<<int(rtcm_3_3_buff->SVID[iv])<<std::endl;
	}*/

}

// Decode MSM header
int RtcmParser::decodeMSMheader(const uint8_t* msgPtr, size_t msgLength, uint8_t sys, RTCM_MSM_h_t &head)
{
	bool mask;
	int ncell = 0;

	// minimum length check
	if (msgLength*8 < RTCM_MSM_HEADER_SIZE)
	{
		displayErrorMsg("size of msg too small for MSM header");
		return -1;
	}

	head.Message_Number          	= static_cast<uint16_t>(decodeUnsignedInteger(msgPtr, 0, 12));
	head.Reference_Station_ID 	= static_cast<uint16_t>(decodeUnsignedInteger(msgPtr,    12, 12));

	// the epoch time is system dependent. For Galileo and GPS, this is simply
	if (sys == SYS_GAL || sys == SYS_GPS)
		head.GNSS_Epoche_Time_ms = static_cast<uint32_t>(decodeUnsignedInteger(msgPtr,   24, 30));
	else
		head.GNSS_Epoche_Time_ms = 0;

	// Bit 55 is the multiple message bit
	head.multiple_message	= static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,         54, 1));

	head.iod		= static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,                 55, 3));
	// Bits 58-64 are reserved, not used
	head.clk_str		= static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,             65, 2));
	head.clk_ext		= static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,             67, 2));
	head.smooth		= static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,                 69, 1));
	head.tint_s		= static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,                 70, 3));

	// Satellite Mask
	for (unsigned int j = 0; j < 64; j++)
	{
	     mask = static_cast<bool>(decodeUnsignedInteger(msgPtr, 73 + j, 1));
	     if (mask)
	     {
	    	 head.sats[head.nsat++] = j + 1;
	     }
	}

	// Signal Mask
	for (unsigned int j = 0; j < 32; j++)
	{
		 mask = static_cast<bool>(decodeUnsignedInteger(msgPtr, 73 + 64 + j, 1));
		 if (mask)
		 {
			 head.sigs[head.nsig++] = j + 1;
		 }
	}

	// Cell Mask
	if (head.nsat * head.nsig > 64)
	{
		displayErrorMsg("Error: too many sats / signals decoded, aborting...");
		return 0;
	}

	// check length including cell mask
	if (msgLength*8 < (RTCM_MSM_HEADER_SIZE + head.nsat * head.nsig))
	{
		displayErrorMsg("size of msg too small for cell mask");
		return -1;
	}

	for (unsigned int j = 0; j < head.nsat * head.nsig; j++)
	{
		 head.cellmask[j] = static_cast<bool>(decodeUnsignedInteger(msgPtr, 73 + 64 + 32 + j, 1));
		 if (head.cellmask[j])
		 {
			 ncell++;
		 }
	}

	return ncell;
}

/// Decode Message of type 1004 and trigger signal call for all subscribers of this message type
void RtcmParser::decodeMsg1004(uint8_t* msgPtr, size_t msgLength){
	boost::shared_ptr<RTCM_30_1004_t> msg(new RTCM_30_1004_t);

	// Header
	msg->Message_Number                          = static_cast<uint16_t>(1004);
	msg->Reference_Station_ID                    = static_cast<uint16_t>(decodeUnsignedInteger(msgPtr, 12, 12));
  	msg->GPS_Epoche_Time_ms                      = static_cast<uint32_t>(decodeUnsignedInteger(msgPtr, 24, 30));
  	msg->Synchronous_GNSS_Flag                   = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 54,  1));
  	msg->No_of_GPS_Satellite_Signals_Processed   = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 55,  5));
  	msg->GPS_Divergence_free_Smoothing_Indicator = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 60,  1));
  	msg->GPS_Smoothing_Interval 	         = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 61,  3));
	
	// sanity check
	if (msg->No_of_GPS_Satellite_Signals_Processed > (sizeof(msg->GPS_Satellite_ID) / sizeof(msg->GPS_Satellite_ID[0])))
	{
		msg->No_of_GPS_Satellite_Signals_Processed = sizeof(msg->GPS_Satellite_ID) / sizeof(msg->GPS_Satellite_ID[0]);
	}

  	// Per Satellite Info
  	for(uint8_t i = 0; i < msg->No_of_GPS_Satellite_Signals_Processed; i++){
  		msg->GPS_Satellite_ID[i] 		 = static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,6));
  		msg->GPS_L1_Code_Indicator[i] 		 = static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,6+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,1));
  		msg->GPS_L1_Pseudorange_mod1lightms_m[i]	 = static_cast<double>( decodeUnsignedInteger(msgPtr,   7+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS, 24))*0.02;
  		msg->GPS_L1_PhaseRange_L1_Pseudorange_m[i] 	 = static_cast<double>( decodeSignedInteger(  msgPtr,  31+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS, 20))*0.0005;
  		msg->GPS_L1_Lock_time_Indicator[i] 		 = static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,  51+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,  7));
  		msg->GPS_Integer_L1_Pseudorange_Modulus_Ambiguity_m[i] = static_cast<double>( decodeUnsignedInteger(msgPtr,  58+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,  8))*299792.458;
  		msg->GPS_L1_CNR_dBHz[i] 		 = static_cast<double>( decodeUnsignedInteger(msgPtr,  66+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,  8))*0.25;
  		msg->GPS_L2_Code_Indicator[i]		 = static_cast<uint8_t>(decodeUnsignedInteger(msgPtr,  74+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,  2));
  		msg->GPS_L2_L1_Pseudorange_Difference_m[i]	 = static_cast<double>( decodeSignedInteger(  msgPtr,  76+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS, 14))*0.02;
  		msg->GPS_L2_PhaseRange_L1_Pseudorange_m[i]	 = static_cast<double>( decodeSignedInteger(  msgPtr,  90+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS, 20))*0.0005;
  		msg->GPS_L2_Lock_time_Indicator[i] 		 = static_cast<uint8_t>(decodeUnsignedInteger(msgPtr, 110+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,  7));
  		msg->GPS_L2_CNR_dbHz[i]		 = static_cast<double>( decodeUnsignedInteger(msgPtr, 117+i*RTCM_1004_PER_SAT_SIZE_BITS+RTCM_1004_HEADER_SIZE_BITS,  8))*0.25;
  	}
  	
  	msg->message_counter = static_cast<uint32_t>(m_stats.numOfMsgs[1004]);

	std::cout << "Function 1004_Decode is called!"<< std::endl;

	m_msg1004Signal(msg);
}

/// Decode Message of type 1005 and trigger signal call for all subscribers of this message type
void RtcmParser::decodeMsg1005(uint8_t* msgPtr, size_t msgLength){
	boost::shared_ptr<RTCM_30_1005_t> msg(new RTCM_30_1005_t);

	msg->Message_Number 	                  = static_cast<uint16_t>(1005);
	msg->Reference_Station_ID 	              = static_cast<uint16_t>(decodeUnsignedInteger(msgPtr, 12,  12));
	msg->Reserved_for_ITRF_Realization_Year   = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 24,   6));
	msg->GPS_Indicator 	                      = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 30,   1));
	msg->GLONASS_Indicator 	                  = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 31,   1));
	msg->Reserved_for_Galileo_Indicator       = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 32,   1));
	msg->Reference_Station_Indicator 	      = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 33,   1));
	msg->Antenna_Reference_Point_ECEF_X_m     = static_cast<double>(  decodeSignedInteger64(msgPtr, 34,  38))*0.0001;
	msg->Single_Receiver_Oscillator_Indicator = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 72,   1));
	msg->Reserved1 		                      = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 73,   1));
	msg->Antenna_Reference_Point_ECEF_Y_m     = static_cast<double>(  decodeSignedInteger64(msgPtr, 74,  38))*0.0001;
	msg->Reserved2	                          = static_cast<uint8_t>( decodeUnsignedInteger(msgPtr, 112,  2));
	msg->Antenna_Reference_Point_ECEF_Z_m     = static_cast<double>(  decodeSignedInteger64(msgPtr, 114, 38))*0.0001;

	msg->message_counter = static_cast<uint32_t>(m_stats.numOfMsgs[1005]);

	//std::cout << "Function 1005_Decode is called!"<< std::endl;

	m_msg1005Signal(msg);
}

/// Decode Message of type 1006
void RtcmParser::decodeMsg1006(uint8_t* msgPtr, size_t msgLength){

	// at this point, a 1006 msg is treated as a 1005 msg, as the only difference is the transmitted antenna height. This height is
	// not required for differential, as only the ARP is of interest, and not the coordinates of the actual base station.

	decodeMsg1005(msgPtr, msgLength);

	// print out antenna height for debug
	//std::cout << static_cast<double>( decodeUnsignedInteger(msgPtr, 152,  16))*0.0001 << std::endl;
}

void RtcmParser::add1005ToStorage(const boost::shared_ptr<RTCM_30_1005_t>& base)
{
	if (rtcm_3_3_buff.use_count())
	{
		rtcm_3_3_buff->Reference_Station_ID = base->Reference_Station_ID;
		rtcm_3_3_buff->base[0] = base->Antenna_Reference_Point_ECEF_X_m;
		rtcm_3_3_buff->base[1] = base->Antenna_Reference_Point_ECEF_Y_m;
		rtcm_3_3_buff->base[2] = base->Antenna_Reference_Point_ECEF_Z_m;
	}
}

void RtcmParser::reset_rtcm_buffer()
{
	rtcm_3_3_buff.reset(new RTCM_3_3_t()); // brackets are required here in order to init with zeros
	for (unsigned char & i : rtcm_3_3_buff->Type)
		i = 255;
}

unsigned int RtcmParser::getSVID_index(uint8_t svid)
{
	for (unsigned int i = 0; i < RTCM_MAX_SATS; i++)
	{
		// find first empty satellite or find already pre-used satellite
		if (rtcm_3_3_buff->SVID[i] == svid || rtcm_3_3_buff->SVID[i] == 0)
			return i;
	}

	// default do not use value
	return RTCM_MAX_SATS;
}

unsigned int RtcmParser::getnextfree_index(uint8_t svid_index)
{		
	for (unsigned int i = 0; i < RTCM_MAX_FREQS; i++)
	{
		if (rtcm_3_3_buff->Pseudorange[svid_index*RTCM_MAX_FREQS + i] < 0.000001)
			return svid_index*RTCM_MAX_FREQS + i;
	}

	// default do not use value
	return RTCM_MAX_SATS * RTCM_MAX_FREQS;
}


/// Compute 24bit QualComm CRC via magic numbers

static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

unsigned int RtcmParser::crc24q(const uint8_t* buffer, size_t length)
{
    unsigned int crc=0;    
    for (size_t i=0;i<length;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buffer[i]];
    return crc;
}

void RtcmParser::displayErrorMsg(const std::string& errorMsg)
{
	char thetime[32] = "---";
	time_t tt = time(nullptr);
	struct tm *ptm = localtime(&tt);
	if (ptm != nullptr)
	{
		strftime(thetime, sizeof thetime, "%Y/%m/%d %H:%M:%S", ptm);
	}
	std::cerr << std::string(thetime) << " | RtcmParser error: " << errorMsg << std::endl;
}

uint8_t RtcmParser::getSBF_SVID(uint8_t svid, uint8_t sys)
{
	switch (sys)
	{
		case SYS_GPS:
			return svid + SVID_OFFSET_GPS;
		case SYS_GLO:
			return svid + SVID_OFFSET_GLO;		
		case SYS_GAL:
			return svid + SVID_OFFSET_GAL;
	}

	// default value: do not use value
	return 0;
}

uint8_t RtcmParser::getSBF_Type(uint8_t type, uint8_t sys)
{
	switch (sys)
	{
		case SYS_GPS:
			switch (type)
			{
				case 2:  return 0;
				case 4:  return 1;
				case 10: return 2;
				case 16: return 3;
				case 23: return 4;
			}
			break;

		case SYS_GAL:
			switch (type)
			{
				case 2:  return 17;
				case 15: return 21;
				case 19: return 22;
				case 23: return 20;
			}
			break;
	}

	// default value: do not use value
	return 255;
}

double RtcmParser::get_wavelength(uint8_t type)
{
	switch (type)
	{
		case 2:
		case 4:
			return LAMBDA_1;
			break;

		case 10:
		case 16:
			return LAMBDA_2;
			break;

		case 23:
			return LAMBDA_5;
			break;

		case 15:
			return LAMBDA_7;
			break;

		case 19:
			return LAMBDA_8;
			break;
	}

	// default value: do not use value
	return 0.0;
}

/// Bitwise decode for RTCM Datafields
/// Taken from libswiftnav @ https://github.com/swift-nav/libswiftnav/blob/master/src/bits.c and adapted for 64bit

unsigned int RtcmParser::decodeUnsignedInteger(const uint8_t* bufferPtr, unsigned int pos, uint8_t length){
	unsigned int bits = 0;
	for(unsigned int i = pos; i<pos+length; i++){
		bits = (bits<<1) + ((bufferPtr[i/8] >> (7-i%8)) & 1u);
	}
	return bits;
}

int RtcmParser::decodeSignedInteger(const uint8_t* bufferPtr, unsigned int pos, uint8_t length){
	int bits = static_cast<int>(decodeUnsignedInteger(bufferPtr, pos, length));
	int mask = 1u << (length - 1);
	return (bits ^ mask) - mask;
}

uint64_t RtcmParser::decodeUnsignedInteger64(const uint8_t* bufferPtr, unsigned int pos, uint8_t length){
	uint64_t bits = 0;
	for(unsigned int i = pos; i<pos+length; i++){
		bits = (bits<<1) + + ((bufferPtr[i/8] >> (7-i%8)) & 1u);
	}
	return bits;
}

int64_t RtcmParser::decodeSignedInteger64(const uint8_t* bufferPtr, unsigned int pos, uint8_t length){
	auto bits = static_cast<int64_t>(decodeUnsignedInteger64(bufferPtr, pos, length));
	int64_t mask = 1u << (length - 1);
	return (bits ^ mask) - mask;
}
