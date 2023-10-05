#ifndef _CONFIG_H_
#define _CONFIG_H_

#pragma once

#pragma pack(push) // push current alignment to stack
#pragma pack(1)    // set alignment boundary to 1 byte
#include <map>
#include <iostream>
#include <iomanip>
#include <cstdint>


const int Length_1004 = 748;
const int Length_DD_4_1004 = 753; 

const int Length_1005 = 40;
const int Length_DD_4_1005 = 45;

const int Length_RTCM_3_3 = 7074;
const int Length_RTCM_3_3_L1E1 = 1154;
const int Length_DD_RTCM_3_3_L1E1 = 1159;

#pragma pack(pop) // zurück zur alten Fragmentierung

#endif