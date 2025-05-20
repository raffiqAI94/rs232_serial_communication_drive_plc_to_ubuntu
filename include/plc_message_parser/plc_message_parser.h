#ifndef PLC_MESSAGE_PARSER_H
#define PLC_MESSAGE_PARSER_H

#include <string>
#include <plc_message_parser/Heartbeat.h>
#include <plc_message_parser/PickPlace.h>
#include <plc_message_parser/Acknowledgement.h>
#include <plc_message_parser/JobTwistlock.h>

// Function declarations for parsing each message type

bool parseHeartbeat(const std::string& raw_message, plc_message_parser::Heartbeat& hb_msg);
bool parsePickPlace(const std::string& raw_message, plc_message_parser::PickPlace& pp_msg);
bool parseAcknowledgement(const std::string& raw_message, plc_message_parser::Acknowledgement& ack_msg);
bool parseJobTwistlock(const std::string& raw_message, plc_message_parser::JobTwistlock& jt_msg);

#endif

