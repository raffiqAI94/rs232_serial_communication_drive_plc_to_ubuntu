#include <ros/ros.h>
#include <std_msgs/String.h>
#include <plc_message_parser/Heartbeat.h>
#include <plc_message_parser/PickPlace.h>
#include <plc_message_parser/Acknowledgement.h>
#include <plc_message_parser/JobTwistlock.h>
#include <plc_message_parser/plc_message_parser.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <algorithm>

// --- Helper Functions ---

unsigned int asciiHexToInt(const std::string& asciiHexStr) {
    if (asciiHexStr.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos) {
         throw std::invalid_argument("Invalid character in AsciiHex string: " + asciiHexStr);
    }
    if (asciiHexStr.empty()) {
        throw std::invalid_argument("Empty AsciiHex string provided.");
    }
    return std::stoul(asciiHexStr, nullptr, 16);
}

std::string calculateAsciiHexCRC(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        ROS_ERROR("Empty data for CRC calculation");
        return "0000";
    }
    uint16_t sum = 0;
    for (uint8_t byte : data) {
        sum += byte;
    }
    uint16_t crc = (~sum) + 1;
    std::stringstream ss;
    ss << std::hex << std::setw(4) << std::setfill('0') << std::uppercase << crc;
    return ss.str();
}

std::string safeSubstrTrim(const std::string& s, size_t pos, size_t len) {
    if (pos >= s.length()) {
        ROS_WARN("Substring start position %zu out of bounds (length %zu)", pos, s.length());
        return "";
    }
    if (pos + len > s.length()) {
        len = s.length() - pos;
    }
    std::string sub = s.substr(pos, len);
    sub.erase(0, sub.find_first_not_of(' '));
    sub.erase(sub.find_last_not_of(' ') + 1);
    return sub;
}

// --- Parsing Functions ---

bool parseHeartbeat(const std::string& payload, plc_message_parser::Heartbeat& hb_msg) {
    if (payload.length() < 74) {
        ROS_WARN_STREAM("Heartbeat payload too short: " << payload.length() << ", expected at least 74. Payload: " << payload);
        return false;
    }
    try {
        hb_msg.msg_id = payload.substr(0, 2);
        if (hb_msg.msg_id != "A1") {
            ROS_DEBUG_STREAM("Not a Heartbeat message (A1): ID=" << hb_msg.msg_id);
            return false;
        }
        hb_msg.version = safeSubstrTrim(payload, 2, 6);
        hb_msg.time = safeSubstrTrim(payload, 8, 6);
        hb_msg.status = safeSubstrTrim(payload, 14, 4);
        hb_msg.block_name = safeSubstrTrim(payload, 18, 8);
        hb_msg.bay_number = safeSubstrTrim(payload, 26, 3);
        hb_msg.row_number = safeSubstrTrim(payload, 29, 3);
        hb_msg.height = safeSubstrTrim(payload, 32, 4);
        hb_msg.crane_status = safeSubstrTrim(payload, 36, 2);
        hb_msg.gps_status = safeSubstrTrim(payload, 38, 2);
        hb_msg.che = safeSubstrTrim(payload, 40, 5);
        hb_msg.system_status = safeSubstrTrim(payload, 45, 6);
        hb_msg.plc_comm_status = safeSubstrTrim(payload, 51, 2);
        hb_msg.pos_x = safeSubstrTrim(payload, 53, 9);
        hb_msg.pos_y = safeSubstrTrim(payload, 62, 9);
        hb_msg.len = safeSubstrTrim(payload, 71, 2);
        hb_msg.twist_lock = safeSubstrTrim(payload, 73, 1);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Parse Heartbeat failed: " << e.what() << ", payload: " << payload);
        return false;
    }
}

bool parsePickPlace(const std::string& payload, plc_message_parser::PickPlace& pp_msg) {
    if (payload.length() < 118) {
        ROS_WARN_STREAM("PickPlace payload too short: " << payload.length() << ", expected at least 118. Payload: " << payload);
        return false; // Re-enabled length check
    }
    try {
        pp_msg.msg_id = payload.substr(0, 2);
        if (pp_msg.msg_id != "A2") {
            ROS_DEBUG_STREAM("Not a PickPlace message (A2): ID=" << pp_msg.msg_id);
            return false;
        }
        pp_msg.version = safeSubstrTrim(payload, 2, 6);
        pp_msg.counter = asciiHexToInt(safeSubstrTrim(payload, 8, 2));
        pp_msg.move_type = safeSubstrTrim(payload, 10, 2);
        pp_msg.time = safeSubstrTrim(payload, 12, 14);
        pp_msg.status = safeSubstrTrim(payload, 26, 5);
        pp_msg.che = safeSubstrTrim(payload, 31, 5);
        pp_msg.cont_weight = safeSubstrTrim(payload, 36, 5);
        pp_msg.pos_on_truck = safeSubstrTrim(payload, 41, 2);
        pp_msg.cont_id1 = safeSubstrTrim(payload, 48, 12);
        pp_msg.cont_id2 = safeSubstrTrim(payload, 60, 12);
        pp_msg.mount = safeSubstrTrim(payload, 72, 1);
        pp_msg.tag = safeSubstrTrim(payload, 73, 1);
        pp_msg.position1 = safeSubstrTrim(payload, 74, 8);
        pp_msg.bay_number = safeSubstrTrim(payload, 82, 3);
        pp_msg.row_number = safeSubstrTrim(payload, 85, 3);
        pp_msg.height = safeSubstrTrim(payload, 88, 4);
        pp_msg.position2 = safeSubstrTrim(payload, 92, 8);
        pp_msg.bay_number2 = safeSubstrTrim(payload, 100, 3);
        pp_msg.row_number2 = safeSubstrTrim(payload, 103, 3);
        pp_msg.height2 = safeSubstrTrim(payload, 106, 4);
        pp_msg.len = safeSubstrTrim(payload, 110, 2);
        pp_msg.system_status = safeSubstrTrim(payload, 112, 6);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Parse PickPlace failed: " << e.what() << ", payload: " << payload);
        return false;
    }
}

bool parseAcknowledgement(const std::string& payload, plc_message_parser::Acknowledgement& ack_msg) {
    if (payload.length() < 4) {
        ROS_WARN_STREAM("Acknowledgement payload too short: " << payload.length() << ", expected at least 4. Payload: " << payload);
        return false;
    }
    try {
        ack_msg.msg_id = payload.substr(0, 2);
        if (ack_msg.msg_id != "A3") {
            ROS_DEBUG_STREAM("Not an Acknowledgement message (A3): ID=" << ack_msg.msg_id);
            return false;
        }
        ack_msg.counter = asciiHexToInt(safeSubstrTrim(payload, 2, 2));
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Parse Acknowledgement failed: " << e.what() << ", payload: " << payload);
        return false;
    }
}

bool parseJobTwistlock(const std::string& payload, plc_message_parser::JobTwistlock& jt_msg) {
    if (payload.length() < 118) {
        ROS_WARN_STREAM("JobTwistlock payload too short: " << payload.length() << ", expected at least 118. Payload: " << payload);
        return false; // Re-enabled length check
    }
    try {
        jt_msg.msg_id = payload.substr(0, 2);
        if (jt_msg.msg_id != "A4") {
            ROS_DEBUG_STREAM("Not a JobTwistlock message (A4): ID=" << jt_msg.msg_id);
            return false;
        }
        jt_msg.version = safeSubstrTrim(payload, 2, 6);
        jt_msg.counter = asciiHexToInt(safeSubstrTrim(payload, 8, 2));
        jt_msg.move_type = safeSubstrTrim(payload, 10, 2);
        jt_msg.time = safeSubstrTrim(payload, 12, 14);
        jt_msg.status = safeSubstrTrim(payload, 26, 5);
        jt_msg.che = safeSubstrTrim(payload, 31, 5);
        jt_msg.chassis = safeSubstrTrim(payload, 36, 12);
        jt_msg.cont_id1 = safeSubstrTrim(payload, 48, 12);
        jt_msg.cont_id2 = safeSubstrTrim(payload, 60, 12);
        jt_msg.mount = safeSubstrTrim(payload, 72, 1);
        jt_msg.tag = safeSubstrTrim(payload, 73, 1);
        jt_msg.position1 = safeSubstrTrim(payload, 74, 8);
        jt_msg.bay_number = safeSubstrTrim(payload, 82, 3);
        jt_msg.row_number = safeSubstrTrim(payload, 85, 3);
        jt_msg.height = safeSubstrTrim(payload, 88, 4);
        jt_msg.position2 = safeSubstrTrim(payload, 92, 8);
        jt_msg.bay_number2 = safeSubstrTrim(payload, 100, 3);
        jt_msg.row_number2 = safeSubstrTrim(payload, 103, 3);
        jt_msg.height2 = safeSubstrTrim(payload, 106, 4);
        jt_msg.len = safeSubstrTrim(payload, 110, 2);
        jt_msg.system_status = safeSubstrTrim(payload, 112, 6);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Parse JobTwistlock failed: " << e.what() << ", payload: " << payload);
        return false;
    }
}

// --- State Machine for Parsing ---
enum class ParserState {
    WAITING_FOR_MARKER,
    WAITING_FOR_LENGTH,
    WAITING_FOR_PAYLOAD_CRC
};

/*// --- Main Function ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "plc_message_parser_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Serial port parameters
    std::string serial_port_name;
    int serial_baudrate;
    double serial_timeout_sec;
    int serial_databits, serial_stopbits;
    std::string serial_parity_str;

    if (!pnh.getParam("serial_port", serial_port_name)) {
        ROS_ERROR("Failed to get serial_port parameter");
        return -1;
    }
    if (!pnh.getParam("baud_rate", serial_baudrate)) {
        ROS_ERROR("Failed to get baud_rate parameter");
        return -1;
    }
    pnh.param("timeout", serial_timeout_sec, 1.0);
    pnh.param("Databits", serial_databits, 8);
    pnh.param("Stopbits", serial_stopbits, 1);
    pnh.param("parity", serial_parity_str, std::string("none"));

    serial::parity_t parity = serial::parity_none;
    if (serial_parity_str == "odd") parity = serial::parity_odd;
    else if (serial_parity_str == "even") parity = serial::parity_even;

    serial::stopbits_t stopbits = serial::stopbits_one;
    if (serial_stopbits == 2) stopbits = serial::stopbits_two;

    serial::bytesize_t bytesize = serial::eightbits;
    if (serial_databits == 7) bytesize = serial::sevenbits;

    // Initialize serial port
    serial::Serial ser;
    try {
        ser.setPort(serial_port_name);
        ser.setBaudrate(serial_baudrate);
        uint32_t timeout_ms = static_cast<uint32_t>(serial_timeout_sec * 1000);
        ser.setTimeout(timeout_ms, timeout_ms, 0, timeout_ms, 0);
        ser.setParity(parity);
        ser.setStopbits(stopbits);
        ser.setBytesize(bytesize);
        ser.open();
    } catch (const serial::SerialException& e) {
        ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
        return -1;
    }
    if (!ser.isOpen()) {
        ROS_ERROR("Serial port not open");
        return -1;
    }
    ROS_INFO_STREAM("Serial Port " << serial_port_name << " initialized...");

    // Publishers
    ros::Publisher heartbeat_pub = nh.advertise<plc_message_parser::Heartbeat>("heartbeat", 10);
    ros::Publisher pick_place_pub = nh.advertise<plc_message_parser::PickPlace>("pick_place", 10);
    ros::Publisher ack_pub = nh.advertise<plc_message_parser::Acknowledgement>("ack", 10);
    ros::Publisher job_twistlock_pub = nh.advertise<plc_message_parser::JobTwistlock>("job_twistlock", 10);

    ros::Rate loop_rate(100);
    std::vector<uint8_t> buffer;
    const size_t MAX_BUFFER_SIZE = 2048;

    ParserState current_state = ParserState::WAITING_FOR_MARKER;
    size_t expected_payload_length = 0;
    std::string current_length_ascii_hex = "";

    while (ros::ok()) {
        // Read Data
        size_t bytes_available = 0;
        try {
            bytes_available = ser.available();
        } catch (const serial::IOException& e) {
            ROS_ERROR_STREAM("Serial read error: " << e.what());
            continue;
        }

        if (bytes_available > 0) {
            ROS_INFO_STREAM("Attempting to read " << bytes_available << " bytes.");
            std::vector<uint8_t> received_data;
            try {
                size_t bytes_read = ser.read(received_data, bytes_available);
                ROS_INFO_STREAM("Successfully read " << bytes_read << " bytes.");
                if (bytes_read > 0) {
                    if (buffer.size() + bytes_read > MAX_BUFFER_SIZE) {
                        ROS_WARN("Buffer size exceeds MAX_BUFFER_SIZE, clearing buffer");
                        buffer.clear();
                    }
                    buffer.insert(buffer.end(), received_data.begin(), received_data.end());
                    std::stringstream ss_hex;
                    for (size_t i = 0; i < bytes_read; ++i) {
                        ss_hex << std::hex << std::setw(2) << std::setfill('0') << (int)received_data[i] << " ";
                    }
                    ROS_INFO_STREAM("Received Hex: " << ss_hex.str());
                }
            } catch (const serial::IOException& e) {
                ROS_ERROR_STREAM("Serial read error: " << e.what());
            }
        }

        // Process Buffer Based on State
        bool state_changed = true;
        while (ros::ok() && state_changed && !buffer.empty()) {
            state_changed = false;

            switch (current_state) {

                case ParserState::WAITING_FOR_MARKER: {
                    ROS_DEBUG("State: WAITING_FOR_MARKER");
                    // Added debug log to show buffer contents before searching for marker
                    std::stringstream buffer_ss;
                    for (size_t i = 0; i < buffer.size(); i++) {
                        buffer_ss << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
                    }
                    ROS_INFO_STREAM("Buffer in WAITING_FOR_MARKER (size=" << buffer.size() << "): " << buffer_ss.str());

                    size_t start_marker_pos = std::string::npos;
                    for (size_t i = 0; (i + 1) < buffer.size(); ++i) {
                        if (buffer[i] == 0xFF && buffer[i + 1] == 0xFF) {
                            start_marker_pos = i;
                            break;
                        }
                    }

                    if (start_marker_pos != std::string::npos) {
                        if (start_marker_pos > 0) {
                            buffer.erase(buffer.begin(), buffer.begin() + start_marker_pos);
                            ROS_INFO_STREAM("Found FF FF marker. Discarded " << start_marker_pos << " bytes before it.");
                        }
                        if (buffer.size() >= 4) {
                            ROS_INFO("Found FF FF marker and potential XX bytes.");
                            buffer.erase(buffer.begin(), buffer.begin() + 2);
                            ROS_INFO_STREAM("Removed FF FF marker. Changing state to WAITING_FOR_LENGTH. Buffer size: " << buffer.size());
                            current_state = ParserState::WAITING_FOR_LENGTH;
                            state_changed = true;
                        } else {
                            ROS_INFO("Found FF FF marker, but not enough data for length yet. Waiting.");
                        }
                    } else {
                        ROS_INFO_STREAM("No FF FF found in buffer. State WAITING_FOR_MARKER. Buffer size: " << buffer.size());
                        if (buffer.size() > MAX_BUFFER_SIZE / 2) {
                            ROS_WARN("Buffer size exceeds half of MAX_BUFFER_SIZE, clearing buffer");
                            buffer.clear();
                        }
                    }
                    break;
                }

                case ParserState::WAITING_FOR_LENGTH: {
                    ROS_DEBUG("State: WAITING_FOR_LENGTH");
                    if (buffer.size() < 2) {
                        ROS_INFO("WAITING_FOR_LENGTH: Buffer too small (%zu). Waiting.", buffer.size());
                        break;
                    }
                    ROS_INFO("Potential XX bytes (Hex): %02x %02x (ASCII: %c%c)", buffer[0], buffer[1], isprint(buffer[0]) ? buffer[0] : '.', isprint(buffer[1]) ? buffer[1] : '.');
                    current_length_ascii_hex = "";
                    unsigned int parsed_length = 0;
                    try {
                        current_length_ascii_hex.push_back(static_cast<char>(buffer[0]));
                        current_length_ascii_hex.push_back(static_cast<char>(buffer[1]));
                        ROS_INFO_STREAM("Attempting to parse length: " << current_length_ascii_hex);
                        parsed_length = asciiHexToInt(current_length_ascii_hex);
                        if (parsed_length < 4 || parsed_length > 1000) {
                            throw std::out_of_range("Decoded payload length out of range: " + std::to_string(parsed_length));
                        }
                        expected_payload_length = parsed_length;
                        ROS_INFO_STREAM("Length OK (" << expected_payload_length << " from XX=" << current_length_ascii_hex << "). Changing state to WAITING_FOR_PAYLOAD_CRC.");
                        buffer.erase(buffer.begin(), buffer.begin() + 2);
                        current_state = ParserState::WAITING_FOR_PAYLOAD_CRC;
                        state_changed = true;
                    } catch (const std::exception &e) {
                        ROS_WARN_STREAM("Failed to parse AsciiHex length '" << current_length_ascii_hex << "': " << e.what() << ". Resetting state to WAITING_FOR_MARKER.");
                        buffer.erase(buffer.begin());
                        current_state = ParserState::WAITING_FOR_MARKER;
                        state_changed = true;
                    }
                    break;
                }

                case ParserState::WAITING_FOR_PAYLOAD_CRC: {
                    ROS_DEBUG("State: WAITING_FOR_PAYLOAD_CRC");
                    size_t expected_remaining_length = expected_payload_length + 4;
                    ROS_INFO_STREAM("Checking remaining buffer size (" << buffer.size() << ") against expected (payload + CRC = " << expected_remaining_length << ")");
                    if (buffer.size() < expected_remaining_length) {
                        ROS_INFO("Buffer too small for payload+CRC. State WAITING_FOR_PAYLOAD_CRC. Waiting.");
                        break;
                    }

                    std::stringstream buffer_ss;
                    for (size_t i = 0; i < buffer.size(); i++) {
                        buffer_ss << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i];
                        if (i < buffer.size() - 1) buffer_ss << " ";
                    }
                    ROS_INFO_STREAM("Full buffer before CRC check (size=" << buffer.size() << "): " << buffer_ss.str());

                    std::string payload_str(buffer.begin(), buffer.begin() + expected_payload_length);
                    std::string msg_id = payload_str.substr(0, 2);
                    ROS_INFO_STREAM("Received message with ID: " << msg_id << ", payload: " << payload_str);

                    std::string calculated_crc_ascii_hex = "ERROR";
                    std::string received_crc_ascii_hex = "ERROR";
                    bool crc_ok = false;
                    bool crc_format_valid = true;

                    try {
                        std::vector<uint8_t> data_for_crc;
                        data_for_crc.push_back(static_cast<uint8_t>(current_length_ascii_hex[0]));
                        data_for_crc.push_back(static_cast<uint8_t>(current_length_ascii_hex[1]));
                        data_for_crc.insert(data_for_crc.end(), buffer.begin(), buffer.begin() + expected_payload_length);

                        std::stringstream crc_data_ss;
                        for (auto b : data_for_crc) {
                            crc_data_ss << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                        }
                        ROS_INFO_STREAM("Data for CRC calculation: " << crc_data_ss.str());

                        calculated_crc_ascii_hex = calculateAsciiHexCRC(data_for_crc);

                        received_crc_ascii_hex = "";
                        for (size_t i = 0; i < 4; i++) {
                            size_t crc_index = expected_payload_length + i;
                            if (crc_index >= buffer.size()) {
                                throw std::runtime_error("Buffer too short to extract CRC at index " + std::to_string(crc_index));
                            }
                            char c = static_cast<char>(buffer[crc_index]);
                            if (!isxdigit(c)) {
                                std::stringstream ss;
                                ss << std::hex << std::setw(2) << std::setfill('0') << (int)c;
                                ROS_ERROR_STREAM("Invalid hex digit in received CRC at position " << i << ": " << ss.str());
                                crc_format_valid = false;
                                break;
                            }
                            received_crc_ascii_hex.push_back(c);
                        }

                        if (crc_format_valid) {
                            std::stringstream crc_bytes_ss;
                            for (size_t i = 0; i < 4; i++) {
                                crc_bytes_ss << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[expected_payload_length + i] << " ";
                            }
                            ROS_INFO_STREAM("Raw CRC bytes: " << crc_bytes_ss.str());
                        }

                        if (crc_format_valid) {
                            crc_ok = (received_crc_ascii_hex == calculated_crc_ascii_hex);
                            ROS_INFO_STREAM("CRC Check - Calculated: " << calculated_crc_ascii_hex 
                                            << " Received: " << received_crc_ascii_hex 
                                            << " Match: " << (crc_ok ? "YES" : "NO"));
                        } else {
                            ROS_WARN("CRC format invalid. Skipping CRC check and treating as mismatch.");
                        }
                    } catch (const std::exception& e) {
                        ROS_ERROR_STREAM("Exception during CRC check: " << e.what());
                        crc_ok = false;
                        crc_format_valid = false;
                    }

                    if (crc_ok) {
                        ROS_INFO_STREAM("CRC OK. Received: " << received_crc_ascii_hex << ". Parsing message.");

                        bool parsed_ok = false;
                        if (msg_id == "A1") {
                            plc_message_parser::Heartbeat hb_msg;
                            parsed_ok = parseHeartbeat(payload_str, hb_msg);
                            if (parsed_ok) heartbeat_pub.publish(hb_msg);
                        } else if (msg_id == "A2") {
                            plc_message_parser::PickPlace pp_msg;
                            parsed_ok = parsePickPlace(payload_str, pp_msg);
                            if (parsed_ok) pick_place_pub.publish(pp_msg);
                        } else if (msg_id == "A3") {
                            plc_message_parser::Acknowledgement ack_msg;
                            parsed_ok = parseAcknowledgement(payload_str, ack_msg);
                            if (parsed_ok) ack_pub.publish(ack_msg);
                        } else if (msg_id == "A4") {
                            plc_message_parser::JobTwistlock jt_msg;
                            parsed_ok = parseJobTwistlock(payload_str, jt_msg);
                            if (parsed_ok) job_twistlock_pub.publish(jt_msg);
                        } else {
                            ROS_WARN_STREAM("Unknown message ID: " << msg_id);
                        }

                        if (parsed_ok) {
                            ROS_INFO_STREAM("Successfully parsed and published message: " << msg_id);
                        } else {
                            ROS_WARN_STREAM("Failed to parse message with ID: " << msg_id);
                        }

                        size_t bytes_to_remove = expected_payload_length + 4;
                        buffer.erase(buffer.begin(), buffer.begin() + bytes_to_remove);
                        ROS_INFO_STREAM("Processed message. Removed " << bytes_to_remove << " bytes. Buffer size: " << buffer.size());
                    } else {
                        ROS_WARN_STREAM("CRC Mismatch or Invalid Format! Recv: " << received_crc_ascii_hex << ", Calc: " << calculated_crc_ascii_hex << ". Discarding entire message.");
                        size_t bytes_to_remove = std::min(buffer.size(), static_cast<size_t>(expected_payload_length + 4));
                        buffer.erase(buffer.begin(), buffer.begin() + bytes_to_remove);
                        ROS_INFO_STREAM("Removed " << bytes_to_remove << " bytes. Buffer size: " << buffer.size());
                    }

                    current_state = ParserState::WAITING_FOR_MARKER;
                    state_changed = true;
                    break;
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "plc_message_parser_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /*// Serial port parameters
    std::string serial_port_name;
    int serial_baudrate;
    double serial_timeout_sec;
    int serial_databits, serial_stopbits;
    std::string serial_parity_str;

    if (!pnh.getParam("serial_port", serial_port_name)) {
        ROS_ERROR("Failed to get serial_port parameter");
        return -1;
    }
    if (!pnh.getParam("baud_rate", serial_baudrate)) {
        ROS_ERROR("Failed to get baud_rate parameter");
        return -1;
    }
    pnh.param("timeout", serial_timeout_sec, 1.0);
    pnh.param("Databits", serial_databits, 8);
    pnh.param("Stopbits", serial_stopbits, 1);
    pnh.param("parity", serial_parity_str, std::string("none"));

    serial::parity_t parity = serial::parity_none;
    if (serial_parity_str == "odd") parity = serial::parity_odd;
    else if (serial_parity_str == "even") parity = serial::parity_even;

    serial::stopbits_t stopbits = serial::stopbits_one;
    if (serial_stopbits == 2) stopbits = serial::stopbits_two;

    serial::bytesize_t bytesize = serial::eightbits;
    if (serial_databits == 7) bytesize = serial::sevenbits;

    // Initialize serial port with retry mechanism
    serial::Serial ser;
    int max_retries = 5;
    int retry_delay_sec = 2;
    bool port_opened = false;

    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        try {
            ser.setPort(serial_port_name);
            ser.setBaudrate(serial_baudrate);
            uint32_t timeout_ms = static_cast<uint32_t>(serial_timeout_sec * 1000);
            ser.setTimeout(timeout_ms, timeout_ms, 0, timeout_ms, 0);
            ser.setParity(parity);
            ser.setStopbits(stopbits);
            ser.setBytesize(bytesize);
            ser.open();
            port_opened = true;
            ROS_INFO_STREAM("Serial Port " << serial_port_name << " initialized on attempt " << attempt);
            break;
        } catch (const serial::SerialException& e) {
            ROS_ERROR_STREAM("Failed to open serial port on attempt " << attempt << "/" << max_retries << ": " << e.what());
            if (attempt == max_retries) {
                ROS_ERROR("Max retries reached. Exiting.");
                return -1;
            }
            ROS_INFO_STREAM("Retrying in " << retry_delay_sec << " seconds...");
            ros::Duration(retry_delay_sec).sleep();
        }
    }

    if (!port_opened) {
        ROS_ERROR("Serial port not opened after retries. Exiting.");
        return -1;
    }

    */

   //ros::NodeHandle pnh("~");

   // ... (Parameter loading and Serial Port Initialization, Publisher setup - remain the same) ...
   std::string serial_port_name; int serial_baudrate; double serial_timeout_sec;
   int serial_databits; int serial_stopbits; std::string serial_parity_str;
   if (!pnh.getParam("serial_port", serial_port_name)) { /* ... */ return -1; }
   if (!pnh.getParam("baud_rate", serial_baudrate)) { /* ... */ return -1; }
   pnh.param("timeout", serial_timeout_sec, 1.0);
   pnh.param("Databits", serial_databits, 8);
   pnh.param("Stopbits", serial_stopbits, 1);
   pnh.param("parity", serial_parity_str, std::string("none"));
   serial::parity_t parity = serial::parity_none; /* ... conversion ... */
   serial::stopbits_t stopbits = serial::stopbits_one; /* ... conversion ... */
   serial::bytesize_t bytesize = serial::eightbits; /* ... conversion ... */
   serial::Serial ser;
   try {
       ser.setPort(serial_port_name); ser.setBaudrate(serial_baudrate);
       uint32_t timeout_ms = static_cast<uint32_t>(serial_timeout_sec * 1000);
       ser.setTimeout(timeout_ms, timeout_ms, 0, timeout_ms, 0);
       ser.setParity(parity); ser.setStopbits(stopbits); ser.setBytesize(bytesize);
       ser.open();
   } catch (...) { /* ... */ return -1; }
   if (!ser.isOpen()) { /* ... */ return -1; }
   ROS_INFO_STREAM("Serial Port " << serial_port_name << " initialized...");
    // Publishers
    ros::Publisher heartbeat_pub = nh.advertise<plc_message_parser::Heartbeat>("heartbeat", 10);
    ros::Publisher pick_place_pub = nh.advertise<plc_message_parser::PickPlace>("pick_place", 10);
    ros::Publisher ack_pub = nh.advertise<plc_message_parser::Acknowledgement>("ack", 10);
    ros::Publisher job_twistlock_pub = nh.advertise<plc_message_parser::JobTwistlock>("job_twistlock", 10);

    ros::Rate loop_rate(100);
    std::vector<uint8_t> buffer;
    const size_t MAX_BUFFER_SIZE = 2048;

    ParserState current_state = ParserState::WAITING_FOR_MARKER;
    size_t expected_payload_length = 0;
    std::string current_length_ascii_hex = "";

    ros::Time last_message_time = ros::Time::now();
    const double MESSAGE_TIMEOUT_SEC = 10.0;

    while (ros::ok()) {
        // Read Data
        size_t bytes_available = 0;
        try {
            bytes_available = ser.available();
        } catch (const serial::IOException& e) {
            ROS_ERROR_STREAM("Serial read error: " << e.what());
            continue;
        }

        if (bytes_available > 0) {
            ROS_INFO_STREAM("Attempting to read " << bytes_available << " bytes.");
            std::vector<uint8_t> received_data;
            try {
                size_t bytes_read = ser.read(received_data, bytes_available);
                ROS_INFO_STREAM("Successfully read " << bytes_read << " bytes.");
                if (bytes_read > 0) {
                    if (buffer.size() + bytes_read > MAX_BUFFER_SIZE) {
                        ROS_WARN("Buffer size exceeds MAX_BUFFER_SIZE, clearing buffer");
                        buffer.clear();
                    }
                    buffer.insert(buffer.end(), received_data.begin(), received_data.end());
                    std::stringstream ss_hex;
                    for (size_t i = 0; i < bytes_read; ++i) {
                        ss_hex << std::hex << std::setw(2) << std::setfill('0') << (int)received_data[i] << " ";
                    }
                    ROS_INFO_STREAM("Received Hex: " << ss_hex.str());
                }
            } catch (const serial::IOException& e) {
                ROS_ERROR_STREAM("Serial read error: " << e.what());
            }
        }

        // Process Buffer Based on State
        bool state_changed = true;
        while (ros::ok() && state_changed && !buffer.empty()) {
            state_changed = false;

            switch (current_state) {

                case ParserState::WAITING_FOR_MARKER: {
                    ROS_DEBUG("State: WAITING_FOR_MARKER");
                    std::stringstream buffer_ss;
                    for (size_t i = 0; i < buffer.size(); i++) {
                        buffer_ss << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
                    }
                    ROS_INFO_STREAM("Buffer in WAITING_FOR_MARKER (size=" << buffer.size() << "): " << buffer_ss.str());

                    size_t start_marker_pos = std::string::npos;
                    for (size_t i = 0; (i + 1) < buffer.size(); ++i) {
                        if (buffer[i] == 0xFF && buffer[i + 1] == 0xFF) {
                            start_marker_pos = i;
                            break;
                        }
                    }

                    if (start_marker_pos != std::string::npos) {
                        if (start_marker_pos > 0) {
                            buffer.erase(buffer.begin(), buffer.begin() + start_marker_pos);
                            ROS_INFO_STREAM("Found FF FF marker. Discarded " << start_marker_pos << " bytes before it.");
                        }
                        if (buffer.size() >= 4) {
                            ROS_INFO("Found FF FF marker and potential XX bytes.");
                            buffer.erase(buffer.begin(), buffer.begin() + 2);
                            ROS_INFO_STREAM("Removed FF FF marker. Changing state to WAITING_FOR_LENGTH. Buffer size: " << buffer.size());
                            current_state = ParserState::WAITING_FOR_LENGTH;
                            state_changed = true;
                        } else {
                            ROS_INFO("Found FF FF marker, but not enough data for length yet. Waiting.");
                        }
                    } else {
                        ROS_INFO_STREAM("No FF FF found in buffer. State WAITING_FOR_MARKER. Buffer size: " << buffer.size());
                        if (buffer.size() > MAX_BUFFER_SIZE / 2) {
                            ROS_WARN("Buffer size exceeds half of MAX_BUFFER_SIZE, clearing buffer");
                            buffer.clear();
                        }
                    }
                    break;
                }

                case ParserState::WAITING_FOR_LENGTH: {
                    ROS_DEBUG("State: WAITING_FOR_LENGTH");
                    if (buffer.size() < 2) {
                        ROS_INFO("WAITING_FOR_LENGTH: Buffer too small (%zu). Waiting.", buffer.size());
                        break;
                    }
                    ROS_INFO("Potential XX bytes (Hex): %02x %02x (ASCII: %c%c)", buffer[0], buffer[1], isprint(buffer[0]) ? buffer[0] : '.', isprint(buffer[1]) ? buffer[1] : '.');
                    current_length_ascii_hex = "";
                    unsigned int parsed_length = 0;
                    try {
                        current_length_ascii_hex.push_back(static_cast<char>(buffer[0]));
                        current_length_ascii_hex.push_back(static_cast<char>(buffer[1]));
                        ROS_INFO_STREAM("Attempting to parse length: " << current_length_ascii_hex);
                        parsed_length = asciiHexToInt(current_length_ascii_hex);
                        if (parsed_length < 4 || parsed_length > 1000) {
                            throw std::out_of_range("Decoded payload length out of range: " + std::to_string(parsed_length));
                        }
                        expected_payload_length = parsed_length;
                        ROS_INFO_STREAM("Length OK (" << expected_payload_length << " from XX=" << current_length_ascii_hex << "). Changing state to WAITING_FOR_PAYLOAD_CRC.");
                        buffer.erase(buffer.begin(), buffer.begin() + 2);
                        current_state = ParserState::WAITING_FOR_PAYLOAD_CRC;
                        state_changed = true;
                    } catch (const std::exception &e) {
                        ROS_WARN_STREAM("Failed to parse AsciiHex length '" << current_length_ascii_hex << "': " << e.what() << ". Resetting state to WAITING_FOR_MARKER.");
                        buffer.erase(buffer.begin());
                        current_state = ParserState::WAITING_FOR_MARKER;
                        state_changed = true;
                    }
                    break;
                }

                case ParserState::WAITING_FOR_PAYLOAD_CRC: {
                    ROS_DEBUG("State: WAITING_FOR_PAYLOAD_CRC");
                    size_t expected_remaining_length = expected_payload_length + 4;
                    ROS_INFO_STREAM("Checking remaining buffer size (" << buffer.size() << ") against expected (payload + CRC = " << expected_remaining_length << ")");
                    if (buffer.size() < expected_remaining_length) {
                        ROS_INFO("Buffer too small for payload+CRC. State WAITING_FOR_PAYLOAD_CRC. Waiting.");
                        break;
                    }

                    std::stringstream buffer_ss;
                    for (size_t i = 0; i < buffer.size(); i++) {
                        buffer_ss << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i];
                        if (i < buffer.size() - 1) buffer_ss << " ";
                    }
                    ROS_INFO_STREAM("Full buffer before CRC check (size=" << buffer.size() << "): " << buffer_ss.str());

                    std::string payload_str(buffer.begin(), buffer.begin() + expected_payload_length);
                    std::string msg_id = payload_str.substr(0, 2);
                    ROS_INFO_STREAM("Received message with ID: " << msg_id << ", payload: " << payload_str);

                    std::string calculated_crc_ascii_hex = "ERROR";
                    std::string received_crc_ascii_hex = "ERROR";
                    bool crc_ok = false;
                    bool crc_format_valid = true;

                    try {
                        std::vector<uint8_t> data_for_crc;
                        data_for_crc.push_back(static_cast<uint8_t>(current_length_ascii_hex[0]));
                        data_for_crc.push_back(static_cast<uint8_t>(current_length_ascii_hex[1]));
                        data_for_crc.insert(data_for_crc.end(), buffer.begin(), buffer.begin() + expected_payload_length);

                        std::stringstream crc_data_ss;
                        for (auto b : data_for_crc) {
                            crc_data_ss << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                        }
                        ROS_INFO_STREAM("Data for CRC calculation: " << crc_data_ss.str());

                        calculated_crc_ascii_hex = calculateAsciiHexCRC(data_for_crc);

                        received_crc_ascii_hex = "";
                        for (size_t i = 0; i < 4; i++) {
                            size_t crc_index = expected_payload_length + i;
                            if (crc_index >= buffer.size()) {
                                throw std::runtime_error("Buffer too short to extract CRC at index " + std::to_string(crc_index));
                            }
                            char c = static_cast<char>(buffer[crc_index]);
                            if (!isxdigit(c)) {
                                std::stringstream ss;
                                ss << std::hex << std::setw(2) << std::setfill('0') << (int)c;
                                ROS_ERROR_STREAM("Invalid hex digit in received CRC at position " << i << ": " << ss.str());
                                crc_format_valid = false;
                                break;
                            }
                            received_crc_ascii_hex.push_back(c);
                        }

                        if (crc_format_valid) {
                            std::stringstream crc_bytes_ss;
                            for (size_t i = 0; i < 4; i++) {
                                crc_bytes_ss << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[expected_payload_length + i] << " ";
                            }
                            ROS_INFO_STREAM("Raw CRC bytes: " << crc_bytes_ss.str());
                        }

                        if (crc_format_valid) {
                            crc_ok = (received_crc_ascii_hex == calculated_crc_ascii_hex);
                            ROS_INFO_STREAM("CRC Check - Calculated: " << calculated_crc_ascii_hex 
                                            << " Received: " << received_crc_ascii_hex 
                                            << " Match: " << (crc_ok ? "YES" : "NO"));
                        } else {
                            ROS_WARN("CRC format invalid. Skipping CRC check and treating as mismatch.");
                        }
                    } catch (const std::exception& e) {
                        ROS_ERROR_STREAM("Exception during CRC check: " << e.what());
                        crc_ok = false;
                        crc_format_valid = false;
                    }

                    if (crc_ok) {
                        ROS_INFO_STREAM("CRC OK. Received: " << received_crc_ascii_hex << ". Parsing message.");

                        bool parsed_ok = false;
                        if (msg_id == "A1") {
                            plc_message_parser::Heartbeat hb_msg;
                            parsed_ok = parseHeartbeat(payload_str, hb_msg);
                            if (parsed_ok) heartbeat_pub.publish(hb_msg);
                        } else if (msg_id == "A2") {
                            plc_message_parser::PickPlace pp_msg;
                            parsed_ok = parsePickPlace(payload_str, pp_msg);
                            if (parsed_ok) pick_place_pub.publish(pp_msg);
                        } else if (msg_id == "A3") {
                            plc_message_parser::Acknowledgement ack_msg;
                            parsed_ok = parseAcknowledgement(payload_str, ack_msg);
                            if (parsed_ok) ack_pub.publish(ack_msg);
                        } else if (msg_id == "A4") {
                            plc_message_parser::JobTwistlock jt_msg;
                            parsed_ok = parseJobTwistlock(payload_str, jt_msg);
                            if (parsed_ok) job_twistlock_pub.publish(jt_msg);
                        } else {
                            ROS_WARN_STREAM("Unknown message ID: " << msg_id);
                        }

                        if (parsed_ok) {
                            ROS_INFO_STREAM("Successfully parsed and published message: " << msg_id);
                        } else {
                            ROS_WARN_STREAM("Failed to parse message with ID: " << msg_id);
                        }

                        size_t bytes_to_remove = expected_payload_length + 4;
                        buffer.erase(buffer.begin(), buffer.begin() + bytes_to_remove);
                        ROS_INFO_STREAM("Processed message. Removed " << bytes_to_remove << " bytes. Buffer size: " << buffer.size());
                    } else {
                        ROS_WARN_STREAM("CRC Mismatch or Invalid Format! Recv: " << received_crc_ascii_hex << ", Calc: " << calculated_crc_ascii_hex << ". Discarding entire message.");
                        size_t bytes_to_remove = std::min(buffer.size(), static_cast<size_t>(expected_payload_length + 4));
                        buffer.erase(buffer.begin(), buffer.begin() + bytes_to_remove);
                        ROS_INFO_STREAM("Removed " << bytes_to_remove << " bytes. Buffer size: " << buffer.size());
                    }

                    current_state = ParserState::WAITING_FOR_MARKER;
                    state_changed = true;
                    break;
                }
            }
        }

        if (!buffer.empty()) {
            last_message_time = ros::Time::now();
        }

        if ((ros::Time::now() - last_message_time).toSec() > MESSAGE_TIMEOUT_SEC) {
            ROS_WARN_THROTTLE(60, "No messages received for %.1f seconds. Device may not be sending data.", MESSAGE_TIMEOUT_SEC);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}
