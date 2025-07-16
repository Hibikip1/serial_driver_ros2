#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_comm.hpp"
#include "serial_driver/protocol_defs.hpp"
#include <iostream>

SerialComm::SerialComm(const std::string& port, unsigned long baudrate)
    : serial_port_(port, baudrate, serial::Timeout::simpleTimeout(1000)) {
    if (isOpen()) {
        RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "✅ Serial Open at: %s @ %lu bps", port.c_str(), baudrate);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "❌ Serial Open ERROR!");
    }
    
}

SerialComm::~SerialComm() {
    if (serial_port_.isOpen()) {
        serial_port_.close();
    }
}

bool SerialComm::isOpen() {
    return serial_port_.isOpen();
}

bool SerialComm::sendFloatArrayCommand(const std::vector<float>& values) {
    if (!isOpen()) return false;

    std::vector<uint8_t> frame = encodeFloatArray(values);
    size_t bytes_written = serial_port_.write(frame);

    return bytes_written == frame.size();
}

std::vector<uint8_t> SerialComm::encodeFloatArray(const std::vector<float>& values) {
    std::vector<uint8_t> frame;

    // 1. 编码 float 数组为 int16_t（缩放 1000）后拆成字节
    std::vector<uint8_t> data;
    for (float val : values) {
        int16_t scaled = static_cast<int16_t>(val * 1000);
        data.push_back(static_cast<uint8_t>((scaled >> 8) & 0xFF)); // 高字节
        data.push_back(static_cast<uint8_t>(scaled & 0xFF));        // 低字节
    }

    // 2. 帧头
    frame.push_back(protocol::FRAME_HEAD);
    frame.push_back(protocol::FRAME_HEAD_SEND);

    // 3. 数据长度（字节数）
    frame.push_back(static_cast<uint8_t>(data.size()));

    // 4. 数据内容
    frame.insert(frame.end(), data.begin(), data.end());

    // 5. 校验和
    uint8_t checksum = protocol::calcChecksum(frame);
    frame.push_back(checksum);

    return frame;
}

std::vector<float> SerialComm::readFloatArrayResponse() {
    std::vector<float> result;

    if (!isOpen()) return result;

    size_t available = serial_port_.available();
    if (available < 5) return result;  // 至少包含帧头+长度+校验

    std::vector<uint8_t> buffer;
    serial_port_.read(buffer, available);

    for (size_t i = 0; i + 4 < buffer.size(); ++i) {
        if (buffer[i] == protocol::FRAME_HEAD) {
            uint8_t frame_type = buffer[i + 1];
            if (frame_type == protocol::FRAME_HEAD_READ) {  // 接收帧
                uint8_t len = buffer[i + 2];
                if (i + 3 + len >= buffer.size()) break;  // 数据未接收完整

                std::vector<uint8_t> frame(buffer.begin() + i, buffer.begin() + i + 4 + len);

                // 校验校验和
                uint8_t expected_checksum = frame.back();
                frame.pop_back();
                uint8_t calc = protocol::calcChecksum(frame);
                if (expected_checksum != calc) {
                    std::cerr << "❌ Checksum mismatch\n";
                    continue;
                }

                // 解析 float 数组
                for (size_t j = 3; j < 3 + len; j += 2) {
                    int16_t raw = (frame[j] << 8) | frame[j + 1];
                    float val = static_cast<float>(raw) / 1000.0f;
                    result.push_back(val);
                }

                break;  // 找到一帧后退出
            }
        }
    }

    return result;
}
