#pragma once

#include <array>
#include <stdint.h>
#include "SimpleCAN.h"
#include "SimpleFOC.h"

// https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can


constexpr size_t BUFFER_RX_SIZE = 64;
constexpr uint8_t VESC_FW_MAJOR = 0xBE;
constexpr uint8_t VESC_FW_MINOR = 0xEF;


class CanInterface {
public:
    void linkMotor(FOCMotor *motor);
    void linkCan(BaseCAN *can);
    
    void setCanAddr(uint8_t address);
    void setCanSpeed(uint32_t speed);
    void setVBus(float voltage);
    void setErrorState(int err);
    void begin();
    void run();
    void enableRemote(bool enable);


private:
    void process_short_buffer(CanMsg rxMsg);
    void queue_buffer_response(uint8_t controller_id, const uint8_t *data, uint8_t len, uint8_t send);
    void process_buffer_response();
    BaseCAN *can = nullptr;
    FOCMotor *motor = nullptr;

    float voltage=0.0;
    int error_state = 0;
    uint32_t time_of_last_comm = 0;
    int32_t can_speed = 500000;
    uint8_t can_address = 0xFF;
    uint8_t remote_enable = 1;

    std::array<uint8_t, BUFFER_RX_SIZE> tx_buffer{};
    uint8_t tx_buffer_len = 0;
    uint8_t tx_buffer_offset = 0;
    uint8_t tx_controller_id = 0;
    uint8_t tx_send = 0;
    bool tx_buffer_pending = false;

};
