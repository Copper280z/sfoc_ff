#include "vesc_can.h"
#include <algorithm>

enum VescState : uint32_t {
	VESC_STATE_UNKNOWN = 0,
	VESC_STATE_INCOMPATIBLE = 1,
	VESC_STATE_PONG = 2,
	VESC_STATE_COMPATIBLE = 3,
	VESC_STATE_READY = 4,
	VESC_STATE_ERROR = 5
};

enum VescCANMsg : uint8_t {
	CAN_PACKET_FILL_RX_BUFFER = 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
	CAN_PACKET_PROCESS_RX_BUFFER = 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
	CAN_PACKET_SET_CURRENT_REL = 10,
	CAN_PACKET_PING = 17,
	CAN_PACKET_PONG = 18,
	CAN_PACKET_POLL_ROTOR_POS = 56
};

enum VescCmd : uint8_t {
	COMM_FW_VERSION = 0,
	COMM_ROTOR_POSITION = 22,
	COMM_GET_VALUES_SELECTIVE = 50
};

void CanInterface::linkCan(BaseCAN *_can) {
    this->can = _can;
}

void CanInterface::linkMotor(FOCMotor *_motor) {
    this->motor = _motor;
}
void CanInterface::setCanSpeed(uint32_t _speed) {
    this->can_speed = _speed;
}
void CanInterface::setCanAddr(uint8_t _addr) {
    this->can_address = _addr;
}
void CanInterface::begin() {
    this->can->logTo(&Serial);
    this->can->disableInternalLoopback();
    CanFilter filter = CanFilter(MASK_EXTENDED, this->can_address, 0xFF, FILTER_ANY_FRAME);
    this->can->filter(filter);
    this->can->begin(this->can_speed);
    this->time_of_last_comm = micros();
}

void CanInterface::enableRemote(bool enable) {
    this->remote_enable = enable;
}

void CanInterface::run() {
    if (!this->can) return;
    if (!this->motor) return;

    uint32_t current_time = micros();
    if ((current_time - this->time_of_last_comm > 9e5f) && this->motor->enabled && this->remote_enable) {
        this->motor->disable();
        Serial.printf("No Comms: Motor Off\n");
    }

    while (this->can->available() > 0)
    {
        CanMsg const rxMsg = CAN.read();
        // vesc is always an extended id
        if (!rxMsg.isExtendedId())
            continue;

        uint32_t ext_id = rxMsg.getExtendedId();
        uint8_t id = ext_id & 0xFF;
        uint8_t cmd = (ext_id >> 8) & 0xFF;

        if (!(id & this->can_address || id & 0xFF)) 
            continue;
        
        CanMsg txMsg;

        switch(cmd) {
            case VescCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER:
            {
                // we process the data as if it were a message
                this->process_short_buffer(rxMsg);
            }
                break;
            case VescCANMsg::CAN_PACKET_SET_CURRENT_REL:
            {
                // set torque as a fraction of max current
                if (rxMsg.data_length <4) break;
                if (!this->remote_enable) {
                    break;
                }
                int32_t int_tq = 0;
                memcpy(&int_tq, rxMsg.data, 4);
                int_tq = __builtin_bswap32(int_tq);
                float torque_request = (float) int_tq / 100000.0f; 
                float max_current = this->motor->current_limit;
                this->motor->target = max_current * torque_request;
            }
                break;

            case VescCANMsg::CAN_PACKET_PING: 
            {
                // reply with a PONG
                Serial.println("PING");
                txMsg = CanMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_PONG<<8)),0, nullptr);
                this->can->write(txMsg);
            }
                break;    
            case VescCANMsg::CAN_PACKET_POLL_ROTOR_POS: 
            {
                // reply with same, scale factor = 1e5
                float mech_angle = fmodf(this->motor->shaft_angle, _2PI)*180.0f/_PI;
                int32_t pos = mech_angle * 100000.0f;
                std::array<uint8_t,4> buffer; 
                memcpy(buffer.data(),&pos,4);
                std::reverse(buffer.begin(), buffer.end());
                txMsg = CanMsg(CanExtendedId(this->can_address | (VescCANMsg::CAN_PACKET_POLL_ROTOR_POS<<8)),4, buffer.data());
                this->can->write(txMsg);
            }
                break;
        }
        
    }
}

void CanInterface::process_short_buffer(CanMsg rxMsg) {

    if (rxMsg.data_length < 3 ) return;

    uint8_t sendTo = rxMsg.data[0];
    uint8_t action = rxMsg.data[1];
    uint8_t comm_cmd = rxMsg.data[2];
    std::array<uint8_t,BUFFER_RX_SIZE> buffer;

    size_t ind = 0;

    CanMsg txMsg;
    
    switch (comm_cmd) {
        case VescCmd::COMM_FW_VERSION:
            buffer[ind++] = this->can_address;
            buffer[ind++] = 0; // process this pls
            buffer[ind++] = VescCmd::COMM_FW_VERSION;
            buffer[ind++] = VESC_FW_MAJOR;
            buffer[ind++] = VESC_FW_MINOR;

            txMsg = CanMsg(CanExtendedId(sendTo | (VescCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)), ind, buffer.data());
            this->can->write(txMsg); 
            break;
        case VescCmd::COMM_GET_VALUES_SELECTIVE:
            this->time_of_last_comm = micros();
            if (!this->motor->enabled) {
                if (this->remote_enable) {
                    this->motor->enable();
                }
            }

            uint32_t request = 0;
            memcpy(&request, &rxMsg.data[3], 4);
            request = __builtin_bswap32(request);

            // TODO
            // this needs to use the FILL_BUFFER part of the protocol
            // the voltage message is too long to fit in an 8 byte CAN frame, so this wont work

            if (request & (1<<8)) {
                ind=0;
                buffer[ind++] = this->can_address;
                buffer[ind++] = 0; // process this pls
                buffer[ind++] = VescCmd::COMM_GET_VALUES_SELECTIVE;
                uint32_t tmp_req = __builtin_bswap32(1<<8);
                memcpy(&buffer[ind],&tmp_req,4);
                ind+=4;
                // send voltage
                int16_t scaled_voltage = (int16_t) (this->voltage*10.0f);
                buffer[ind++] = scaled_voltage>>8;


                // ind+=2; // commented out because if we try to send an invalid frame, it gets null'd out on us
                // so only send the voltage MS Byte
                txMsg = CanMsg(CanExtendedId(sendTo | (VescCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)), ind, buffer.data());
                this->can->write(txMsg); 
            }
            if (request & (1<<15)) {
                ind=0;
                buffer[ind++] = this->can_address;
                buffer[ind++] = 0; // process this pls
                buffer[ind++] = VescCmd::COMM_GET_VALUES_SELECTIVE;
                uint32_t tmp_req = __builtin_bswap32(1<<15);
                memcpy(&buffer[ind],&tmp_req,4);

                ind+=4;
                // send status
                uint8_t status = 0;
                if (error_state) {
                    status = 1;
                }
                buffer[ind++] = status;
                txMsg = CanMsg(CanExtendedId(sendTo | (VescCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)), ind, buffer.data());
                this->can->write(txMsg); 
            }

 
            break;
        };
}

void CanInterface::setVBus(float _voltage) {
    this->voltage = _voltage;
}

void CanInterface::setErrorState(int _error) {
    this->error_state = _error;
}

// const uint8_t crctable[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
// 		0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
// 		0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
// 		0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
// 		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
// 		0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
// 		0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
// 		0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
// 		0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
// 		0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
// 		0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
// 		0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
// 		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
// 		0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
// 		0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
// 		0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
// 		0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
// 		0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
// 		0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
// 		0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
// 		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
// 		0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
// 		0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
// 		0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
// 		0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
// 		0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
// 		0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
// 		0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
// 		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };


// static uint16_t calculateCrc16_8(uint8_t* buf,uint16_t len){
//     uint16_t crc = 0;
// 	for(uint16_t i = 0;i<len;i++){
// 		crc = crctable[(((crc >> 8) ^ buf[i]) & 0xFF)] ^ (crc << 8);
// 	}
// 	return crc;
// }
