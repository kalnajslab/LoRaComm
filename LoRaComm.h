/*
 * LoRaComm.h
 * Author:  Alex St. Clair
 * Created: August 2019
 * Updated: November 2020
 *
 * This file declares an Arduino library (C++ class) that implements a simple, robust
 * serial (UART) protocol for inter-Arduino messaging.
 *
 * This class doesn't define specific messages, so any project using the protocol must
 * implement message definitions on top of this class.
 * 
 * This has been adapted for LoRa messages.  
 */

#ifndef LoRaComm_H
#define LoRaComm_H

#include "Arduino.h"
#include <stdint.h>
#include <LoRa.h>
#include <LoopbackStream.h>

#define ASCII_DELIMITER    '#'
#define ACK_DELIMITER      '?'
#define BIN_DELIMITER      '!'

#define READ_TIMEOUT       2000 // milliseconds

#define ASCII_BUFFER_SIZE  128

enum LoRaMessage_t {
    NO_MESSAGE,
    ASCII_MESSAGE,
    ACK_MESSAGE,
    BIN_MESSAGE
};

struct LoRa_ASCII_MSG_t {
    uint8_t msg_id;
    uint8_t num_params;
    uint8_t buffer_index;
    char buffer[ASCII_BUFFER_SIZE];
};

struct LoRa_BIN_MSG_t {
    uint8_t bin_id;
    uint16_t bin_length;
    uint16_t buffer_size;
    uint16_t checksum_read;
    uint16_t checksum_calc;
    uint8_t * bin_buffer;
};

class LoRaComm {
public:
    LoRaComm(Stream * stream_in);
    ~LoRaComm() { };

    // To allow user to change to the USB serial port for testing or debug
    void UpdatePort(Stream * stream_in);

    // Attach pre-allocated buffers for binary messaging
    void AssignBinaryRXBuffer(uint8_t * buffer, uint16_t size);
    void AssignBinaryTXBuffer(uint8_t * buffer, uint16_t size, uint16_t num_bytes);

    // Receive interface
    int RXtoBuff();
    LoRaMessage_t RX();

    // Transmit interface
    int TXfromBuff();
    void TX_ASCII();
    void TX_ASCII(uint8_t msg_id);
    void TX_Ack(uint8_t msg_id, bool ack_val);
    bool TX_Bin();
    bool TX_Bin(uint8_t bin_id);

    // ASCII RX buffer interface
    bool Get_uint8(uint8_t * ret_val);
    bool Get_uint16(uint16_t * ret_val);
    bool Get_uint32(uint32_t * ret_val);
    bool Get_int8(int8_t * ret_val);
    bool Get_int16(int16_t * ret_val);
    bool Get_int32(int32_t * ret_val);
    bool Get_float(float * ret_val);
    bool Get_string(char * buffer, uint8_t length);

    // ASCII TX buffer interface
    bool Add_uint8(uint8_t val);
    bool Add_uint16(uint16_t val);
    bool Add_uint32(uint32_t val);
    bool Add_int8(int8_t val);
    bool Add_int16(int16_t val);
    bool Add_int32(int32_t val);
    bool Add_float(float val);
    bool Add_string(const char * buffer);

    // ASCII messages with buffers
    LoRa_ASCII_MSG_t ascii_rx = {0};
    LoRa_ASCII_MSG_t ascii_tx = {0};

    // Binary messages with buffers
    LoRa_BIN_MSG_t binary_rx = {0};
    LoRa_BIN_MSG_t binary_tx = {0};

    // Last ACK/NAK
    uint8_t ack_id = 0;
    bool ack_value = false;

private:
    // Receive message parsing
    bool Read_ASCII(uint32_t timeout);
    bool Read_Ack(uint32_t timeout);
    bool Read_Bin(uint32_t timeout);

    // reset RX/TX internal state
    void ResetRX();
    void ResetTX();

    //calculate a 16 bit checksum for a given buffer
    uint16_t calcChecksum(uint8_t *, uint16_t);

    // Serial port
    Stream * buffer_stream;

};

#endif /* LoRaComm_H */