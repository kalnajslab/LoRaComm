/*
 * LoRaComm.cpp
 * Author:  Alex St. Clair
 * Created: August 2019
 *
 * This file implements an Arduino library (C++ class) that implements a simple, robust
 * serial (UART) protocol for inter-Arduino messaging.
 *
 * This class doesn't define specific messages, so any project using the protocol must
 * implement message definitions on top of this class.
 */


#include "LoRaComm.h"
#if defined(CORE_TEENSY)
    #define DEBUG_SERIAL Serial
#else
    #define DEBUG_SERIAL SerialUSB
#endif 
// -------------------- Initialization --------------------

LoRaComm::LoRaComm(Stream * stream_in)
{
    buffer_stream = stream_in;

    // explicity set the pointers to NULL
    binary_rx.bin_buffer = NULL;
    binary_tx.bin_buffer = NULL;
}

void LoRaComm::UpdatePort(Stream * stream_in)
{
    buffer_stream = stream_in;
}

void LoRaComm::AssignBinaryRXBuffer(uint8_t * buffer, uint16_t size)
{
    binary_rx.bin_buffer = buffer;
    binary_rx.buffer_size = size;
}

void LoRaComm::AssignBinaryTXBuffer(uint8_t * buffer, uint16_t size, uint16_t num_bytes)
{
    binary_tx.bin_buffer = buffer;
    binary_tx.buffer_size = size;
    binary_tx.bin_length = num_bytes;
}

// -------------------------- RX --------------------------
int LoRaComm::RXtoBuff()
{
  int cnt = 0;
  if (LoRa.available() == 0) {
      DEBUG_SERIAL.println("Nothing Available");
      return -1;
  }

  DEBUG_SERIAL.print("Receiving:");
  buffer_stream->flush();  
  while(LoRa.available()) //we received a parsePacket
    {
      DEBUG_SERIAL.write(LoRa.peek());
      buffer_stream->write(LoRa.read());
      cnt++;
    }
  LoRa.flush();
 
  DEBUG_SERIAL.println();
  return cnt;
}

SerialMessage_t LoRaComm::RX()
{
    ResetRX();

    if (!RXtoBuff()) return NO_MESSAGE;
    

    uint32_t timeout = millis() + READ_TIMEOUT;
    char rx_char = '\0';

    while (timeout > millis() && -1 != (rx_char = buffer_stream->read())) {
        DEBUG_SERIAL.write(rx_char);
        switch (rx_char) {
        case ASCII_DELIMITER:
            if (Read_ASCII(timeout)) {
                buffer_stream->flush();
                return ASCII_MESSAGE;
            } else {
                return NO_MESSAGE;
            }
        case ACK_DELIMITER:
            if (Read_Ack(timeout)) {
                buffer_stream->flush();
                return ACK_MESSAGE;
            } else {
                return NO_MESSAGE;
            }
        case BIN_DELIMITER:
            if (Read_Bin(timeout)) {
                buffer_stream->flush();
                return BIN_MESSAGE;
            } else {
                return NO_MESSAGE;
            }
        default:
            break;
        }
    }

    return NO_MESSAGE;
}

void LoRaComm::ResetRX()
{
    ascii_rx.msg_id = 0;
    ascii_rx.num_params = 0;
    ascii_rx.buffer_index = 0;
    ascii_rx.buffer[0] = '\0';
}

void LoRaComm::ResetTX()
{
    ascii_tx.msg_id = 0;
    ascii_tx.num_params = 0;
    ascii_tx.buffer_index = 0;
    ascii_tx.buffer[0] = '\0';
}

bool LoRaComm::Read_ASCII(uint32_t timeout)
{
    char id_buffer[4] = {0}; // uint8 up to 3 chars long
    char rx_char = '\0';
    unsigned int temp = 0;

    // read the message id
    while (timeout > millis() && temp < 3) {
        // check for delimiters
        rx_char = buffer_stream->peek();
        if (rx_char == ',' || rx_char == ';') break;

        // add to the id buffer
        id_buffer[temp++] = buffer_stream->read();
    }

    // if the next char isn't a delimiter, there's been an error
    rx_char = buffer_stream->peek();
    if (rx_char != ',' && rx_char != ';') return false;

    // convert the message id
    if (1 != sscanf(id_buffer, "%u", &temp)) return false;
    if (temp > 255) return false;
    ascii_rx.msg_id = (uint8_t) temp;

    // read the parameters into the buffer
    while (timeout > millis() && -1 != (rx_char = buffer_stream->read())) {
        // check for special characters
        if (';' == rx_char) {
            ascii_rx.buffer[ascii_rx.buffer_index] = '\0'; // null terminate
            ascii_rx.buffer_index = 0; // reset index to zero
            return true;
        }
        if (',' == rx_char) ascii_rx.num_params++;

        // add character to the buffer
        ascii_rx.buffer[ascii_rx.buffer_index++] = rx_char;
    }
    return false;
}

bool LoRaComm::Read_Ack(uint32_t timeout)
{
    char id_buffer[4] = {0}; // uint8 up to 3 chars long
    char rx_char = '\0';
    unsigned int temp = 0;

    // read the message id
    while (timeout > millis() && temp < 3) {
        // check for delimiters
        rx_char = buffer_stream->peek();
        if (rx_char == ',') break;

        // add to the id buffer
        id_buffer[temp++] = buffer_stream->read();
    }

    // if the next char isn't a comma, there's been an error
    rx_char = buffer_stream->read();
    if (rx_char != ',') return false;

    // convert the message id
    if (1 != sscanf(id_buffer, "%u", &temp)) return false;
    if (temp > 255) return false;
    ack_id = (uint8_t) temp;

    // read the ack value
    rx_char = buffer_stream->read();
    if (';' != buffer_stream->peek()) return false; // verify trailing ';' first
    if ('0' == rx_char) {
        ack_value = false;
    } else if ('1' == rx_char) {
        ack_value = true;
    } else {
        return false;
    }

    buffer_stream->read(); // read trailing ';'

    return true;
}

bool LoRaComm::Read_Bin(uint32_t timeout)
{
    char id_buffer[4] = {0}; // uint8 up to 3 chars long
    char length_buffer[6] = {0};
    char rx_char = '\0';
    unsigned int temp = 0;

    // ensure rx message struct is reset
    binary_rx.bin_length = 0;
    binary_rx.bin_id = 0;
    binary_rx.checksum_read = 0;
    binary_rx.checksum_calc = 0;

    // ensure the destination buffer is valid
    if (binary_rx.bin_buffer == NULL){
        DEBUG_SERIAL.println("Binary RX Buffer no assigned!");    
        return false;
    }
    // read the binary id
    while (timeout > millis() && temp < 3) {
        // check for delimiters
        rx_char = buffer_stream->peek();
        if (rx_char == ',') break;

        // add to the id buffer
        id_buffer[temp++] = buffer_stream->read();
    }
   

    // if timed out, flush the buffer and return error
    if (timeout <= millis()) {
        buffer_stream->flush();
        return false;
    }

    // if the next char isn't a comma, there's been an error
    rx_char = buffer_stream->read();
    if (rx_char != ',') return false;

    // convert the binary id
    if (1 != sscanf(id_buffer, "%u", &temp)) return false;
    if (temp > 255) return false;
    binary_rx.bin_id = (uint8_t) temp;
    

    // read the binary length
    temp = 0;
    while (timeout > millis() && temp < 5) {
        // check for delimiters
        rx_char = buffer_stream->peek();
        if (rx_char == ';') break;

        // add to the id buffer
        length_buffer[temp++] = buffer_stream->read();
    }

    // if timed out, flush the buffer and return error
    if (timeout <= millis()) {
        buffer_stream->flush();
        return false;
    }

    // if the next char isn't a semicolon, there's been an error
    rx_char = buffer_stream->read();
    if (rx_char != ';') return false;

    // convert the binary length
    if (1 != sscanf(length_buffer, "%u", &temp)) return false;
    if (temp > 65535) return false;
    binary_rx.bin_length = (uint16_t) temp;

    // ensure we won't overflow the buffer
    if (binary_rx.bin_length > binary_rx.buffer_size) {
        buffer_stream->flush();
        return false;
    }

    // read the binary section
    temp = 0;
    while (timeout > millis() && temp < binary_rx.bin_length) {
        binary_rx.bin_buffer[temp++] = buffer_stream->read();
    }

    // if timed out, flush the buffer and return error
    if (timeout <= millis()) {
        buffer_stream->flush();
        return false;
    }

    // if the next char isn't a semicolon, there's been an error
    rx_char = buffer_stream->read();
    if (rx_char != ';') return false;

    // read the checksum
    temp = 0;
    while (timeout > millis() && temp < 5) {
        // check for delimiters
        rx_char = buffer_stream->peek();
        if (rx_char == ';') break;

        // add to the id buffer
        length_buffer[temp++] = buffer_stream->read();
    }

    // if timed out, flush the buffer and return error
    if (timeout <= millis()) {
        buffer_stream->flush();
        return false;
    }

    // if the next char isn't a semicolon, there's been an error
    rx_char = buffer_stream->read();
    if (rx_char != ';') return false;

    // convert the checksum
    if (1 != sscanf(length_buffer, "%u", &temp)) return false;
    if (temp > 65535) return false;
    binary_rx.checksum_read = (uint16_t) temp;

    //calulate the checksum and return false if they don' agree
    binary_rx.checksum_calc = calcChecksum(binary_rx.bin_buffer,binary_rx.bin_length);
    if (binary_rx.checksum_calc != binary_rx.checksum_read) return false;

    return true;
}

// -------------------------- TX --------------------------
int LoRaComm::TXfromBuff()
{
  int cnt = 0;
  DEBUG_SERIAL.print("Sending: ");
  LoRa.beginPacket();
  while (buffer_stream->available()) {
    DEBUG_SERIAL.write(buffer_stream->peek());
    LoRa.write(buffer_stream->read());
    cnt++;
  }
  buffer_stream->flush();
  DEBUG_SERIAL.println();

  LoRa.endPacket(); //blocking to make sure it sends
  
  return cnt;
}

void LoRaComm::TX_ASCII()
{
    TX_ASCII(ascii_tx.msg_id);
}

void LoRaComm::TX_ASCII(uint8_t msg_id)
{
   
    buffer_stream->print(ASCII_DELIMITER);
    buffer_stream->print(msg_id);
    if (ascii_tx.buffer_index > 0) {
        buffer_stream->print(ascii_tx.buffer);
    }
    buffer_stream->print(';');
    TXfromBuff();
    ResetTX();
}

void LoRaComm::TX_Ack(uint8_t msg_id, bool ack_val)
{
   
    buffer_stream->print(ACK_DELIMITER);
    buffer_stream->print(msg_id);
    buffer_stream->print(',');
    ack_val ? buffer_stream->print('1') : buffer_stream->print('0');
    buffer_stream->print(';');
    TXfromBuff();
    ResetTX();
}

bool LoRaComm::TX_Bin()
{
    return TX_Bin(binary_tx.bin_id);
}

bool LoRaComm::TX_Bin(uint8_t bin_id)
{
    if (binary_tx.bin_buffer == NULL) return false;
    //Calculate checksum
    binary_tx.checksum_calc = calcChecksum(binary_tx.bin_buffer, binary_tx.bin_length);
    
 
    buffer_stream->print(BIN_DELIMITER);
    buffer_stream->print(bin_id);
    buffer_stream->print(",");
    buffer_stream->print(binary_tx.bin_length);
    buffer_stream->print(";");
    
    for (int i = 0; i < binary_tx.bin_length; i++) {
        buffer_stream->write(binary_tx.bin_buffer[i]);
    }
    buffer_stream->print(";");
    //Add the checksum at the end
    buffer_stream->print(binary_tx.checksum_calc);
    buffer_stream->print(";");
    TXfromBuff();
    ResetTX();

    return true;
}

// -------------------- Buffer Parsing --------------------

bool LoRaComm::Get_uint8(uint8_t * ret_val)
{
    char int_buffer[4] = {0};
    uint8_t max_index = ascii_rx.buffer_index + 3; // uint8_t means 3 chars max
    unsigned int temp = 0;

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        int_buffer[temp++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // convert the message id
    if (1 != sscanf(int_buffer, "%u", &temp)) return false;
    if (temp > 255) return false;
    *ret_val = (uint8_t) temp;

    return true;
}

bool LoRaComm::Get_uint16(uint16_t * ret_val)
{
    char int_buffer[6] = {0};
    uint8_t max_index = ascii_rx.buffer_index + 5; // uint16_t means 5 chars max
    unsigned int temp = 0;

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        int_buffer[temp++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // convert the message id
    if (1 != sscanf(int_buffer, "%u", &temp)) return false;
    if (temp > 65535) return false;
    *ret_val = (uint16_t) temp;

    return true;
}

bool LoRaComm::Get_uint32(uint32_t * ret_val)
{
    char int_buffer[11] = {0};
    uint8_t max_index = ascii_rx.buffer_index + 10; // uint32_t means 10 chars max
    unsigned int temp = 0;

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        int_buffer[temp++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // convert the message id
    if (1 != sscanf(int_buffer, "%u", &temp)) return false;
    *ret_val = temp;

    return true;
}

bool LoRaComm::Get_int8(int8_t * ret_val)
{
    char int_buffer[5] = {0};
    uint8_t max_index = ascii_rx.buffer_index + 4; // int8_t means 4 chars max
    int temp = 0;

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        int_buffer[temp++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // convert the message id
    if (1 != sscanf(int_buffer, "%d", &temp)) return false;
    if (temp > 127 || temp < -128) return false;
    *ret_val = (int8_t) temp;

    return true;
}

bool LoRaComm::Get_int16(int16_t * ret_val)
{
    char int_buffer[7] = {0};
    uint8_t max_index = ascii_rx.buffer_index + 6; // int16_t means 6 chars max
    int temp = 0;

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        int_buffer[temp++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // convert the message id
    if (1 != sscanf(int_buffer, "%d", &temp)) return false;
    if (temp > 32767 || temp < -32768) return false;
    *ret_val = (int16_t) temp;

    return true;
}

bool LoRaComm::Get_int32(int32_t * ret_val)
{
    char int_buffer[12] = {0};
    uint8_t max_index = ascii_rx.buffer_index + 11; // int32_t means 11 chars max
    int temp = 0;

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        int_buffer[temp++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // convert the message id
    if (1 != sscanf(int_buffer, "%d", &temp)) return false;
    *ret_val = temp;

    return true;
}

bool LoRaComm::Get_float(float * ret_val)
{
    char int_buffer[16] = {0};
    uint8_t max_index = ascii_rx.buffer_index + 15; // 15 chars max
    unsigned int temp = 0;
    float temp_float = 0.0f;

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        int_buffer[temp++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // convert the message id
    if (1 != sscanf(int_buffer, "%f", &temp_float)) return false;
    *ret_val = temp_float;

    return true;
}

bool LoRaComm::Get_string(char * buffer, uint8_t length)
{
    uint8_t buffer_index = 0;

    // leave room for null termination
    uint16_t max_index = ascii_rx.buffer_index + length - 1;
    if (max_index > (ASCII_BUFFER_SIZE - 1)) max_index = (ASCII_BUFFER_SIZE - 1);

    if (',' != ascii_rx.buffer[ascii_rx.buffer_index++]) return false; // always a leading comma

    while (ascii_rx.buffer_index <= max_index) {
        if (',' == ascii_rx.buffer[ascii_rx.buffer_index] || '\0' == ascii_rx.buffer[ascii_rx.buffer_index]) {
            break;
        }

        buffer[buffer_index++] = ascii_rx.buffer[ascii_rx.buffer_index++];
    }

    // ensure next char is ',' or '\0'
    if (',' != ascii_rx.buffer[ascii_rx.buffer_index] && '\0' != ascii_rx.buffer[ascii_rx.buffer_index]) return false;

    // null terminate the buffer
    buffer[buffer_index] = '\0';

    return true;
}

// -------------------- Buffer Addition -------------------

bool LoRaComm::Add_uint8(uint8_t val)
{
    return Add_uint32((uint32_t) val);
}

bool LoRaComm::Add_uint16(uint16_t val)
{
    return Add_uint32((uint32_t) val);
}

bool LoRaComm::Add_uint32(uint32_t val)
{
    uint8_t buffer_remaining = ASCII_BUFFER_SIZE - ascii_tx.buffer_index;
    int num_written = 0;

    // snprintf will return the number of chars it could write, but won't write more than buffer_remaining
    // note leading comma!
    num_written = snprintf(ascii_tx.buffer + ascii_tx.buffer_index, buffer_remaining, ",%u", (unsigned int) val);

    // make sure the write was valid and not too large
    if (num_written < 1 || num_written >= buffer_remaining) {
        ResetTX();
        return false;
    }

    ascii_tx.buffer_index += num_written;

    return true;
}

bool LoRaComm::Add_int8(int8_t val)
{
    return Add_int32((int32_t) val);
}

bool LoRaComm::Add_int16(int16_t val)
{
    return Add_int32((int32_t) val);
}

bool LoRaComm::Add_int32(int32_t val)
{
    uint8_t buffer_remaining = ASCII_BUFFER_SIZE - ascii_tx.buffer_index;
    int num_written = 0;

    // snprintf will return the number of chars it could write, but won't write more than buffer_remaining
    // note leading comma!
    num_written = snprintf(ascii_tx.buffer + ascii_tx.buffer_index, buffer_remaining, ",%d", (int) val);

    // make sure the write was valid and not too large
    if (num_written < 1 || num_written >= buffer_remaining) {
        ResetTX();
        return false;
    }

    ascii_tx.buffer_index += num_written;

    return true;
}

bool LoRaComm::Add_float(float val)
{
    uint8_t buffer_remaining = ASCII_BUFFER_SIZE - ascii_tx.buffer_index;
    int num_written = 0;

    // snprintf will return the number of chars it could write, but won't write more than buffer_remaining
    // note leading comma!
    num_written = snprintf(ascii_tx.buffer + ascii_tx.buffer_index, buffer_remaining, ",%f", val);

    // make sure the write was valid and not too large
    if (num_written < 1 || num_written >= buffer_remaining) {
        ResetTX();
        return false;
    }

    ascii_tx.buffer_index += num_written;

    return true;
}

bool LoRaComm::Add_string(const char * buffer)
{
    uint8_t buffer_remaining = ASCII_BUFFER_SIZE - ascii_tx.buffer_index;
    int num_written = 0;

    // snprintf will return the number of chars it could write, but won't write more than buffer_remaining
    // note leading comma!
    num_written = snprintf(ascii_tx.buffer + ascii_tx.buffer_index, buffer_remaining, ",%s", buffer);

    // make sure the write was valid and not too large
    if (num_written < 1 || num_written >= buffer_remaining) {
        ResetTX();
        return false;
    }

    ascii_tx.buffer_index += num_written;

    return true;
}

uint16_t LoRaComm::calcChecksum(uint8_t * checksumPayload, uint16_t payloadSize)
{
    uint8_t CK_A = 0, CK_B = 0;
    uint16_t checkSum = 0; 
    for (int i = 0; i < payloadSize ;i++) {
        CK_A = CK_A + *checksumPayload;
        CK_B = CK_B + CK_A;
        checksumPayload++;
    }
    checkSum = (uint16_t)(CK_A * 256 + CK_B);
    return checkSum;
}