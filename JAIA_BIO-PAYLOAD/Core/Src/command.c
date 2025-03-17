/*
 * command.c
 *
 *  Created on: Mar 5, 2025
 *      Author: ColinVincent
 */


#include "command.h"
#include "cobs.h"
#include <../nanopb/pb_encode.h>
#include <../nanopb/pb_decode.h>
#include "nanopb/jaiabot/messages/sensor/sensor_core.pb.h"

typedef jaiabot_sensor_protobuf_SensorRequest SensorRequest;

// Command Processing
UART_QUEUE uQueue;
uint8_t msg[256];
#define DECODED_MSG_SIZE 256
// Number of bytes in CRC32 
#define CRC32_SIZE 4
// Bit shift factor
#define BITS_IN_BYTE 8

void process_cmd(void)
{
  if (uQueue.msgCount > 0)
  {
    // First calculate which message we need to process from the queue (0 - 16). wIndex - msgCount
    uQueue.rIndex = uQueue.wIndex - uQueue.msgCount;
    uQueue.rIndex = (uQueue.rIndex < 0) ? (uQueue.rIndex + UART_QUEUE_SIZE) : uQueue.rIndex;

    uQueue.msgCount--;

    printf("Processing Command! : %s\r\n", uQueue.msgQueue[uQueue.rIndex]);

    // Buffer to hold decoded message
    uint8_t decoded_msg[DECODED_MSG_SIZE] = {0};

    // Perform COBS decoding
    COBSUnStuffData((const unsigned char*)uQueue.msgQueue[uQueue.rIndex],
                    strlen((char*)uQueue.msgQueue[uQueue.rIndex]),
                    decoded_msg);

    // Print the decoded message
    printf("Decoded Command: %s\r\n", decoded_msg);

    // Ensure message has enough bytes for CRC32 verification
    size_t decoded_length = strlen((char*)decoded_msg);
    if (decoded_length < CRC32_SIZE) {
        printf("Error: Message too short for CRC32 validation!\n");
        return;
    }

    // Compute CRC32 of the actual message (excluding last 4 bytes)
    uint32_t computed_crc = compute_crc32(decoded_msg, decoded_length - CRC32_SIZE);

    // Extract the provided CRC32 from the last 4 bytes of the message
    uint32_t provided_crc = 0;
    for (size_t i = 0; i < CRC32_SIZE; i++) {
        provided_crc |= decoded_msg[decoded_length - CRC32_SIZE + i] << (i * BITS_IN_BYTE);
    }

    // Validate CRC32
    if (computed_crc != provided_crc) {
        printf("Error: Computed CRC (0x%lX) does not match provided CRC (0x%lX)\n", computed_crc, provided_crc);
        return;
    }

    printf("CRC32 verification successful!\n");

    // Create a protobuf input stream
    pb_istream_t istream = pb_istream_from_buffer(decoded_msg, decoded_length - CRC32_SIZE);
    SensorRequest message = jaiabot_sensor_protobuf_SensorData_init_zero;
    if (!pb_decode(&istream, &jaiabot_sensor_protobuf_SensorRequest_msg, &message)) {
        printf("Error: Protobuf decoding failed: %s\n", PB_GET_ERROR(&istream));
        return;
    }

  }
}
