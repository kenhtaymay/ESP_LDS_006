#pragma once

#include <stdbool.h>
#include <stdint.h>

/* LDS-006 frame: 0xFA, index, speed (u16 LE), 4 x (distance, reflectivity) (u16 LE),
 * checksum (u16 LE) = plain sum of bytes 0..19. Index 0xA0..0xF9 is a measurement
 * (angle = (index - 0xA0) * 4), 0xFB means the sensor rejects the current speed. */
#define LIDAR_FRAME_LEN 22
#define LIDAR_INDEX_SPEED_ERROR 0xFB

typedef struct {
    uint8_t buf[LIDAR_FRAME_LEN];
    int len;
    uint32_t valid;         /* measurement frames */
    uint32_t speed_errors;  /* 0xFB frames */
    uint32_t bad_checksum;
} lidar_parser_t;

typedef struct {
    uint8_t index;
    uint16_t speed;
    uint16_t distance[4];  /* readings for angles start+0..3 */
    uint16_t strength[4];
} lidar_frame_t;

/* Start angle of a measurement frame, or -1 for 0xFB / other status frames. */
int lidar_frame_angle(const lidar_frame_t *f);

/* True when a reading is a real return: strength > 10 and not a 0x7777/0x8888/0x9999 code. */
bool lidar_reading_valid(uint16_t distance, uint16_t strength);

void lidar_parser_init(lidar_parser_t *p);

/* Feeds one byte. Returns true and fills *out when a checksum-valid frame completes. */
bool lidar_parser_feed(lidar_parser_t *p, uint8_t byte, lidar_frame_t *out);
