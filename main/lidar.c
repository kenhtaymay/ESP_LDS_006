#include "lidar.h"

#include <string.h>

void lidar_parser_init(lidar_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

int lidar_frame_angle(const lidar_frame_t *f)
{
    return (f->index >= 0xA0 && f->index <= 0xF9) ? (f->index - 0xA0) * 4 : -1;
}

bool lidar_reading_valid(uint16_t distance, uint16_t strength)
{
    return strength > 10 && distance != 0x7777 && distance != 0x8888 && distance != 0x9999;
}

/* After a checksum failure, restart from the next 0xFA already buffered (if any)
 * instead of dropping the whole 22 bytes. */
static void resync(lidar_parser_t *p)
{
    for (int i = 1; i < p->len; i++) {
        if (p->buf[i] == 0xFA) {
            memmove(p->buf, p->buf + i, p->len - i);
            p->len -= i;
            return;
        }
    }
    p->len = 0;
}

bool lidar_parser_feed(lidar_parser_t *p, uint8_t byte, lidar_frame_t *out)
{
    if (p->len == 0 && byte != 0xFA) {
        return false;
    }
    p->buf[p->len++] = byte;
    if (p->len < LIDAR_FRAME_LEN) {
        return false;
    }

    uint16_t sum = 0;
    for (int i = 0; i < 20; i++) {
        sum += p->buf[i];
    }
    if (sum != (uint16_t)(p->buf[20] | (p->buf[21] << 8))) {
        p->bad_checksum++;
        resync(p);
        return false;
    }

    out->index = p->buf[1];
    out->speed = p->buf[2] | (p->buf[3] << 8);
    for (int j = 0; j < 4; j++) {
        out->distance[j] = p->buf[4 + 4 * j] | (p->buf[5 + 4 * j] << 8);
        out->strength[j] = p->buf[6 + 4 * j] | (p->buf[7 + 4 * j] << 8);
    }
    if (out->index >= 0xA0 && out->index <= 0xF9) {
        p->valid++;
    } else if (out->index == LIDAR_INDEX_SPEED_ERROR) {
        p->speed_errors++;
    }
    p->len = 0;
    return true;
}
