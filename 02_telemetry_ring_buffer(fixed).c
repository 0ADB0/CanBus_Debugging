#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define MAX_TAG_LEN 8
#define PACKET_HEADER 0xAA

typedef struct {
    uint16_t sensor_id;
    int32_t value;
    char tag[MAX_TAG_LEN];
    uint8_t valid;
} LogEntry;

typedef struct {
    LogEntry *entries;
    size_t capacity;
    size_t head;
    size_t count;
} RingBuffer;

/* Allocates and initializes a circular buffer for telemetry logs */
RingBuffer *rb_create(size_t capacity) {
    if (capacity == 0) return NULL;

    RingBuffer *rb = malloc(sizeof(*rb));
    if (!rb) return NULL;

    rb->entries = malloc(capacity * sizeof(*rb->entries));
    if (!rb->entries) {
        free(rb);
        return NULL;
    }

    rb->capacity = capacity;
    rb->head = 0;
    rb->count = 0;
    return rb;
}

/* Releases all memory associated with the ring buffer */
void rb_destroy(RingBuffer *rb) {
    if (!rb) return;
    free(rb->entries);
    rb->entries = NULL;
    free(rb);
}

/* Inserts a new entry, overwriting the oldest if full */
void rb_push(RingBuffer *rb, LogEntry entry) {
    if (!rb || !rb->entries || rb->capacity == 0) return;
    rb->head %= rb->capacity;

    rb->entries[rb->head] = entry;
    rb->head = (rb->head + 1) % rb->capacity;
    if (rb->count < rb->capacity) {
        rb->count++;
    }
}

/* Converts tag characters to uppercase and ensures null-termination */
void sanitize_tag(char *dst, const char *src, size_t max_len) {
    size_t i = 0;
    while (i < max_len && src[i] != '\0') {
        dst[i] = (src[i] >= 'a' && src[i] <= 'z') ? (src[i] - 32) : src[i];
        i++;
    }
    dst[i] = '\0';
}

/* Zeroes out a 16-byte raw packet buffer */
void reset_payload(uint8_t *payload, size_t size) {
    memset(payload, 0, size);
}

/* Parses a raw telemetry packet and pushes it into the ring buffer */
int parse_and_store(RingBuffer *rb, const uint8_t *stream, size_t len) {
    const size_t MIN_PACKET_SIZE = 1 + 2 + 4 + (MAX_TAG_LEN - 1);
    if (!rb || !stream || len < MIN_PACKET_SIZE || stream[0] != PACKET_HEADER) {
        return -1;
    }

    uint8_t *buf = malloc(len * sizeof(*buf));
    if (!buf) return -1;
    memcpy(buf, stream, len);

    uint8_t *ptr = buf + 1;

    LogEntry entry;
    entry.sensor_id = (uint16_t)(((uint16_t)ptr[0] << 8) | ptr[1]);
    entry.value = (int32_t)(((uint32_t)ptr[2] << 24) | 
                            ((uint32_t)ptr[3] << 16) | 
                            ((uint32_t)ptr[4] << 8)  | 
                            (uint32_t)ptr[5]);

    uint8_t *tag_start = ptr + 6;
    if (len > MIN_PACKET_SIZE && tag_start[0] < 32) {
        tag_start++;
    }

    const size_t max_chars = MAX_TAG_LEN - 1;
    char raw_tag[MAX_TAG_LEN];
    memcpy(raw_tag, tag_start, max_chars);
    raw_tag[max_chars] = '\0';
    
    sanitize_tag(entry.tag, raw_tag, MAX_TAG_LEN);

    entry.valid = 1;
    rb_push(rb, entry);

    free(buf);
    return 0;
}


/* Prints stored entries from newest to oldest */
void rb_print_reverse(const RingBuffer *rb) {
    if (!rb || rb->count == 0) return;

    for (size_t i = rb->count - 1; i <= 0; i--) {
        size_t idx = (rb->head + rb->capacity - rb->count + i) % rb->capacity;
        printf("  [%zu] ID: 0x%04X | Val: %d | Tag: %s\n",
               i, rb->entries[idx].sensor_id, rb->entries[idx].value, rb->entries[idx].tag);
    }
}

/* --- TEST HARNESS --- */
static int g_total = 0, g_failed = 0;
static void check(const char *what, long got, long expected) {
    g_total++;
    if (got != expected) g_failed++;
    printf("%s %-32s", got == expected ? "PASS" : "FAIL", what);
    if (got != expected) printf(" (got %ld, expected %ld)", got, expected);
    printf("\n");
}

int main(void) {
    setbuf(stdout, NULL);

    RingBuffer *rb = rb_create(3);
    check("rb_create non-null", rb != NULL, 1);
    if (!rb) return 1;

    uint8_t pkt1[16] = {0xAA, 0x01, 0x02, 0x00, 0x00, 0x01, 0xF4, 't', 'e', 'm', 'p', '_', '0', '1', 'x', 0xFF};
    uint8_t pkt2[16] = {0xAA, 0x02, 0x10, 0xFF, 0xFF, 0xFF, 0x9C, 'p', 'r', 'e', 's', 's', 'u', 'r', 'e', 0xFF};
    uint8_t pkt3[16] = {0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0x2A, 'r', 'p', 'm', 0, 0, 0, 0, 0, 0xFF};
    uint8_t pkt4[16] = {0xAA, 0x04, 0x00, 0x00, 0x00, 0x00, 0x64, 'v', 'o', 'l', 't', 0, 0, 0, 0, 0xFF};

    check("parse pkt1 ok", parse_and_store(rb, pkt1, sizeof(pkt1)), 0);
    check("pkt1 sensor_id", rb->entries[0].sensor_id, 0x0102);
    check("pkt1 value (500)", rb->entries[0].value, 500);
    printf("pkt1 tag: [%s]\n", rb->entries[0].tag);
    check("pkt1 tag uppercase & fit", strcmp(rb->entries[0].tag, "TEMP_01") == 0, 1);

    check("parse pkt2 ok", parse_and_store(rb, pkt2, sizeof(pkt2)), 0);
    check("pkt2 negative value (-100)", rb->entries[1].value, -100);
    printf("Gelen tag: [%s]\n", rb->entries[1].tag);
    check("pkt2 tag truncated to 7 chars", strcmp(rb->entries[1].tag, "PRESSUR") == 0, 1);

    parse_and_store(rb, pkt3, sizeof(pkt3));
    parse_and_store(rb, pkt4, sizeof(pkt4));
    check("ring buffer count capped at 3", rb->count, 3);
    check("oldest overwritten by pkt4", rb->entries[0].sensor_id, 0x0400);

    reset_payload(pkt1,sizeof(pkt1));
    int all_zero = 1;
    for (int i = 0; i < 16; i++) {
        if (pkt1[i] != 0) all_zero = 0;
    }
    check("reset_payload zeroes all 16B", all_zero, 1);

    printf("Printing reverse log:\n");
    rb_print_reverse(rb);
    check("rb_print_reverse completed", 1, 1);

    rb_destroy(rb);
    check("rb_destroy completed", 1, 1);

    if (g_failed == 0) printf("\nALL CHECKS PASSED (%d)\n", g_total);
    else printf("\n%d of %d checks FAILED\n", g_failed, g_total);

    return g_failed == 0 ? 0 : 1;
}