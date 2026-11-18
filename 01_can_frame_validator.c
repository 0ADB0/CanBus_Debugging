#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define NUM_SIGNALS 3
typedef struct { uint16_t id; uint8_t dlc; uint8_t data[8]; } Frame;
typedef struct { uint16_t id; const char *name; double factor, max, min,offset; } SignalInfo;
typedef enum { V_OK, V_BAD_DLC, V_UNKNOWN_ID, V_BAD_CHECKSUM, V_BAD_COUNTER,
    V_OUT_OF_RANGE } VStatus;

static const SignalInfo SIGNALS[NUM_SIGNALS] = {
    {0x100, "VehicleSpeed", 0.01,300.0,0.00,0.00}, {0x200, "EngineRpm", 0.25, 9000.0,0.00,0.00},
    {0x300, "BrakePressure",1.0,100.0,0.00,0.00},
};

/* Write v as two bytes, high byte first. */
void pack_u16(uint8_t *out, uint16_t v){
    out[0] = (uint8_t)(v >> 8);
    out[1] = (uint8_t)(v & 0xFF);
}

/* Read two bytes (high byte first) back into one number.*/
uint64_t unpack_u16(const uint8_t *in) {
    return ((uint64_t)in[0] << 8) | (uint64_t)in[1];
}

/* XOR of the first len bytes*/
uint8_t checksum_xor(const uint8_t *d, uint8_t len){
    uint8_t cs = 0;
    for(uint16_t i = 0; i < len; i++) cs ^= d[i];
    return cs;
}

/*Alive counters run 0..15 and then wrap around to 0.*/
int counter_ok(uint8_t last, uint8_t now) {
    if(last > 15 || now > 15) return 0;
    return (now == ((last + 1) & 0x0F));
}

/* Returns the table entry for this ID, or NULL if the ID is unknown.*/
const SignalInfo *find_signal(uint16_t id){
    for(int i = 0; i < NUM_SIGNALS; i++)
        if (SIGNALS[i].id == id) return &SIGNALS[i];
    return NULL;
}

/* Frame layout: [value hi][value lo][counter][checksum]; dlc must be 4.*/
VStatus validate(const Frame *f, uint8_t last_counter) {
    if (f->dlc != 4) return V_BAD_DLC;
    const SignalInfo *s = find_signal(f->id);
    if (s == NULL) return V_UNKNOWN_ID;
    if (checksum_xor(f->data, 3) != f->data[3]) return V_BAD_CHECKSUM;
    if (!counter_ok(last_counter, f->data[2])) return V_BAD_COUNTER;
    double val = (double)unpack_u16(f->data) * s->factor + s->offset;
    if (val < s->min || val > s->max) {
        return V_OUT_OF_RANGE;
    }
    return V_OK;
}

/* How many stuff bits would CAN insert while sending these bytes (MSB first)?
   After 5 equal bits in a row, one bit of the opposite value is inserted.*/
int count_stuff_bits(const uint8_t *bytes, size_t n){
    int stuffed = 0, last = -1, run = 0;
    for (size_t i = 0; i < n; i++){
        for (int b = 7; b >= 0; b--){
            int bit = (bytes[i] >> b) & 1;
            if (bit == last) { run++; } else {last = bit; run = 1;}
            if( run == 5) {stuffed++; last = bit ^ 1; run = 1; }
        }    
    }
    return stuffed;
}

/* Write the signal name into dst as "<name>" (dst hast dst_size bytes).*/
void format_label(char *dst, size_t dst_size, const SignalInfo *s){
    if (dst == NULL || dst_size == 0) return;
    snprintf(dst, dst_size ,"<%s>", s->name);
}

static int g_total = 0, g_failed = 0;
static void check(const char *what, long got, long expected){
    g_total++;
    if(got != expected) g_failed++;
    printf("%s %-28s", got == expected ? "PASS" : "FAIL",what);
    if (got != expected) printf("got %ld, expected %ld", got, expected);
    printf("\n");
}

int main(void){
    setbuf(stdout, NULL);
    uint8_t buf[2], raw[2] = {0x1F, 0x40 }, d[4] = {0x12,0x34,0x56,0x78};
    pack_u16(buf,8000);
    check("pack 8000: high byte",buf[0],0x1F);
    check("unpack {1F,40}", unpack_u16(raw), 8000);
    check("xor of the first 3 bytes",checksum_xor(d,3),0x70);
    check("counter 3 -> 4", counter_ok(3,4),1);
    check("counter 3 -> 5", counter_ok(3,5),0);
    check("counter 15 -> 0",counter_ok(15,0),1);
    const uint8_t s1[] = {0x00}, s2[] = {0x07,0x80};
    check("stuff bits {00}", count_stuff_bits(s1,1),1);
    check("stuff bits {07,80}", count_stuff_bits(s2, 2), 3);

    Frame good = {0x100,4,{0x1F,0x40,0x04,0x5B}};
    Frame badcs = { 0x100,4,{0x1F,0x40,0x04,0x00}};
    Frame badcnt = {0x100,4,{0x1F,0x40,0x09,0x56}};
    Frame fast = {0x100,4,{0xFF,0xFF,0x05,0x05}};
    Frame alien = {0x7FF,4,{0x00,0x00,0x00,0x00,0x00}};
    check("validate good frame",validate(&good,3),V_OK);
    check("validate wrong checksum",validate(&badcs,3), V_BAD_CHECKSUM);
    check("validate wrong counter",validate(&badcnt,3), V_BAD_COUNTER);
    check("validate absurd speed",validate(&fast,4),V_OUT_OF_RANGE);
    check("validate unknown ID",validate(&alien,0),V_UNKNOWN_ID);

    char label[4];
    format_label(label, sizeof(label),&SIGNALS[0]);
    check("label fits 4-byte buffer",strcmp(label,"<Ve") == 0,1);

    if(g_failed == 0) printf("\n ALL CHECKS PASSED (%d)\n", g_total);
    else printf("\n%d of %d checks FAILED\n", g_failed, g_total);
    return g_failed == 0 ? 0 : 1;
}