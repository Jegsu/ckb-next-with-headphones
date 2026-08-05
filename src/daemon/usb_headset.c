#include "structures.h"
#include "usb.h"
#include "device.h"
#include "usb_headset.h"

// Corsair Virtuoso XT SlipStream dongle (1b1c:0a64) protocol. Every frame:
// [0x02][routing][opcode bytes...][payload bytes...], zero padded to MSG_SIZE (64)
// bytes, sent to EP 0x01 (interface 3's OUT endpoint), response read from EP 0x81
// (interface 3's IN endpoint). Routing byte 0x09 addresses the one paired headset
// directly; a single paired headset is assumed (multi-device enumeration is not
// implemented).
#define HEADSET_EP_OUT 0x01
#define HEADSET_EP_IN  0x81
#define HEADSET_ROUTING 0x09

void headset_fill_input_eps(usbdevice* kb){
    kb->input_endpoints[0] = HEADSET_EP_IN;
    kb->input_endpoints[1] = 0;
}

int headset_usb_write(usbdevice* kb, void* out, int len, int is_recv, const char* file, int line){
    if(len != MSG_SIZE){
        ckb_fatal_fn("len != %d not supported in headset backend", file, line, MSG_SIZE);
        return -1;
    }
    // If we need to read a response, lock the interrupt mutex so headset_usb_read()'s
    // cond_nanosleep() below picks up the matching response (see process_input_urb()
    // in keymap.c, which populates kb->interruptbuf and signals this condvar for
    // PROTO_HEADSET devices).
    if(is_recv)
        if(pthread_mutex_lock(intmutex(kb)))
            ckb_fatal("Error locking interrupt mutex in headset_usb_write()");

    int res = os_usb_interrupt_out(kb, HEADSET_EP_OUT, len, out, file, line);
    if(is_recv && res < 1)
        pthread_mutex_unlock(intmutex(kb));
    return res;
}

int headset_usb_read(usbdevice* kb, void* in, int len, int dummy, const char* file, int line){
    (void)dummy;
    if(len != MSG_SIZE){
        ckb_fatal_fn("len != %d not supported in headset backend", file, line, MSG_SIZE);
        return -1;
    }
    // Wait for max 2s for the input thread to signal a response arrived.
    int condret = cond_nanosleep(intcond(kb), intmutex(kb), 2000000000);
    if(condret != 0){
        if(pthread_mutex_unlock(intmutex(kb)))
            ckb_fatal("Error unlocking interrupt mutex in headset_usb_read()");
        if(condret == ETIMEDOUT)
            ckb_warn_fn("ckb%d: Timeout while waiting for response", file, line, INDEX_OF(kb, keyboard));
        else
            ckb_warn_fn("Interrupt cond error %i", file, line, condret);
        return -1;
    }
    memcpy(in, kb->interruptbuf, len);
    memset(kb->interruptbuf, 0, len);
    if(pthread_mutex_unlock(intmutex(kb)))
        ckb_fatal("Error unlocking interrupt mutex in headset_usb_read()");
    return len;
}

// Builds a headset frame (02 09 <opcode> <payload>, zero-padded to MSG_SIZE) and sends
// it via the generic usbrecv() machinery (which dispatches through headset_usb_write()/
// headset_usb_read() above). resp must be MSG_SIZE bytes. Returns bytes read, or 0 on
// failure.
static int headset_transfer(usbdevice* kb, const uchar* opcode, int opcode_len,
                             const uchar* payload, int payload_len, uchar* resp){
    uchar frame[MSG_SIZE] = {0};
    frame[0] = 0x02;
    frame[1] = HEADSET_ROUTING;
    if(opcode_len > 0)
        memcpy(&frame[2], opcode, opcode_len);
    if(payload_len > 0)
        memcpy(&frame[2 + opcode_len], payload, payload_len);
    return usbrecv(kb, frame, MSG_SIZE, resp);
}

static const uchar cmd_software_mode[]   = { 0x01, 0x03, 0x00, 0x02 };
static const uchar cmd_hardware_mode[]   = { 0x01, 0x03, 0x00, 0x01 };
static const uchar cmd_init_leds[]       = { 0x0d, 0x00, 0x01 };
static const uchar cmd_write_color[]     = { 0x06, 0x00 };
static const uchar cmd_brightness[]      = { 0x01, 0x02, 0x00 };
static const uchar cmd_mic_status[]      = { 0x02, 0x8e };
static const uchar cmd_heartbeat[]       = { 0x12 };
static const uchar cmd_sidetone_mode[]   = { 0x01, 0x46, 0x00 };
static const uchar cmd_sidetone_volume[] = { 0x01, 0x47, 0x00 };

// The device requires a periodic heartbeat roughly every 10s while in software mode,
// or it reverts to hardware mode and the custom colors are lost.
static const struct timespec headset_poll_delay = { .tv_sec = 10 };

static void* headset_poll_thread(void* ctx){
    usbdevice* kb = ctx;
    int ret;
    while(!(ret = clock_nanosleep(CLOCK_MONOTONIC, 0, &headset_poll_delay, NULL))){
        queued_mutex_lock(dmutex(kb));
        if(kb->active)
            headset_keepalive(kb);
        queued_mutex_unlock(dmutex(kb));
    }
    ckb_info("ckb%d: Headset keepalive thread shutting down due to %d (%s)", INDEX_OF(kb, keyboard), ret, strerror(ret));
    return NULL;
}

static int setactive_headset(usbdevice* kb, int active){
    clear_input_and_rgb(kb, active);
    uchar resp[MSG_SIZE];
    if(active){
        if(!headset_transfer(kb, cmd_software_mode, sizeof(cmd_software_mode), NULL, 0, resp))
            return -1;
        // Must run once before writeColor has any visible effect.
        headset_transfer(kb, cmd_init_leds, sizeof(cmd_init_leds), NULL, 0, resp);

        // Start the keepalive poll thread if it's not running already. Reuses the
        // generic kb->pollthread field/lifecycle (see bragi_poll_thread for the same
        // pattern) -- closeusb() already kills/joins/frees it for any device type.
        if(!kb->pollthread){
            kb->pollthread = malloc(sizeof(pthread_t));
            if(kb->pollthread){
                int err = pthread_create(kb->pollthread, 0, headset_poll_thread, kb);
                if(err != 0){
                    ckb_err("ckb%d: Failed to create headset keepalive thread", INDEX_OF(kb, keyboard));
                    free(kb->pollthread);
                    kb->pollthread = NULL;
                }
            } else {
                ckb_err("ckb%d: Failed to allocate memory for headset keepalive thread", INDEX_OF(kb, keyboard));
            }
        }
    } else {
        if(!headset_transfer(kb, cmd_hardware_mode, sizeof(cmd_hardware_mode), NULL, 0, resp))
            return -1;
    }
    return 0;
}

int cmd_active_headset(usbdevice* kb, usbmode* dummy1, int dummy2, int dummy3, const char* dummy4){
    (void)dummy1;
    (void)dummy2;
    (void)dummy3;
    (void)dummy4;

    return setactive_headset(kb, 1);
}

int cmd_idle_headset(usbdevice* kb, usbmode* dummy1, int dummy2, int dummy3, const char* dummy4){
    (void)dummy1;
    (void)dummy2;
    (void)dummy3;
    (void)dummy4;

    return setactive_headset(kb, 0);
}

int headset_get_mic_muted(usbdevice* kb){
    uchar resp[MSG_SIZE];
    if(!headset_transfer(kb, cmd_mic_status, sizeof(cmd_mic_status), NULL, 0, resp))
        return -1;
    return resp[4];
}

// Compare just the 3 zones we use (indices 0-2, matching keymap.c's "zone1"/"zone2"/
// "zone3" entries), ignore the rest of the shared N_KEYS_EXTENDED-sized arrays.
static int rgbcmp_headset(const lighting* lhs, const lighting* rhs){
    return memcmp(lhs->r, rhs->r, 3) || memcmp(lhs->g, rhs->g, 3) || memcmp(lhs->b, rhs->b, 3);
}

int updatergb_headset(usbdevice* kb, int force){
    if(!kb->active)
        return 0;
    lighting* lastlight = &kb->profile->lastlight;
    lighting* newlight = &kb->profile->currentmode->light;
    if(!force && !lastlight->forceupdate && !newlight->forceupdate && !rgbcmp_headset(lastlight, newlight))
        return 0;
    lastlight->forceupdate = newlight->forceupdate = 0;

    // zone1 (index 1) is the mic-mute-ring LED: always show red while muted,
    // regardless of the user's configured color for that zone (matches iCUE).
    uchar mic_r = newlight->r[1], mic_g = newlight->g[1], mic_b = newlight->b[1];
    if(headset_get_mic_muted(kb) == 1){
        mic_r = 0xff;
        mic_g = 0x00;
        mic_b = 0x00;
    }

    // Wire format groups bytes BY CHANNEL, not by zone: [R0,R2,R1, G0,G2,G1, B0,B2,B1]
    // where zone0=logo (index 0), zone1=mic ring (index 1), zone2=power indicator
    // (index 2), matching keymap.c's "zone1"/"zone2"/"zone3" led=0/1/2 entries.
    uchar payload[13] = {
        0x09, 0x00, 0x00, 0x00,
        newlight->r[0], newlight->r[2], mic_r,
        newlight->g[0], newlight->g[2], mic_g,
        newlight->b[0], newlight->b[2], mic_b,
    };

    uchar resp[MSG_SIZE];
    if(!headset_transfer(kb, cmd_write_color, sizeof(cmd_write_color), payload, sizeof(payload), resp))
        return -1;
    memcpy(lastlight, newlight, sizeof(lighting));
    return 0;
}

int headset_set_brightness(usbdevice* kb, ushort value){
    if(value > 1000)
        value = 1000;
    uchar payload[2] = { (uchar)(value & 0xff), (uchar)(value >> 8) };
    uchar resp[MSG_SIZE];
    return headset_transfer(kb, cmd_brightness, sizeof(cmd_brightness), payload, sizeof(payload), resp) ? 0 : -1;
}

void headset_keepalive(usbdevice* kb){
    uchar resp[MSG_SIZE];
    headset_transfer(kb, cmd_heartbeat, sizeof(cmd_heartbeat), NULL, 0, resp);
}

int headset_set_sidetone_enabled(usbdevice* kb, int enabled){
    uchar payload[1] = { enabled ? 0x00 : 0x01 }; // inverted: 0x00=on, 0x01=off
    uchar resp[MSG_SIZE];
    return headset_transfer(kb, cmd_sidetone_mode, sizeof(cmd_sidetone_mode), payload, sizeof(payload), resp) ? 0 : -1;
}

int headset_set_sidetone_volume(usbdevice* kb, uchar value_0_100){
    if(value_0_100 > 100)
        value_0_100 = 100;
    ushort scaled = (ushort)(value_0_100 * 10); // 0-100 -> 0-1000
    uchar payload[2] = { (uchar)(scaled & 0xff), (uchar)(scaled >> 8) };
    uchar resp[MSG_SIZE];
    return headset_transfer(kb, cmd_sidetone_volume, sizeof(cmd_sidetone_volume), payload, sizeof(payload), resp) ? 0 : -1;
}
