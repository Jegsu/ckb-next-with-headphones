#include "structures.h"

// Low-level transport (vtable .write/.read/.fill_input_eps)
void headset_fill_input_eps(usbdevice* kb);
int headset_usb_write(usbdevice* kb, void* out, int len, int is_recv, const char* file, int line);
int headset_usb_read(usbdevice* kb, void* in, int len, int dummy, const char* file, int line);

// vtable .active/.idle
int cmd_active_headset(usbdevice* kb, usbmode* dummy1, int dummy2, int dummy3, const char* dummy4);
int cmd_idle_headset(usbdevice* kb, usbmode* dummy1, int dummy2, int dummy3, const char* dummy4);

// vtable .updatergb
int updatergb_headset(usbdevice* kb, int force);

// Extra headset-specific controls, not part of the standard vtable
int headset_get_mic_muted(usbdevice* kb);
int headset_set_brightness(usbdevice* kb, unsigned short value);
void headset_keepalive(usbdevice* kb);
int headset_set_sidetone_enabled(usbdevice* kb, int enabled);
int headset_set_sidetone_volume(usbdevice* kb, uchar value_0_100);
