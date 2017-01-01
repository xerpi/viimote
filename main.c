#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/bt.h>
#include <psp2kern/ctrl.h>
#include <taihen.h>
#include "log.h"

extern int ksceKernelPowerTick(int);

#define WIIMOTE_VID 0x057E
#define WIIMOTE_OLD_PID 0x0306
#define WIIMOTE_NEW_PID 0x0330

#define NUNCHUK_ANALOG_X_MIN 35
#define NUNCHUK_ANALOG_X_MAX 228
#define NUNCHUK_ANALOG_X_RANGE (NUNCHUK_ANALOG_X_MAX - NUNCHUK_ANALOG_X_MIN)
#define NUNCHUK_ANALOG_Y_MIN 27
#define NUNCHUK_ANALOG_Y_MAX 220
#define NUNCHUK_ANALOG_Y_RANGE (NUNCHUK_ANALOG_Y_MAX - NUNCHUK_ANALOG_Y_MIN)
#define NUNCHUK_ANALOG_THRESHOLD 5

/* Grabbed from: https://github.com/abstrakraft/cwiid */
#define RPT_LED_RUMBLE		0x11
#define RPT_RPT_MODE		0x12
#define RPT_STATUS_REQ		0x15
#define RPT_WRITE		0x16
#define RPT_READ_REQ		0x17

#define RPT_MODE_BUF_LEN	2
#define RPT_READ_REQ_LEN	6
#define RPT_WRITE_LEN		21

#define RPT_BTN			0x30
#define RPT_BTN_ACC		0x31
#define RPT_BTN_EXT8		0x32

#define RW_EEPROM		0x00
#define RW_REG			0x04
#define RW_DECODE		0x00

/* Button flags */
#define BTN_LEFT		0x0001
#define BTN_RIGHT		0x0002
#define BTN_DOWN		0x0004
#define BTN_UP			0x0008
#define BTN_PLUS		0x0010
#define BTN_2			0x0100
#define BTN_1			0x0200
#define BTN_B			0x0400
#define BTN_A			0x0800
#define BTN_MINUS		0x1000
#define BTN_HOME		0x8000

#define NUNCHUK_BTN_Z		0x01
#define NUNCHUK_BTN_C		0x02

/* Extension Values */
#define EXT_NONE		0x2E2E
#define EXT_PARTIAL		0xFFFF
#define EXT_NUNCHUK		0x0000
#define EXT_CLASSIC		0x0101
#define EXT_BALANCE		0x0402
#define EXT_MOTIONPLUS		0x0405

#define abs(x) (((x) < 0) ? -(x) : (x))

static SceUID bt_mempool_uid = -1;
static SceUID bt_thread_uid = -1;
static SceUID bt_cb_uid = -1;
static int bt_thread_run = 1;

enum wiimote_ext_type {
	WIIMOTE_EXT_NONE,
	WIIMOTE_EXT_NUNCHUK,
	WIIMOTE_EXT_CLASSIC,
	WIIMOTE_EXT_BALANCE,
	WIIMOTE_EXT_MOTIONPLUS,
	WIIMOTE_EXT_UNKNOWN
};

struct wiimote_info {
	int connected;
	unsigned int mac0;
	unsigned int mac1;
	int init_ext_step;
	enum wiimote_ext_type extension;
	unsigned short buttons;

	union {
		struct {
			unsigned char sx;
			unsigned char sy;
			unsigned short ax;
			unsigned short ay;
			unsigned short az;
			unsigned char buttons;
		} nunchuk;

		struct {
			unsigned short buttons;
		} classic;
	};
};

static struct wiimote_info wiimote;

static tai_hook_ref_t SceBt_sub_22999C8_ref;
static SceUID SceBt_sub_22999C8_hook_uid = -1;
static tai_hook_ref_t SceBt_sub_228C3F0_ref;
static SceUID SceBt_sub_228C3F0_hook_uid = -1;

static inline void wiimote_extension_nunchuk_reset(struct wiimote_info *wiimote)
{
	wiimote->nunchuk.sx = 1 << 7;
	wiimote->nunchuk.sy = 1 << 7;
	wiimote->nunchuk.ax = 1 << 9;
	wiimote->nunchuk.ay = 1 << 9;
	wiimote->nunchuk.az = 1 << 9;
	wiimote->nunchuk.buttons = 0;
}

static inline void wiimote_extension_classic_reset(struct wiimote_info *wiimote)
{
	/*wiimote->classic.sx = 1 << 7;
	wiimote->classic.sy = 1 << 7;
	wiimote->classic.ax = 1 << 9;
	wiimote->classic.ay = 1 << 9;
	wiimote->classic.az = 1 << 9;*/
	wiimote->classic.buttons = 0;
}

static void wiimote_info_reset(struct wiimote_info *wiimote)
{
	memset(wiimote, 0, sizeof(*wiimote));
}

static int is_wiimote(const unsigned short vid_pid[2], const char *name)
{
	if (vid_pid[0] == WIIMOTE_VID &&
	    (vid_pid[1] == WIIMOTE_OLD_PID || vid_pid[1] == WIIMOTE_NEW_PID)) {
		return 1;
	}

	if (!strcmp(name, "Nintendo RVL-CNT-01"))
		return 1;
	else if (!strcmp(name, "Nintendo RVL-CNT-01-TR"))
		return 1;

	return 0;
}

static inline void *mempool_alloc(unsigned int size)
{
	return ksceKernelMemPoolAlloc(bt_mempool_uid, size);
}

static inline void mempool_free(void *ptr)
{
	ksceKernelMemPoolFree(bt_mempool_uid, ptr);
}

static int wiimote_send_rpt(unsigned int mac0, unsigned int mac1, uint8_t flags, uint8_t report,
			    size_t len, const void *data)
{
	SceBtHidRequest *req;
	unsigned char *buf;

	req = mempool_alloc(sizeof(*req));
	if (!req) {
		LOG("Error allocatin BT HID Request\n");
		return -1;
	}

	if ((buf = mempool_alloc((len + 1) * sizeof(*buf))) == NULL) {
		LOG("Memory allocation error (mesg array)\n");
		return -1;
	}

	buf[0] = report;
	memcpy(buf + 1, data, len);

	memset(req, 0, sizeof(*req));
	req->type = 1; // 0xA2 -> type = 1
	req->buffer = buf;
	req->length = len + 1;
	req->next = req;

	ksceBtHidTransfer(mac0, mac1, req);

	mempool_free(buf);
	mempool_free(req);

	return 0;
}

static int wiimote_request_status(unsigned int mac0, unsigned int mac1)
{
	unsigned char data;

	data = 0;
	if (wiimote_send_rpt(mac0, mac1, 0, RPT_STATUS_REQ, 1, &data)) {
		LOG("Status request error\n");
		return -1;
	}

	return 0;
}

static int wiimote_set_rpt_type(unsigned int mac0, unsigned int mac1, uint8_t rpt_type)
{
	unsigned char buf[RPT_MODE_BUF_LEN];
	const int continuous = 0;

	buf[0] = continuous ? 0x04 : 0;
	buf[1] = rpt_type;
	if (wiimote_send_rpt(mac0, mac1, 0, RPT_RPT_MODE, RPT_MODE_BUF_LEN, buf)) {
		LOG("Send report error (report mode)");
		return -1;
	}

	return 0;
}

static int wiimote_set_led(unsigned int mac0, unsigned int mac1, uint8_t led)
{
	unsigned char data;

	data = (led & 0x0F) << 4;
	if (wiimote_send_rpt(mac0, mac1, 0, RPT_LED_RUMBLE, 1, &data)) {
		LOG("Report send error (led)\n");
		return -1;
	}

	return 0;
}

static int wiimote_request_mem_data_read(unsigned int mac0, unsigned int mac1, uint8_t flags,
					 uint32_t offset, uint16_t len)
{
	unsigned char buf[RPT_READ_REQ_LEN];

	/* Compose read request packet */
	buf[0] = flags & (RW_EEPROM | RW_REG);
	buf[1] = (unsigned char)((offset >> 16) & 0xFF);
	buf[2] = (unsigned char)((offset >> 8) & 0xFF);
	buf[3] = (unsigned char)(offset & 0xFF);
	buf[4] = (unsigned char)((len >> 8) & 0xFF);
	buf[5] = (unsigned char)(len & 0xFF);

	if (wiimote_send_rpt(mac0, mac1, 0, RPT_READ_REQ, RPT_READ_REQ_LEN, buf)) {
		LOG("Report send error (read)\n");
		return -1;
	}

	return 0;
}

static int wiimote_request_mem_data_write(unsigned int mac0, unsigned int mac1, uint8_t flags,
					 uint32_t offset, uint16_t len, const void *data)
{
	unsigned char buf[RPT_WRITE_LEN];

	/* Compose write packet */
	buf[0] = flags;
	buf[1] = (unsigned char)((offset >> 16) & 0xFF);
	buf[2] = (unsigned char)((offset >> 8) & 0xFF);
	buf[3] = (unsigned char)(offset & 0xFF);
	buf[4] = (unsigned char)len;
	memcpy(buf + 5, data, len);

	if (wiimote_send_rpt(mac0, mac1, 0, RPT_WRITE, RPT_WRITE_LEN, buf)) {
		LOG("Report send error (read)\n");
		return -1;
	}

	return 0;
}

static int SceBt_sub_22999C8_hook_func(void *dev_base_ptr, int r1)
{
	unsigned int flags = *(unsigned int *)(r1 + 4);

	if (dev_base_ptr && !(flags & 2)) {
		const void *dev_info = *(const void **)(dev_base_ptr + 0x14A4);
		const unsigned short *vid_pid = (const unsigned short *)(dev_info + 0x28);
		const char *name = (const char *)(dev_info + 0x80);

		if (is_wiimote(vid_pid, name)) {
			unsigned int *v8_ptr = (unsigned int *)(*(unsigned int *)dev_base_ptr + 8);

			/*
			 * We need to enable the following bits in order to make the Vita
			 * accept the new connection, otherwise it will refuse it.
			 */
			*v8_ptr |= 0x11000;
		}
	}

	return TAI_CONTINUE(int, SceBt_sub_22999C8_ref, dev_base_ptr, r1);
}

static int SceBt_sub_228C3F0_hook_func(void *base_ptr)
{
	int ret;
	unsigned int old_value;
	int patched = 0;
	unsigned int flags = *(unsigned int *)(base_ptr + 4);
	const void *dev_info = *(const void **)(base_ptr + 0x14A4);

	if (!(flags & 0x200) && !(flags & 2)) {
		const unsigned short *vid_pid = (const unsigned short *)(dev_info + 0x28);
		const char *name = (const char *)(dev_info + 0x80);

		if (is_wiimote(vid_pid, name)) {
			/*
			 * We need to set the following bits in order to make the Vita
			 * request a PIN code.
			 */
			old_value = *(unsigned int *)(dev_info + 0x20);
			*(unsigned int *)(dev_info + 0x20) &= ~0x40;
			patched = 1;
		}
	}

	ret = TAI_CONTINUE(int, SceBt_sub_228C3F0_ref, base_ptr);

	if (patched) {
		*(unsigned int *)(dev_info + 0x20) = old_value;
	}

	return ret;
}

static void reset_input_emulation()
{
	ksceCtrlSetButtonEmulation(0, 0, 0, 0, 32);
	ksceCtrlSetAnalogEmulation(0, 0, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0);
}

static void set_input_emulation()
{
	unsigned int buttons = 0;
	int js_moved = 0;

	if (wiimote.buttons & BTN_LEFT)
		buttons |= SCE_CTRL_LEFT;
	if (wiimote.buttons & BTN_RIGHT)
		buttons |= SCE_CTRL_RIGHT;
	if (wiimote.buttons & BTN_DOWN)
		buttons |= SCE_CTRL_DOWN;
	if (wiimote.buttons & BTN_UP)
		buttons |= SCE_CTRL_UP;
	if (wiimote.buttons & BTN_PLUS)
		buttons |= SCE_CTRL_START;
	if (wiimote.buttons & BTN_2)
		buttons |= SCE_CTRL_TRIANGLE;
	if (wiimote.buttons & BTN_1)
		buttons |= SCE_CTRL_SQUARE;
	if (wiimote.buttons & BTN_B)
		buttons |= SCE_CTRL_CIRCLE;
	if (wiimote.buttons & BTN_A)
		buttons |= SCE_CTRL_CROSS;
	if (wiimote.buttons & BTN_MINUS)
		buttons |= SCE_CTRL_SELECT;
	if (wiimote.buttons & BTN_HOME)
		buttons |= SCE_CTRL_INTERCEPTED;

	switch (wiimote.extension) {
	case WIIMOTE_EXT_NUNCHUK: {
		unsigned char lx;
		unsigned char ly;
		unsigned char sx = wiimote.nunchuk.sx;
		unsigned char sy = 255 - wiimote.nunchuk.sy;

		if (sx > NUNCHUK_ANALOG_X_MAX)
			lx = 255;
		else if (sx < NUNCHUK_ANALOG_X_MIN)
			lx = 0;
		else
			lx = ((sx - NUNCHUK_ANALOG_X_MIN) * 255) / NUNCHUK_ANALOG_X_RANGE;

		if (sy > NUNCHUK_ANALOG_Y_MAX)
			ly = 255;
		else if (sx < NUNCHUK_ANALOG_Y_MIN)
			ly = 0;
		else
			ly = ((sy - NUNCHUK_ANALOG_Y_MIN) * 255) / NUNCHUK_ANALOG_Y_RANGE;

		if ((abs((signed char)lx - 128) > NUNCHUK_ANALOG_THRESHOLD) ||
		    (abs((signed char)ly - 128) > NUNCHUK_ANALOG_THRESHOLD)) {
			js_moved = 1;
		}

		if (wiimote.nunchuk.buttons & NUNCHUK_BTN_Z)
			buttons |= SCE_CTRL_R1;
		if (wiimote.nunchuk.buttons & NUNCHUK_BTN_C)
			buttons |= SCE_CTRL_L1;

		ksceCtrlSetAnalogEmulation(0, 0, lx, ly, 0, 0, 0, 0, 0, 0, 32);
		break;
	}

	case WIIMOTE_EXT_CLASSIC: {


		break;
	}

	default:
		break;
	}

	ksceCtrlSetButtonEmulation(0, 0, buttons, buttons, 32);

	if (buttons != 0 || js_moved)
		ksceKernelPowerTick(0);
}

static void enqueue_read_request(unsigned int mac0, unsigned int mac1,
				 SceBtHidRequest *request, unsigned char *buffer,
				 unsigned int length)
{
	memset(request, 0, sizeof(*request));
	memset(buffer, 0, length);

	request->type = 0;
	request->buffer = buffer;
	request->length = length;
	request->next = request;

	ksceBtHidTransfer(mac0, mac1, request);
}

static int bt_cb_func(int notifyId, int notifyCount, int notifyArg, void *common)
{
	static SceBtHidRequest hid_request;
	static unsigned char recv_buff[0x100];

	while (1) {
		int ret;
		SceBtEvent hid_event;

		memset(&hid_event, 0, sizeof(hid_event));

		do {
			ret = ksceBtReadEvent(&hid_event, 1);
		} while (ret == SCE_BT_ERROR_CB_OVERFLOW);

		if (ret <= 0) {
			break;
		}

		LOG("->Event:");
		for (int i = 0; i < 0x10; i++)
			LOG(" %02X", hid_event.data[i]);
		LOG("\n");

		/*
		 * If we get an event with a MAC, and the MAC is different
		 * from the connected Wiimote, skip the event.
		 */
		if (wiimote.connected) {
			if (hid_event.mac0 != wiimote.mac0 || hid_event.mac1 != wiimote.mac1)
				continue;
		}

		switch (hid_event.id) {
		case 0x01: { /* Inquiry result event */
			unsigned short vid_pid[2];
			char name[0x79];

			ksceBtGetVidPid(hid_event.mac0, hid_event.mac1, vid_pid);
			ksceBtGetDeviceName(hid_event.mac0, hid_event.mac1, name);

			if (is_wiimote(vid_pid, name)) {
				ksceBtStopInquiry();
				wiimote.mac0 = hid_event.mac0;
				wiimote.mac1 = hid_event.mac1;
			}
			break;
		}

		case 0x02: /* Inquiry stop event */
			if (!wiimote.connected) {
				if (wiimote.mac0 || wiimote.mac1)
					ksceBtStartConnect(wiimote.mac0, wiimote.mac1);
			}

			break;

		case 0x03: { /* Pin request event */
			unsigned char pin[6];

			/*
			 * When connecting using 1+2, the PIN code is the
			 * MAC address of the Wiimote backwards.
			 */
			pin[0] = (hid_event.mac0 >> 0) & 0xFF;
			pin[1] = (hid_event.mac0 >> 8) & 0xFF;
			pin[2] = (hid_event.mac0 >> 16) & 0xFF;
			pin[3] = (hid_event.mac0 >> 24) & 0xFF;
			pin[4] = (hid_event.mac1 >> 0) & 0xFF;
			pin[5] = (hid_event.mac1 >> 8) & 0xFF;

			ksceBtReplyPinCode(hid_event.mac0, hid_event.mac1,
				pin, sizeof(pin));

			break;
		}

		case 0x05: /* Connection accepted event */
			wiimote_set_led(hid_event.mac0, hid_event.mac1, 1);
			wiimote.mac0 = hid_event.mac0;
			wiimote.mac1 = hid_event.mac1;
			wiimote.extension = WIIMOTE_EXT_NONE;
			wiimote.connected = 1;
			break;

		case 0x06: /* Device disconnect event*/
			wiimote.connected = 0;
			reset_input_emulation();
			break;

		case 0x08: /* Connection requested event */
			/*
			 * Do nothing since we will get a 0x05 event afterwards.
			 */
			break;

		case 0x09: /* Connect request without being paired? event */
			/*
			 * The Vita needs to have a pairing with the Wiimote,
			 * otherwise it won't connect.
			 */
			break;

		case 0x0A: /* HID reply to 0-type request */

			LOG("Wiimote 0x0A event: 0x%02X\n", recv_buff[0]);

			switch (recv_buff[0]) {
			case 0x20: /* Status */
				/* Extension connected */
				if (recv_buff[3] & 0x02) {
					LOG("Extension connected!\n");

					/* Read extension ID */
					if (wiimote_request_mem_data_read(hid_event.mac0, hid_event.mac1, RW_REG, 0xA400FE, 2)) {
						LOG("Read error (extension error)\n");
						break;
					}
				} else {
					wiimote.extension = WIIMOTE_EXT_NONE;
					reset_input_emulation();
					wiimote_set_rpt_type(hid_event.mac0, hid_event.mac1, RPT_BTN);
				}

				break;

			case 0x21: /* Read Memory and Registers Data */
				switch ((recv_buff[6] << 8) | recv_buff[7]) {
				case EXT_NONE:
					LOG("No extension\n");
					wiimote.extension = WIIMOTE_EXT_NONE;
					wiimote_set_rpt_type(hid_event.mac0, hid_event.mac1, RPT_BTN);
					break;
				case EXT_NUNCHUK:
					LOG("Nunchuk extension\n");
					wiimote_extension_nunchuk_reset(&wiimote);
					wiimote.extension = WIIMOTE_EXT_NUNCHUK;
					wiimote_set_rpt_type(hid_event.mac0, hid_event.mac1, RPT_BTN_EXT8);
					break;

				case EXT_CLASSIC:
					LOG("Classic controller extension\n");
					wiimote_extension_classic_reset(&wiimote);
					wiimote.extension = WIIMOTE_EXT_CLASSIC;
					wiimote_set_rpt_type(hid_event.mac0, hid_event.mac1, RPT_BTN_EXT8);
					break;

				case EXT_PARTIAL: {
					LOG("Partial extension\n");
					unsigned char buf[1];

					/* Initialize extension register space */
					buf[0] = 0x55;
					if (wiimote_request_mem_data_write(hid_event.mac0, hid_event.mac1, RW_REG, 0xA400F0, 1, buf)) {
						LOG("Extension initialization error\n");
						break;
					}

					wiimote.init_ext_step = 1;

					break;
				}

				default:
					LOG("Unknown extension: %02X%02X\n",
						recv_buff[7], recv_buff[6]);
					break;
				}
				break;

			case 0x22: /* Acknowledge output report, return function result */
				if (wiimote.init_ext_step == 1) {
					unsigned char buf[1];

					buf[0] = 0x00;
					if (wiimote_request_mem_data_write(hid_event.mac0, hid_event.mac1, RW_REG, 0xA400FB, 1, buf)) {
						LOG("Extension initialization error\n");
						break;
					}

					wiimote.init_ext_step = 2;
				} else if (wiimote.init_ext_step == 2) {
					/* Read extension ID */
					if (wiimote_request_mem_data_read(hid_event.mac0, hid_event.mac1, RW_REG, 0xA400FE, 2)) {
						LOG("Read error (extension error)\n");
						break;
					}

					wiimote.init_ext_step = 0;
				}
				break;

			case 0x30: /* Core Buttons */
				wiimote.buttons = (recv_buff[2] << 8) | recv_buff[1];

				set_input_emulation();

				enqueue_read_request(hid_event.mac0, hid_event.mac1,
					&hid_request, recv_buff, sizeof(recv_buff));

				break;

			case 0x32: /* Core Buttons with 8 Extension bytes */
				wiimote.buttons = (recv_buff[2] << 8) | recv_buff[1];

				switch (wiimote.extension) {
				case WIIMOTE_EXT_NUNCHUK:
					wiimote.nunchuk.sx = recv_buff[3];
					wiimote.nunchuk.sy = recv_buff[4];
					wiimote.nunchuk.ax = (recv_buff[5] << 2) | (recv_buff[8] >> 2);
					wiimote.nunchuk.ay = (recv_buff[6] << 2) | (recv_buff[8] >> 4);
					wiimote.nunchuk.az = (recv_buff[7] << 2) | (recv_buff[8] >> 6);
					wiimote.nunchuk.buttons = ~recv_buff[8] & 3;
					break;

				case WIIMOTE_EXT_CLASSIC:
					break;

				default:
					break;
				}

				set_input_emulation();

				enqueue_read_request(hid_event.mac0, hid_event.mac1,
					&hid_request, recv_buff, sizeof(recv_buff));

				break;

			default:
				LOG("Unknown Wiimote event: 0x%02X\n", recv_buff[0]);
				break;
			}

			break;

		case 0x0B: /* HID reply to 1-type request */

			//LOG("Wiimote 0x0B event: 0x%02X\n", recv_buff[0]);

			enqueue_read_request(hid_event.mac0, hid_event.mac1,
				&hid_request, recv_buff, sizeof(recv_buff));

			break;
		}
	}

	return 0;
}

static int viimote_bt_thread(SceSize args, void *argp)
{
	bt_cb_uid = ksceKernelCreateCallback("viimote_bt_callback", 0, bt_cb_func, NULL);

	wiimote_info_reset(&wiimote);

	ksceBtRegisterCallback(bt_cb_uid, 0, 0xFFFFFFFF, 0xFFFFFFFF);

/*#ifndef RELEASE
	ksceBtStartInquiry();
	ksceKernelDelayThreadCB(2 * 1000 * 1000);
	ksceBtStopInquiry();
#endif*/

	while (bt_thread_run) {
		ksceKernelDelayThreadCB(200 * 1000);
	}

	if (wiimote.connected) {
		ksceBtStartDisconnect(wiimote.mac0, wiimote.mac1);
		wiimote_info_reset(&wiimote);
		reset_input_emulation();
	}

	ksceBtUnregisterCallback(bt_cb_uid);

	ksceKernelDeleteCallback(bt_cb_uid);

	return 0;
}

void _start() __attribute__ ((weak, alias ("module_start")));

int module_start(SceSize argc, const void *args)
{
	int ret;
	tai_module_info_t SceBt_modinfo;

	log_reset();

	LOG("viimote by xerpi\n");

	SceBt_modinfo.size = sizeof(SceBt_modinfo);
	ret = taiGetModuleInfoForKernel(KERNEL_PID, "SceBt", &SceBt_modinfo);
	if (ret < 0) {
		LOG("Error finding SceBt module\n");
		goto error_find_scebt;
	}

	/* SceBt hooks */
	SceBt_sub_22999C8_hook_uid = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&SceBt_sub_22999C8_ref, SceBt_modinfo.modid, 0,
		0x22999C8 - 0x2280000, 1, SceBt_sub_22999C8_hook_func);

	SceBt_sub_228C3F0_hook_uid = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&SceBt_sub_228C3F0_ref, SceBt_modinfo.modid, 0,
		0x228C3F0 - 0x2280000, 1, SceBt_sub_228C3F0_hook_func);

	SceKernelMemPoolCreateOpt opt;
	opt.size = 0x1C;
	opt.uselock = 0x100;
	opt.field_8 = 0x10000;
	opt.field_C = 0;
	opt.field_10 = 0;
	opt.field_14 = 0;
	opt.field_18 = 0;

	bt_mempool_uid = ksceKernelMemPoolCreate("viimote_mempool", 0x100, &opt);
	LOG("Bluetooth mempool UID: 0x%08X\n", bt_mempool_uid);

	bt_thread_uid = ksceKernelCreateThread("viimote_bt_thread", viimote_bt_thread,
		0x3C, 0x1000, 0, 0x10000, 0);
	LOG("Bluetooth thread UID: 0x%08X\n", bt_thread_uid);
	ksceKernelStartThread(bt_thread_uid, 0, NULL);

	LOG("module_start finished successfully!\n");

	return SCE_KERNEL_START_SUCCESS;

error_find_scebt:
	return SCE_KERNEL_START_FAILED;
}

int module_stop(SceSize argc, const void *args)
{
	SceUInt timeout = 0xFFFFFFFF;

	if (bt_thread_uid > 0) {
		bt_thread_run = 0;
		ksceKernelWaitThreadEnd(bt_thread_uid, NULL, &timeout);
		ksceKernelDeleteThread(bt_thread_uid);
	}

	if (bt_mempool_uid > 0) {
		ksceKernelMemPoolDestroy(bt_mempool_uid);
	}

	if (SceBt_sub_22999C8_hook_uid > 0) {
		taiHookReleaseForKernel(SceBt_sub_22999C8_hook_uid,
			SceBt_sub_22999C8_ref);
	}

	if (SceBt_sub_228C3F0_hook_uid > 0) {
		taiHookReleaseForKernel(SceBt_sub_228C3F0_hook_uid,
			SceBt_sub_228C3F0_ref);
	}

	log_flush();

	return SCE_KERNEL_STOP_SUCCESS;
}
