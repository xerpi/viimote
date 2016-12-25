#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/bt.h>
#include <psp2/ctrl.h>
#include <taihen.h>
#include "log.h"

/* Imported from taiHEN */
extern int module_get_offset(SceUID pid, SceUID modid, int segidx, size_t offset, uintptr_t *addr);

#define WIIMOTE_VID 0x057E
#define WIIMOTE_PID 0x0306

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

/* Extension Values */
#define EXT_NONE		0x2E2E
#define EXT_PARTIAL		0xFFFF
#define EXT_NUNCHUK		0x0000
#define EXT_CLASSIC		0x0101
#define EXT_BALANCE		0x0402
#define EXT_MOTIONPLUS		0x0405

static SceUID bt_mempool_uid = -1;
static SceUID bt_thread_uid = -1;
static SceUID bt_cb_uid = -1;
static int bt_thread_run = 1;

static int wiimote_connected = 0;
static unsigned int wiimote_mac0 = 0;
static unsigned int wiimote_mac1 = 0;
static unsigned short wiimote_buttons = 0;
static int wiimote_init_ext_step = 0;
static int wiimote_nunchuk_connected = 0;
static struct {
	unsigned char sx;
	unsigned char sy;
	unsigned short ax;
	unsigned short ay;
	unsigned short az;
	unsigned char bc;
	unsigned char bz;
} wiimote_nunchuk_data = { 0 };

static tai_hook_ref_t SceBt_sub_22999C8_ref;
static SceUID SceBt_sub_22999C8_hook_uid = -1;
static tai_hook_ref_t SceCtrl_sceCtrlPeekBufferPositive_ref;
static SceUID SceCtrl_sceCtrlPeekBufferPositive_hook_uid = -1;
static tai_hook_ref_t SceCtrl_sceCtrlPeekBufferPositive2_ref;
static SceUID SceCtrl_sceCtrlPeekBufferPositive2_hook_uid = -1;

static inline void *mempool_alloc(unsigned int size)
{
	return ksceKernelMemPoolAlloc(bt_mempool_uid, size);
}

static inline void mempool_free(void *ptr)
{
	ksceKernelMemPoolFree(bt_mempool_uid, ptr);
}

static int vita_get_mac(unsigned char mac[6])
{
	int ret;
	uintptr_t addr;
	tai_module_info_t SceBt_modinfo;

	SceBt_modinfo.size = sizeof(SceBt_modinfo);
	ret = taiGetModuleInfoForKernel(KERNEL_PID, "SceBt", &SceBt_modinfo);
	if (ret < 0)
		return 0;

	addr = 0;
	ret = module_get_offset(KERNEL_PID, SceBt_modinfo.modid, 1, 0x453A4, &addr);
	if (ret < 0)
		return 0;

	if (addr != 0) {
		memcpy(mac, (void *)addr, 6);
		return 1;
	}

	return 0;
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

static int SceBt_sub_22999C8_hook_func(int r0, int r1)
{
	unsigned int foo = *(unsigned int *)(r1 + 4);
	void *v2 = (void **)r0;

	if (v2 && !(foo & 2)) {
		unsigned int v5 = *(unsigned int *)(v2 + 0x14A4);
		unsigned short vid = *(unsigned short *)(v5 + 0x28);
		unsigned short pid = *(unsigned short *)(v5 + 0x2A);

		if (vid == WIIMOTE_VID && pid == WIIMOTE_PID) {
			unsigned int *v8_ptr = (unsigned int *)(*(unsigned int *)v2 + 8);

			/*
			 * We need to enable the following bits in order to make the Vita
			 * accept the new connection, otherwise it will refuse it.
			 */
			*v8_ptr |= 0x11000;
		}
	}

	return TAI_CONTINUE(int, SceBt_sub_22999C8_ref, r0, r1);
}

static void patch_ctrldata_positive(SceCtrlData *pad_data, int count, unsigned short buttons)
{
	int i;
	SceCtrlData *upad_data = pad_data;

	for (i = 0; i < count; i++) {
		SceCtrlData kpad_data;

		ksceKernelMemcpyUserToKernel(&kpad_data, (uintptr_t)upad_data, sizeof(kpad_data));

		if (buttons & (1 << 0))
			kpad_data.buttons |= SCE_CTRL_LEFT;
		if (buttons & (1 << 1))
			kpad_data.buttons |= SCE_CTRL_RIGHT;
		if (buttons & (1 << 2))
			kpad_data.buttons |= SCE_CTRL_DOWN;
		if (buttons & (1 << 3))
			kpad_data.buttons |= SCE_CTRL_UP;
		if (buttons & (1 << 4))
			kpad_data.buttons |= SCE_CTRL_START;
		if (buttons & (1 << 8))
			kpad_data.buttons |= SCE_CTRL_TRIANGLE;
		if (buttons & (1 << 9))
			kpad_data.buttons |= SCE_CTRL_SQUARE;
		if (buttons & (1 << 10))
			kpad_data.buttons |= SCE_CTRL_CIRCLE;
		if (buttons & (1 << 11))
			kpad_data.buttons |= SCE_CTRL_CROSS;
		if (buttons & (1 << 12))
			kpad_data.buttons |= SCE_CTRL_START;
		if (buttons & (1 << 15))
			kpad_data.buttons |= SCE_CTRL_INTERCEPTED;

		if (wiimote_nunchuk_connected) {
			kpad_data.lx = wiimote_nunchuk_data.sx;
			kpad_data.ly = 255 - wiimote_nunchuk_data.sy;

			if (wiimote_nunchuk_data.bz)
				kpad_data.buttons |= SCE_CTRL_R1;
			if (wiimote_nunchuk_data.bc)
				kpad_data.buttons |= SCE_CTRL_L1;
		}

		ksceKernelMemcpyKernelToUser((uintptr_t)upad_data, &kpad_data, sizeof(kpad_data));

		upad_data++;
	}
}

static int SceCtrl_sceCtrlPeekBufferPositive_hook_func(int port, SceCtrlData *pad_data, int count)
{
	int ret;

	ret = TAI_CONTINUE(int, SceCtrl_sceCtrlPeekBufferPositive_ref, port, pad_data, count);

	if (ret >= 0 && wiimote_connected) {
		patch_ctrldata_positive(pad_data, count, wiimote_buttons);
	}

	return ret;
}

static int SceCtrl_sceCtrlPeekBufferPositive2_hook_func(int port, SceCtrlData *pad_data, int count)
{
	int ret;

	ret = TAI_CONTINUE(int, SceCtrl_sceCtrlPeekBufferPositive2_ref, port, pad_data, count);

	if (ret >= 0 && wiimote_connected) {
		patch_ctrldata_positive(pad_data, count, wiimote_buttons);
	}

	return ret;
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

		switch (hid_event.id) {
		case 0x01: { /* Inquiry result event */
			unsigned short vid_pid[2];
			ksceBtGetVidPid(hid_event.mac0, hid_event.mac1, vid_pid);

			if (vid_pid[0] == WIIMOTE_VID && vid_pid[1] == WIIMOTE_PID) {
				ksceBtStopInquiry();
				wiimote_mac0 = hid_event.mac0;
				wiimote_mac1 = hid_event.mac1;
			}
			break;
		}

		case 0x02: /* Inquiry stop event */
			if (wiimote_mac0 || wiimote_mac1) {
				ksceBtStartConnect(wiimote_mac0, wiimote_mac1);
			}
			break;

		case 0x03: { /* Pin request event */
			unsigned char mac[6];

			if (vita_get_mac(mac)) {
				int i;
				unsigned char pin[6];

				for (i = 0; i < 6; i++)
					pin[i] = mac[5 - i];

				ksceBtReplyPinCode(hid_event.mac0, hid_event.mac1,
					pin, sizeof(pin));
			}

			break;
		}

		case 0x08: /* Connection requested event */
			/*
			 * Do nothing since we will get a 0x05 event afterwards.
			 */
			break;

		case 0x05: /* Connection accepted event */
			wiimote_set_led(hid_event.mac0, hid_event.mac1, 1);
			wiimote_connected = 1;
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
					wiimote_nunchuk_connected = 0;
				}

				break;

			case 0x21: /* Read Memory and Registers Data */
				switch ((recv_buff[6] << 8) | recv_buff[7]) {
				case EXT_NONE:
					LOG("No extension\n");
					wiimote_set_rpt_type(hid_event.mac0, hid_event.mac1, RPT_BTN);
					break;
				case EXT_NUNCHUK:
					LOG("Nunchuk extension\n");
					wiimote_nunchuk_connected = 1;
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

					wiimote_init_ext_step = 1;

					break;
				}

				default:
					LOG("Unknown extension: %02X%02X\n",
						recv_buff[7], recv_buff[6]);
					break;
				}
				break;

			case 0x22: /* Acknowledge output report, return function result */
				if (wiimote_init_ext_step == 1) {
					unsigned char buf[1];

					buf[0] = 0x00;
					if (wiimote_request_mem_data_write(hid_event.mac0, hid_event.mac1, RW_REG, 0xA400FB, 1, buf)) {
						LOG("Extension initialization error\n");
						break;
					}

					wiimote_init_ext_step = 2;
				} else if (wiimote_init_ext_step == 2) {
					/* Read extension ID */
					if (wiimote_request_mem_data_read(hid_event.mac0, hid_event.mac1, RW_REG, 0xA400FE, 2)) {
						LOG("Read error (extension error)\n");
						break;
					}

					wiimote_init_ext_step = 0;
				}
				break;

			case 0x30: /* Core Buttons */
				wiimote_buttons = (recv_buff[2] << 8) | recv_buff[1];

				enqueue_read_request(hid_event.mac0, hid_event.mac1,
					&hid_request, recv_buff, sizeof(recv_buff));

				break;

			case 0x32: /* Core Buttons with 8 Extension bytes */
				wiimote_buttons = (recv_buff[2] << 8) | recv_buff[1];

				wiimote_nunchuk_data.sx = recv_buff[3];
				wiimote_nunchuk_data.sy = recv_buff[4];
				wiimote_nunchuk_data.ax = (recv_buff[5] << 2) | (recv_buff[8] >> 2);
				wiimote_nunchuk_data.ay = (recv_buff[6] << 2) | (recv_buff[8] >> 4);
				wiimote_nunchuk_data.az = (recv_buff[7] << 2) | (recv_buff[8] >> 6);
				wiimote_nunchuk_data.bc = !(recv_buff[8] & 0x2);
				wiimote_nunchuk_data.bz = !(recv_buff[8] & 0x1);

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

		case 0x06: /* Device disconnect event*/
			wiimote_connected = 0;
			break;
		}
	}

	return 0;
}

static int viimote_bt_thread(SceSize args, void *argp)
{
	bt_cb_uid = ksceKernelCreateCallback("kbluetooth_callback", 0, bt_cb_func, NULL);
	LOG("Bluetooth callback UID: 0x%08X\n", bt_cb_uid);

	TEST_CALL(ksceBtRegisterCallback, bt_cb_uid, 0, 0xFFFFFFFF, 0xFFFFFFFF);

	ksceBtStartInquiry();
	ksceKernelDelayThreadCB(2 * 1000 * 1000);
	ksceBtStopInquiry();

	while (bt_thread_run) {
		ksceKernelDelayThreadCB(200 * 1000);
	}

	if (wiimote_mac0 || wiimote_mac1) {
		ksceBtStartDisconnect(wiimote_mac0, wiimote_mac1);
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

	/* SceCtrl hooks */
	SceCtrl_sceCtrlPeekBufferPositive_hook_uid = taiHookFunctionExportForKernel(KERNEL_PID,
		&SceCtrl_sceCtrlPeekBufferPositive_ref, "SceCtrl", TAI_ANY_LIBRARY,
		0xA9C3CED6, SceCtrl_sceCtrlPeekBufferPositive_hook_func);

	SceCtrl_sceCtrlPeekBufferPositive2_hook_uid = taiHookFunctionExportForKernel(KERNEL_PID,
		&SceCtrl_sceCtrlPeekBufferPositive2_ref, "SceCtrl", TAI_ANY_LIBRARY,
		0x15F81E8C, SceCtrl_sceCtrlPeekBufferPositive2_hook_func);

	SceKernelMemPoolCreateOpt opt;
	opt.size = 0x1C;
	opt.uselock = 0x100;
	opt.field_8 = 0x10000;
	opt.field_C = 0;
	opt.field_10 = 0;
	opt.field_14 = 0;
	opt.field_18 = 0;

	bt_mempool_uid = ksceKernelMemPoolCreate("viimote_mempool", 0x7000, &opt);
	LOG("Bluetooth mempool UID: 0x%08X\n", bt_mempool_uid);

	bt_thread_uid = ksceKernelCreateThread("viimote_vt_thread", viimote_bt_thread,
		0x3C, 0x2000, 0, 0x10000, 0);
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
		LOG("Unhooked SceBt_sub_22999C8\n");
	}

	if (SceCtrl_sceCtrlPeekBufferPositive_hook_uid > 0) {
		taiHookReleaseForKernel(SceCtrl_sceCtrlPeekBufferPositive_hook_uid,
			SceCtrl_sceCtrlPeekBufferPositive_ref);
		LOG("Unhooked SceCtrl_sceCtrlPeekBufferPositive\n");
	}

	if (SceCtrl_sceCtrlPeekBufferPositive2_hook_uid > 0) {
		taiHookReleaseForKernel(SceCtrl_sceCtrlPeekBufferPositive2_hook_uid,
			SceCtrl_sceCtrlPeekBufferPositive2_ref);
		LOG("Unhooked SceCtrl_sceCtrlPeekBufferPositive2\n");
	}

	log_flush();

	return SCE_KERNEL_STOP_SUCCESS;
}
