#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/bt.h>
#include <psp2/ctrl.h>
#include <taihen.h>
#include "log.h"

#define WIIMOTE_VID 0x057E
#define WIIMOTE_PID 0x0306

/* Grabbed from: https://github.com/abstrakraft/cwiid */
#define RPT_LED_RUMBLE		0x11
#define RPT_RPT_MODE		0x12
#define RPT_STATUS_REQ		0x15

#define RPT_MODE_BUF_LEN	2

#define RPT_BTN			0x30
#define RPT_BTN_ACC		0x31

typedef struct {
	unsigned char id;
	unsigned char unk1;
	unsigned short unk2;
	unsigned int unk3;
	unsigned int mac0;
	unsigned int mac1;
} BtHidEvent;

static SceUID bt_mempool_uid = -1;
static SceUID bt_thread_uid = -1;
static SceUID bt_cb_uid = -1;
static int bt_thread_run = 1;

static int wiimote_connected = 0;
static unsigned int wiimote_mac0 = 0;
static unsigned int wiimote_mac1 = 0;
static unsigned short wiimote_buttons = 0;

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

/*static int wiimote_request_status(unsigned int mac0, unsigned int mac1)
{
	unsigned char data;

	data = 0;
	if (wiimote_send_rpt(mac0, mac1, 0, RPT_STATUS_REQ, 1, &data)) {
		LOG("Status request error\n");
		return -1;
	}

	return 0;
}

static int update_rpt_mode(unsigned int mac0, unsigned int mac1, int8_t rpt_mode)
{
	unsigned char buf[RPT_MODE_BUF_LEN];
	uint8_t rpt_type;
	const int continuous = 1;

	rpt_type = RPT_BTN_ACC;

	// Send SET_REPORT
	buf[0] = continuous ? 0x04 : 0;
	buf[1] = rpt_type;
	if (wiimote_send_rpt(mac0, mac1, 0, RPT_RPT_MODE, RPT_MODE_BUF_LEN, buf)) {
		LOG("Send report error (report mode)");
		return -1;
	}

	return 0;
}

static int wiimote_set_rpt_mode(unsigned int mac0, unsigned int mac1)
{
	return update_rpt_mode(mac0, mac1, -1);
}*/

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

static int bt_cb_func(int notifyId, int notifyCount, int notifyArg, void *common)
{
	static SceBtHidRequest hid_request;
	static unsigned char recv_buff[8];

	while (1) {
		int ret;
		BtHidEvent hid_event;

		memset(&hid_event, 0, sizeof(hid_event));

		do {
			ret = ksceBtReadEvent((SceBtEvent *)&hid_event, 1);
		} while (ret == SCE_BT_ERROR_CB_OVERFLOW);

		if (ret <= 0) {
			break;
		}

		/*LOG("->Event:");
		for (int i = 0; i < 0x10; i++)
			LOG(" %02X", ((unsigned char *)&hid_event)[i]);
		LOG("\n");*/

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

		case 0x05: /* Device connected event */
			wiimote_set_led(hid_event.mac0, hid_event.mac1, 1);
			wiimote_connected = 1;
			break;

		case 0x0A: /* HID message received */

			switch (recv_buff[0]) {
			case 0x30: /* Core Buttons */
				wiimote_buttons = (recv_buff[2] << 8) | recv_buff[1];
				break;
			}

			memset(&hid_request, 0, sizeof(hid_request));
			memset(recv_buff, 0, sizeof(recv_buff));

			/* Enqueue a new read request */
			hid_request.type = 0;
			hid_request.buffer = recv_buff;
			hid_request.length = sizeof(recv_buff);
			hid_request.next = &hid_request;

			ksceBtHidTransfer(hid_event.mac0, hid_event.mac1, &hid_request);
			break;

		case 0x0B:
			memset(&hid_request, 0, sizeof(hid_request));
			memset(recv_buff, 0, sizeof(recv_buff));

			hid_request.type = 0;
			hid_request.buffer = recv_buff;
			hid_request.length = sizeof(recv_buff);
			hid_request.next = &hid_request;

			ksceBtHidTransfer(hid_event.mac0, hid_event.mac1, &hid_request);
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

	LOG("SceBt_sub_22999C8 hook UID: 0x%08X\n", SceBt_sub_22999C8_hook_uid);

	/* SceCtrl hooks */
	SceCtrl_sceCtrlPeekBufferPositive_hook_uid = taiHookFunctionExportForKernel(KERNEL_PID,
		&SceCtrl_sceCtrlPeekBufferPositive_ref, "SceCtrl", TAI_ANY_LIBRARY,
		0xA9C3CED6, SceCtrl_sceCtrlPeekBufferPositive_hook_func);

	SceCtrl_sceCtrlPeekBufferPositive2_hook_uid = taiHookFunctionExportForKernel(KERNEL_PID,
		&SceCtrl_sceCtrlPeekBufferPositive2_ref, "SceCtrl", TAI_ANY_LIBRARY,
		0x15F81E8C, SceCtrl_sceCtrlPeekBufferPositive2_hook_func);

	LOG("SceBt_sub_22999C8 hook UID: 0x%08X\n", SceBt_sub_22999C8_hook_uid);

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
