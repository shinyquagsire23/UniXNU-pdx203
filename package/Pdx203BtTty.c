// SPDX-License-Identifier: BSD-2-Clause-Patent
// The Bluetooth controller's serial port as iOS opens it. bluetoothd and BlueTool talk to the kernel control socket
// "com.apple.uart.bluetooth" (AppleOnboardSerial's interface: SYSPROTO_CONTROL, SOCK_STREAM, termios through getsockopt/setsockopt
// TIOCGETA/TIOCSETA, the H4 stream as the socket's data); bluetoothd also requires /dev/cu.bluetooth or /dev/uart.bluetooth to exist,
// which are one tty here (the kernel's tty layer, as a pty replica: usable by hand). The controller's bytes go to the socket's client
// when there is one, else to the tty when open. The hardware side (Pdx203Bluetooth.cpp) owns the UART, the controller's speed and its
// firmware, so speed changes asked through termios are accepted and change nothing.
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/tty.h>
#include <sys/ttycom.h>
#include <sys/fcntl.h>
#include <sys/proc.h>
#include <sys/kauth.h>
#include <sys/ioccom.h>
#include <miscfs/devfs/devfs.h>
#include <sys/kern_control.h>
#include <sys/kpi_mbuf.h>
#include <sys/sys_domain.h>
#include <kern/locks.h>
#include <string.h>
#include <stdint.h>

// the IOKit serial family's ioctls (IOKit/serial/ioss.h)
#define BT_IOSSIOSPEED _IOW('T', 2, unsigned long)
#define BT_IOSSDATALAT _IOW('T', 0, unsigned long)
#define BT_IOSSPREEMPT _IOW('T', 1, int)
// below the input queue's flow-control mark (tty.c I_HIGH_WATER, TTYHOG - 512): the tty never blocks or flushes input of its own accord
#define I_HIGH_WATER_BT (TTYHOG - 2 * 256 - 16)

// [0] major, [1] opens, [2] closes, [3] bytes in (to the app), [4] bytes out (from the app), [5] bytes held back (input queue full),
// [6] ioctls not handled, [7] the last such ioctl, [8] speed asked last, [9] modem bits set last
volatile int32_t pdx203_bt_tty_stats[10];

static struct tty *gTty;
static int gMajor = -1;
static int gOpen;
static uint32_t gModem = TIOCM_DTR | TIOCM_RTS;
static void (*gKick)(void);
static void (*gOpened)(int open);
static int ctl_attach(void);
static lck_grp_t *gGrp;
static lck_mtx_t *gLock;

static void bt_start(struct tty *tp)
{
	(void)tp;
	if (gKick) {
		gKick();
	}
}

static int bt_param(struct tty *tp, struct termios *t)
{
	(void)tp;
	pdx203_bt_tty_stats[8] = (int32_t)t->c_ospeed;
	return 0;
}

static void bt_stop(struct tty *tp, int flush)
{
	(void)tp; (void)flush;
}

static int bt_open(dev_t dev, int flag, __unused int devtype, __unused struct proc *p)
{
	struct tty *tp = gTty;
	int error;
	if (minor(dev) >= 2) {
		return 0;   // /dev/btwake: bluetoothd's H4BC holds it open while it wants the controller awake; this one never sleeps
	}
	if (!tp) {
		return ENXIO;
	}
	tty_lock(tp);
	if (!ISSET(tp->t_state, TS_ISOPEN)) {
		termioschars(&tp->t_termios);
		tp->t_iflag = 0;
		tp->t_oflag = 0;
		tp->t_lflag = 0;
		tp->t_cflag = CS8 | CREAD | CLOCAL | CRTSCTS;
		tp->t_ispeed = tp->t_ospeed = 3000000;
		ttsetwater(tp);
	} else if (ISSET(tp->t_state, TS_XCLUDE) && !kauth_cred_issuser(kauth_cred_get())) {
		tty_unlock(tp);
		return EBUSY;
	}
	tp->t_dev = dev;
	(void)(*linesw[tp->t_line].l_modem)(tp, 1);
	error = (*linesw[tp->t_line].l_open)(dev, tp);
	if (!error) {
		++gOpen;
		pdx203_bt_tty_stats[1]++;
	}
	tty_unlock(tp);
	if (!error && gOpened) {
		gOpened(1);
	}
	return error;
}

static int bt_close(dev_t dev, int flag, __unused int mode, __unused proc_t p)
{
	struct tty *tp = gTty;
	int error;
	if (minor(dev) >= 2) {
		return 0;
	}
	if (!tp) {
		return ENXIO;
	}
	tty_lock(tp);
	error = (*linesw[tp->t_line].l_close)(tp, flag);
	(void)ttyclose(tp);
	if (gOpen > 0) {
		--gOpen;
	}
	pdx203_bt_tty_stats[2]++;
	tty_unlock(tp);
	if (gOpened) {
		gOpened(0);
	}
	return error;
}

static int bt_read(dev_t dev, struct uio *uio, int flag)
{
	struct tty *tp = gTty;
	int error;
	if (minor(dev) >= 2) {
		return 0;
	}
	if (!tp) {
		return ENXIO;
	}
	tty_lock(tp);
	error = (*linesw[tp->t_line].l_read)(tp, uio, flag);
	tty_unlock(tp);
	// room in the input queue again: the hardware side may have bytes held back
	if (gKick) {
		gKick();
	}
	return error;
}

static int bt_write(dev_t dev, struct uio *uio, int flag)
{
	struct tty *tp = gTty;
	int error;
	if (minor(dev) >= 2) {
		return 0;
	}
	if (!tp) {
		return ENXIO;
	}
	tty_lock(tp);
	error = (*linesw[tp->t_line].l_write)(tp, uio, flag);
	tty_unlock(tp);
	return error;
}

static int bt_ioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	struct tty *tp = gTty;
	int error;
	if (minor(dev) >= 2) {
		return ENOTTY;
	}
	if (!tp) {
		return ENXIO;
	}
	tty_lock(tp);
	error = (*linesw[tp->t_line].l_ioctl)(tp, cmd, data, flag, p);
	if (error == ENOTTY) {
		error = ttioctl_locked(tp, cmd, data, flag, p);
	}
	if (error == ENOTTY) {
		error = 0;
		switch (cmd) {
		case TIOCMGET:
			*(int *)data = (int)(gModem | TIOCM_CTS | TIOCM_DSR | TIOCM_CD | TIOCM_LE);
			break;
		case TIOCMSET:
			gModem = (uint32_t)*(int *)data;
			break;
		case TIOCMBIS:
			gModem |= (uint32_t)*(int *)data;
			break;
		case TIOCMBIC:
			gModem &= ~(uint32_t)*(int *)data;
			break;
		case TIOCSDTR:
			gModem |= TIOCM_DTR;
			break;
		case TIOCCDTR:
			gModem &= ~(uint32_t)TIOCM_DTR;
			break;
		case TIOCSBRK:
		case TIOCCBRK:
			break;
		case BT_IOSSIOSPEED:
			pdx203_bt_tty_stats[8] = (int32_t)*(unsigned long *)data;
			tp->t_ispeed = tp->t_ospeed = (speed_t)*(unsigned long *)data;
			break;
		case BT_IOSSDATALAT:
		case BT_IOSSPREEMPT:
			break;
		default:
			pdx203_bt_tty_stats[6]++;
			pdx203_bt_tty_stats[7] = (int32_t)cmd;
			error = ENOTTY;
			break;
		}
		pdx203_bt_tty_stats[9] = (int32_t)gModem;
	}
	tty_unlock(tp);
	return error;
}

static int bt_select(dev_t dev, int rw, void *wql, proc_t p)
{
	struct tty *tp = gTty;
	int rv;
	if (minor(dev) >= 2) {
		return 1;
	}
	if (!tp) {
		return ENXIO;
	}
	tty_lock(tp);
	rv = ttyselect(tp, rw, wql, p);
	tty_unlock(tp);
	return rv;
}

static int bt_dstop(struct tty *tp, int flush)
{
	(void)tp; (void)flush;
	return 0;
}

static int bt_reset(__unused int uban)
{
	return 0;
}

static const struct cdevsw bt_cdev = {
	.d_open       = bt_open,
	.d_close      = bt_close,
	.d_read       = bt_read,
	.d_write      = bt_write,
	.d_ioctl      = bt_ioctl,
	.d_stop       = bt_dstop,
	.d_reset      = bt_reset,
	.d_ttys       = NULL,
	.d_select     = bt_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_TTY
};

// Makes the tty, its two nodes and the control socket (owned by mobile: bluetoothd and BlueTool run as mobile). kick: called (tty locked, or not) when the
// host wrote bytes or read some; opened: after a tty open (1) and close (0), a socket connect (2) and disconnect (3). Returns 0 or an errno.
int pdx203_bt_tty_attach(void (*kick)(void), void (*opened)(int))
{
	if (gTty) {
		return 0;
	}
	gKick = kick;
	gOpened = opened;
	gGrp = lck_grp_alloc_init("pdx203-bt", LCK_GRP_ATTR_NULL);
	gLock = lck_mtx_alloc_init(gGrp, LCK_ATTR_NULL);
	if (!gLock) {
		return ENOMEM;
	}
	struct tty *tp = ttymalloc();
	if (!tp) {
		return ENOMEM;
	}
	tp->t_oproc = bt_start;
	tp->t_param = bt_param;
	tp->t_stop = bt_stop;
	gMajor = cdevsw_add(-24, &bt_cdev);
	if (gMajor < 0) {
		ttyfree(tp);
		return EBUSY;
	}
	gTty = tp;
	pdx203_bt_tty_stats[0] = gMajor;
	(void)devfs_make_node(makedev(gMajor, 0), DEVFS_CHAR, 501, 501, 0660, "cu.bluetooth");
	(void)devfs_make_node(makedev(gMajor, 1), DEVFS_CHAR, 501, 501, 0660, "uart.bluetooth");
	(void)devfs_make_node(makedev(gMajor, 2), DEVFS_CHAR, 501, 501, 0660, "btwake");
	return ctl_attach();
}

// ---- the kernel control socket
#define BT_TIOCGETA 0x40487413
#define BT_TIOCSETA 0x80487414
_Static_assert(sizeof(struct termios) == 0x48, "termios as the socket options carry it");
// [0] registered (1) or the errno, [1] connects, [2] disconnects, [3] bytes in, [4] bytes out, [5] options got, [6] options set,
// [7] the last unknown option, [8] speed set last, [9] sends refused (ring full)
volatile int32_t pdx203_bt_ctl_stats[10];
static kern_ctl_ref gCtl, gCtlSk;
static uint32_t gUnit;             // the client the controller's bytes go to (the latest), 0 none
static kern_ctl_ref gUnitRef;      // and its control
static struct termios gTermios;
#define BT_TX_RING (64 * 1024)
static uint8_t gTxRing[BT_TX_RING];
static size_t gTxHead, gTxTail;

static errno_t ctl_connect_cb(kern_ctl_ref ref, struct sockaddr_ctl *sac, void **unitinfo)
{
	(void)unitinfo;
	lck_mtx_lock(gLock);
	gUnit = sac->sc_unit;
	gUnitRef = ref;
	gTxHead = gTxTail = 0;
	lck_mtx_unlock(gLock);
	pdx203_bt_ctl_stats[1]++;
	if (gOpened) {
		gOpened(2);
	}
	return 0;
}

static errno_t ctl_disconnect_cb(kern_ctl_ref ref, u_int32_t unit, void *unitinfo)
{
	(void)unitinfo;
	lck_mtx_lock(gLock);
	if (gUnit == unit && gUnitRef == ref) {
		gUnit = 0;
	}
	lck_mtx_unlock(gLock);
	pdx203_bt_ctl_stats[2]++;
	if (gOpened) {
		gOpened(3);
	}
	return 0;
}

static errno_t ctl_send_cb(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, mbuf_t m, int flags)
{
	(void)ref; (void)unit; (void)unitinfo; (void)flags;
	size_t len = mbuf_pkthdr_len(m);
	errno_t error = 0;
	lck_mtx_lock(gLock);
	if (BT_TX_RING - (gTxHead - gTxTail) < len) {
		error = ENOBUFS;
		pdx203_bt_ctl_stats[9]++;
	} else {
		size_t at = 0;
		while (at < len) {
			size_t pos = gTxHead % BT_TX_RING, n = MIN(len - at, BT_TX_RING - pos);
			if (mbuf_copydata(m, at, n, gTxRing + pos)) {
				error = EINVAL;
				break;
			}
			gTxHead += n;
			at += n;
		}
		if (!error) {
			pdx203_bt_ctl_stats[4] += (int32_t)len;
		}
	}
	lck_mtx_unlock(gLock);
	mbuf_freem(m);
	if (!error && gKick) {
		gKick();
	}
	return error;
}

static errno_t ctl_setopt_cb(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, int opt, void *data, size_t len)
{
	(void)ref; (void)unit; (void)unitinfo;
	if ((uint32_t)opt == BT_TIOCSETA && len >= sizeof gTermios) {
		memcpy(&gTermios, data, sizeof gTermios);
		pdx203_bt_ctl_stats[6]++;
		pdx203_bt_ctl_stats[8] = (int32_t)gTermios.c_ospeed;
		return 0;
	}
	pdx203_bt_ctl_stats[7] = opt;
	return ENOTSUP;
}

static errno_t ctl_getopt_cb(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, int opt, void *data, size_t *len)
{
	(void)ref; (void)unit; (void)unitinfo;
	if ((uint32_t)opt == BT_TIOCGETA) {
		if (data) {
			if (*len < sizeof gTermios) {
				return EINVAL;
			}
			memcpy(data, &gTermios, sizeof gTermios);
		}
		*len = sizeof gTermios;
		pdx203_bt_ctl_stats[5]++;
		return 0;
	}
	pdx203_bt_ctl_stats[7] = opt;
	return ENOTSUP;
}

static void ctl_rcvd_cb(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, int flags)
{
	(void)ref; (void)unit; (void)unitinfo; (void)flags;
	if (gKick) {
		gKick();
	}
}

static int ctl_attach(void)
{
	struct kern_ctl_reg reg;
	memset(&reg, 0, sizeof reg);
	strlcpy(reg.ctl_name, "com.apple.uart.bluetooth", sizeof reg.ctl_name);
	reg.ctl_flags = CTL_FLAG_REG_SOCK_STREAM | CTL_FLAG_REG_EXTENDED;
	reg.ctl_sendsize = 64 * 1024;
	reg.ctl_recvsize = 64 * 1024;
	reg.ctl_connect = ctl_connect_cb;
	reg.ctl_disconnect = ctl_disconnect_cb;
	reg.ctl_send = ctl_send_cb;
	reg.ctl_setopt = ctl_setopt_cb;
	reg.ctl_getopt = ctl_getopt_cb;
	reg.ctl_rcvd = ctl_rcvd_cb;
	gTermios.c_cflag = CS8 | CREAD | CLOCAL | CRTSCTS;
	gTermios.c_ispeed = gTermios.c_ospeed = 115200;
	errno_t error = ctl_register(&reg, &gCtl);
	// bluetoothd's default name, the Skywalk port's: its Skywalk branch is patched out (board.toml), so it is the same plain socket
	strlcpy(reg.ctl_name, "com.apple.uart.sk.bluetooth", sizeof reg.ctl_name);
	errno_t errorSk = ctl_register(&reg, &gCtlSk);
	pdx203_bt_ctl_stats[0] = error ? (int32_t)error : errorSk ? (int32_t)(1000 + errorSk) : 1;
	return error ? error : errorSk;
}

// Controller bytes for the host: the socket's client, else the tty. Returns how many were taken: none when nobody listens (the caller
// drops them), fewer than n when the reader's queue is near full (the caller keeps the rest and offers it again after the next read).
size_t pdx203_bt_tty_rx(const uint8_t *bytes, size_t n)
{
	lck_mtx_lock(gLock);
	uint32_t unit = gUnit;
	kern_ctl_ref ref = gUnitRef;
	lck_mtx_unlock(gLock);
	if (unit && ref) {
		size_t space = 0;
		if (ctl_getenqueuespace(ref, unit, &space) || !space) {
			return 0;
		}
		size_t k = MIN(n, space);
		if (ctl_enqueuedata(ref, unit, (void *)(uintptr_t)bytes, k, 0)) {
			return 0;
		}
		pdx203_bt_ctl_stats[3] += (int32_t)k;
		return k;
	}
	struct tty *tp = gTty;
	size_t taken = 0;
	if (!tp) {
		return 0;
	}
	tty_lock(tp);
	if (!ISSET(tp->t_state, TS_ISOPEN)) {
		tty_unlock(tp);
		return 0;
	}
	while (taken < n && tp->t_rawq.c_cc + tp->t_canq.c_cc < I_HIGH_WATER_BT) {
		(*linesw[tp->t_line].l_rint)(bytes[taken], tp);
		++taken;
	}
	tty_unlock(tp);
	pdx203_bt_tty_stats[3] += (int32_t)taken;
	if (taken < n) {
		pdx203_bt_tty_stats[5]++;
	}
	return taken;
}

// Somebody listens: the socket has a client or the tty is open.
int pdx203_bt_tty_is_open(void)
{
	return gUnit != 0 || gOpen > 0;
}

// The host's bytes for the controller, up to max: the socket's first, then the tty's (waking its writers).
size_t pdx203_bt_tty_tx(uint8_t *bytes, size_t max)
{
	size_t got = 0;
	lck_mtx_lock(gLock);
	while (got < max && gTxTail < gTxHead) {
		size_t pos = gTxTail % BT_TX_RING, n = MIN(MIN(max - got, gTxHead - gTxTail), BT_TX_RING - pos);
		memcpy(bytes + got, gTxRing + pos, n);
		gTxTail += n;
		got += n;
	}
	lck_mtx_unlock(gLock);
	struct tty *tp = gTty;
	if (got < max && tp) {
		tty_lock(tp);
		int k = q_to_b(&tp->t_outq, bytes + got, (int)MIN(max - got, (size_t)INT32_MAX));
		if (k > 0) {
			ttwwakeup(tp);
			pdx203_bt_tty_stats[4] += k;
			got += (size_t)k;
		}
		tty_unlock(tp);
	}
	return got;
}
