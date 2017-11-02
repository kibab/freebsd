/*-
 * Copyright (c) 2017 Ilya Bakulin <kibab@FreeBSD.org>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <machine/atomic.h>
#include <sys/kthread.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/unistd.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/mbuf.h>
#include <sys/condvar.h>
#include <sys/cons.h>
#include <sys/sysctl.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <netinet/in.h>

#define MAX_EVENT 4
#define RINGBUFSIZE 16384

const char helloworld[] = "HELLO FROM KIBAB\n";

static struct proc *kthread;
static int event;
static struct cv event_cv;
static struct mtx event_mtx;

static struct sysctl_ctx_list clist;
static struct sysctl_oid *poid;

struct socket *s;
struct sockaddr_in addr;

static char buf[RINGBUFSIZE];
static uint32_t r_pos, w_pos;

static int
init_sock() {
	int err;
	char host[256];
	err = socreate(PF_INET, &s, SOCK_DGRAM, IPPROTO_UDP,
	    curthread->td_ucred, curthread);
	if (err != 0) {
		printf("Cannot create socket, err=%d\n", err);
		return err;
	}
	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_len = sizeof(struct sockaddr_in);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(6666);
	if (!getenv_string("netconsole.host", (char *) &host, 256)) {
		printf("NetConsole host not set\n");
		return EINVAL;
	}
	err = inet_aton(host, &addr.sin_addr);
	if (err != 1) {
		printf("Cannot convert address\n");
		return EINVAL;
	}
	return 0;
}

static int
send_something(const char *msg, uint32_t len) {
	struct mbuf *m;
	char *str;
	int err;

	if (s == NULL)
		return EINVAL;
	m = m_gethdr(M_WAITOK, MT_DATA);
	if (m == NULL) {
		printf("Cannot allocate mbuf\n");
		return ENOMEM;
	}
	KASSERT(len <= MHLEN, ("len > MHLEN: %d > %d", len, MHLEN));
	str = mtod(m, char*);
	memcpy(str, msg, len);
	m->m_pkthdr.len = m->m_len = len;

	err = sosend(s, (struct sockaddr *) &addr, NULL, m, NULL, 0, curthread);

	if (err != 0)
		printf("sosend() error: %d\n", err);

	return err;
}

static void send_buf() {
	int l, err, _w_pos;
sendagain:
	mtx_lock(&event_mtx);
	_w_pos = atomic_load_acq_32(&w_pos);
	if (_w_pos < r_pos) {
		/* Ringbuf filled */
		l = min(RINGBUFSIZE - r_pos, MHLEN);
	} else
		l = min(_w_pos - r_pos, MHLEN);
	if (l < 0) {
		mtx_unlock(&event_mtx);
		return;
	}
	mtx_unlock(&event_mtx);
	err = send_something((const char *)buf + r_pos, l);
	if (err != 0) {
		if (err == ENETDOWN || err == ENETUNREACH) {
			/* Network not ready yet, retry */
			printf("Net is down(%d), retry...\n", err);
			pause("netcons", hz / 10);
			goto sendagain;
		}
		return;
	}
	r_pos += l;
	if (r_pos == RINGBUFSIZE)
		r_pos = 0;
	if (r_pos != _w_pos)
		goto sendagain;
}

static void
sleep_thread(void *arg)
{
	int ev;

	for (;;) {
		mtx_lock(&event_mtx);
		while ((ev = event) == 0)
			cv_wait(&event_cv, &event_mtx);
		event = 0;
		mtx_unlock(&event_mtx);

		switch (ev) {
		case -1:
			kproc_exit(0);
			break;
		case 0:
			break;
		case 1:
			printf("sleep... is alive and well.\n");
			printf("r_pos=%d w_pos=%d\n", r_pos, w_pos);
			break;
		case 2:
			printf("Initializing UDP socket\n");
			init_sock();
			break;
		case 3:
			send_something(helloworld, strlen(helloworld) - 1);
			break;
		case 4:
			send_buf();
			break;
		default:
			panic("event %d is bogus\n", event);
		}
	}
}

static int
sysctl_debug_sleep_test(SYSCTL_HANDLER_ARGS)
{
	int error, i = 0;

	error = sysctl_handle_int(oidp, &i, 0, req);
	if (error == 0 && req->newptr != NULL) {
		if (i >= 1 && i <= MAX_EVENT) {
			mtx_lock(&event_mtx);
			KASSERT(event == 0, ("event %d was unhandled",
			    event));
			event = i;
			cv_signal(&event_cv);
			mtx_unlock(&event_mtx);
		} else
			error = EINVAL;
	}

	return (error);
}

static cn_probe_t	netcons_cnprobe;
static cn_init_t	netcons_cninit;
static cn_term_t	netcons_cnterm;
static cn_getc_t	netcons_cngetc;
static cn_putc_t	netcons_cnputc;
static cn_grab_t	netcons_cngrab;
static cn_ungrab_t	netcons_cnungrab;

static void
netcons_cngrab(struct consdev *cp)
{
	printf("%s\n", __func__);
}

static void
netcons_cnungrab(struct consdev *cp)
{
	printf("%s\n", __func__);
}


static void
netcons_cnprobe(struct consdev *cp)
{
	sprintf(cp->cn_name, "netconsole");
	cp->cn_pri = CN_REMOTE;
	cp->cn_flags = CN_FLAG_NODEBUG;
}

static void
netcons_cninit(struct consdev *cp)
{
	printf("%s\n", __func__);
}

/* This function cannot sleep or lock mutexes! */
static void
netcons_cnputc(struct consdev *cp, int c)
{
	buf[w_pos++] = (char) c;
	event = 4;
	if (w_pos == RINGBUFSIZE)
		w_pos = 0;
	cv_signal(&event_cv);
}

static int
netcons_cngetc(struct consdev * cp)
{
	printf("%s\n", __func__);
	return 0;
}

static void
netcons_cnterm(struct consdev * cp)
{
	printf("%s\n", __func__);
}

CONSOLE_DRIVER(netcons);

static int
load(void *arg)
{
	int error;
	struct proc *p;
	struct thread *td;

	error = kproc_create(sleep_thread, NULL, &p, RFSTOPPED, 0, "sleep");
	if (error)
		return (error);

	s = NULL;
	event = 0;
	mtx_init(&event_mtx, "sleep event", NULL, MTX_DEF);
	cv_init(&event_cv, "sleep");

	td = FIRST_THREAD_IN_PROC(p);
	thread_lock(td);
	TD_SET_CAN_RUN(td);
	sched_add(td, SRQ_BORING);
	thread_unlock(td);
	kthread = p;

	sysctl_ctx_init(&clist);
	poid = SYSCTL_ADD_NODE(&clist, SYSCTL_STATIC_CHILDREN(_debug),
	    OID_AUTO, "sleep", CTLFLAG_RD, 0, "sleep tree");
	SYSCTL_ADD_PROC(&clist, SYSCTL_CHILDREN(poid), OID_AUTO, "test",
	    CTLTYPE_INT | CTLFLAG_RW, 0, 0, sysctl_debug_sleep_test, "I",
	    "");

	init_sock();
	bzero(buf, sizeof(buf));
	r_pos = w_pos = 0;
	netcons_cnprobe(&netcons_consdev);
	error = cnadd(&netcons_consdev);
	if (error)
		printf("Failed to add console: %d\n", error);
	return (0);
}

static int
unload(void *arg)
{
	sysctl_ctx_free(&clist);
	mtx_lock(&event_mtx);
	event = -1;
	cv_signal(&event_cv);
	mtx_sleep(kthread, &event_mtx, PWAIT, "sleep", 0);
	mtx_unlock(&event_mtx);
	mtx_destroy(&event_mtx);
	cv_destroy(&event_cv);

	if (s) {
		soshutdown(s, SHUT_RDWR);
		soclose(s);
		s = NULL;
	}
	cnremove(&netcons_consdev);
	return (0);
}


static int
netcons_modevent(module_t mod __unused, int event, void *arg)
{
	int error = 0;

	switch (event) {
	case MOD_LOAD:
		error = load(arg);
		break;
	case MOD_UNLOAD:
		error = unload(arg);
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static moduledata_t netcons_mod = {
	"netcons",
	netcons_modevent,
	NULL
};

DECLARE_MODULE(netcons, netcons_mod, SI_SUB_PROTO_FIREWALL, SI_ORDER_ANY);
