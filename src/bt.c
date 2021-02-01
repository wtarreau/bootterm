/*
 * Copyright (C) 2020 Willy Tarreau <w@1wt.eu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(__linux__) && !defined(NO_TCGETS2)
#include <asm/termbits.h>
#else
#include <termios.h>
#endif
#include <time.h>
#include <unistd.h>
#if defined(__FreeBSD__)
#include <sys/sysctl.h>
#endif

#define MAXPORTS 100
#define BUFSIZE 2048
#define MARGIN  1024

#ifndef VERSION
#define VERSION "0.4.0"
#endif

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif

#ifndef CRTSCTS
#define CRTSCTS 0
#endif

#ifndef O_NOFOLLOW
#define O_NOFOLLOW 0
#endif

const char version_str[] =
	"BootTerm " VERSION " -- the terminal written for users by its users\n"
#if defined(TCGETS2) && !defined(NO_TCGETS2)
	"Built with support for custom baud rates (TCGETS2).\n"
#endif
	"Copyright (C) 2020-2021 Willy Tarreau <w@1wt.eu>\n"
	"This is free software: you are encouraged to improve and redistribute it.\n"
	"There is no warranty of any kind, see the source for license details.\n";

const char usage_msg[] =
	"Usage: %s [options] [/dev/ttyXXX | port#]\n"
	"Options:\n"
	"  -h           display this help\n"
	"  -q           quiet mode: do not report events\n"
	"  -p           only print selected port name and quit\n"
	"  -l           list detected serial ports and quit\n"
	"  -a           wait for a port to be available (BT_SCAN_WAIT_ANY)\n"
	"  -n           wait for a new port to be registered (BT_SCAN_WAIT_NEW)\n"
	"  -m <min>     specify lowest printable character  (default: 0)\n"
	"  -M <max>     specify highest printable character (default: 255)\n"
	"  -b <baud>    specify baud rate (BT_PORT_BAUD_RATE; default:115200; 0=current)\n"
	"  -c {none|fixed|timed} enable capture to file (BT_CAPTURE_MODE)\n"
	"  -t {none|abs|init|line} enable timestamps in captures (BT_TIMESTAMP_MODE)\n"
	"  -T           enable timestamps in terminal (see -t for formats)\n"
	"  -e <escape>  escape character or ASCII code  (default: 29 = Ctrl-])\n"
	"  -f <fmt>     capture file name (default: 'bootterm-%%Y%%m%%d-%%H%%M%%S.log')\n"
	"  -V           show version and license\n"
	"  -B           send a break sequence before starting the terminal\n"
	"  -N           do not launch the terminal, just quit\n"
	"  -L <mode>    new line mode: 0=CRLF, 1=LF only, 2=CR only (BT_PORT_CRLF)\n"
	"\n"
	"Ports are sorted in reverse registration order so that port 0 is the most\n"
	"recently added one. A number may be set instead of the port. With no name nor\n"
	"number, last port is used. Use '?' or 'help' in baud rate to list them all.\n"
	"Comma-delimited lists of ports to exclude/include/restrict may be passed in\n"
	"BT_SCAN_EXCLUDE_PORTS, BT_SCAN_INCLUDE_PORTS, and BT_SCAN_RESTRICT_PORTS.\n"
	"BT_SCAN_EXCLUDE_DRIVERS ignores ports matching these drivers.\n"
	"";

/* Note that we need CRLF in raw mode */
const char menu_str[] =
	"\r\n"
	"BootTerm supports several single-character commands after the escape character:\r\n"
	"  H h ?      display this help\r\n"
	"  Q q .      quit\r\n"
	"  P p        show port status\r\n"
	"  D d        flip DTR pin\r\n"
	"  R r        flip RTS pin\r\n"
	"  F f        flip both DTR and RTS pins\r\n"
	"  B b        send break\r\n"
	"  C c        enable / disable capture\r\n"
	"  T t        enable / disable timestamps on terminal\r\n"
	"Enter the escape character again after this menu to use these commands.\r\n"
	"";

enum crlf_mode {
	CRLF_NORMAL = 0,
	CRLF_LFONLY = 1,
	CRLF_CRONLY = 2,
};

enum cap_mode {
	CAP_NONE = 0,
	CAP_FIXED,
	CAP_TIMED,
};

enum ts_mode {
	TS_NONE = 0,
	TS_ABS,  // absolute time
	TS_INIT, // time relative to init time
	TS_LINE, // time relative to previous line
};

/* In order to better deal with isolated LF, isolated CR, CRLF and LFCR when it
 * comes to inserting timestamps, we'll see a line in 4 different states:
 * Beginning-Of-New-Line (BONL), Beginning-Of-Same-Line (BOSL), Same-Line (SL),
 * New-Line (NL). The transitions will occur like this:
 *
 *         |        CR                 The timestamp is emitted after entering
 *    /\   v   ---------->             the BONL state on LF, and before leaving
 * LF \_> BONL <---------- BOSL        BOSL for SL. This deals with line
 *         ^        LF     *| ^        overwriting using CR and LFCR cleanly.
 *      CR |                V | CR     BONL ensures a TS was already printed,
 *         NL <------------ SL         so none is needed on CR there. In BOSL,
 *                                     a timestamp is needed only before data.
 *                                     SL is assumed first. Time is retrieved
 *                                     when entering BONL.
 *
 * We stay in NL until a CR is there so that even with LFxxxCR we emit the TS.
 */
enum line_state {
	LS_BONL,               // beginning of new line
	LS_BOSL,               // beginning of same line
	LS_SL,                 // same line (at least one char)
	LS_NL,                 // new line (changed but not homed)
};

struct serial {
	char *name;            // device name without /dev
	char *driver;          // driver name (usually a short word)
	char *desc;            // descrption when reported
	time_t ctime;          // device attachment date
};

struct buffer {
	int data;              // where data area starts, <0 if output closed.
	int room;              // where room area starts, <0 if input closed.
	int len;               // number of bytes in.
	unsigned char b[BUFSIZE];
};

const struct {
	int b;      // baud rate
	tcflag_t f; // corresponding flag
} baud_rates[] = {
#ifdef B50
	{ .b = 50, .f = B50 },
#endif
#ifdef B75
	{ .b = 75, .f = B75 },
#endif
#ifdef B110
	{ .b = 110, .f = B110 },
#endif
#ifdef B134
	{ .b = 134, .f = B134 },
#endif
#ifdef B150
	{ .b = 150, .f = B150 },
#endif
#ifdef B200
	{ .b = 200, .f = B200 },
#endif
#ifdef B300
	{ .b = 300, .f = B300 },
#endif
#ifdef B600
	{ .b = 600, .f = B600 },
#endif
#ifdef B1200
	{ .b = 1200, .f = B1200 },
#endif
#ifdef B1800
	{ .b = 1800, .f = B1800 },
#endif
#ifdef B2400
	{ .b = 2400, .f = B2400 },
#endif
#ifdef B4800
	{ .b = 4800, .f = B4800 },
#endif
#ifdef B9600
	{ .b = 9600, .f = B9600 },
#endif
#ifdef B19200
	{ .b = 19200, .f = B19200 },
#endif
#ifdef B38400
	{ .b = 38400, .f = B38400 },
#endif
#ifdef B57600
	{ .b = 57600, .f = B57600 },
#endif
#ifdef B76800
	{ .b = 76800, .f = B76800 },
#endif
#ifdef B115200
	{ .b = 115200, .f = B115200 },
#endif
#ifdef B153600
	{ .b = 153600, .f = B153600 },
#endif
#ifdef B230400
	{ .b = 230400, .f = B230400 },
#endif
#ifdef B307200
	{ .b = 307200, .f = B307200 },
#endif
#ifdef B460800
	{ .b = 460800, .f = B460800 },
#endif
#ifdef B500000
	{ .b = 500000, .f = B500000 },
#endif
#ifdef B576000
	{ .b = 576000, .f = B576000 },
#endif
#ifdef B614400
	{ .b = 614400, .f = B614400 },
#endif
#ifdef B921600
	{ .b = 921600, .f = B921600 },
#endif
#ifdef B1000000
	{ .b = 1000000, .f = B1000000 },
#endif
#ifdef B1152000
	{ .b = 1152000, .f = B1152000 },
#endif
#ifdef B1500000
	{ .b = 1500000, .f = B1500000 },
#endif
#ifdef B2000000
	{ .b = 2000000, .f = B2000000 },
#endif
#ifdef B2500000
	{ .b = 2500000, .f = B2500000 },
#endif
#ifdef B3000000
	{ .b = 3000000, .f = B3000000 },
#endif
#ifdef B3500000
	{ .b = 3500000, .f = B3500000 },
#endif
#ifdef B4000000
	{ .b = 4000000, .f = B4000000 },
#endif
	{ .b = 0, .f = ~0 } // end
};

/* operating modes */
struct serial serial_ports[MAXPORTS];
const char *baud_rate_str = NULL;
const char *cap_mode_str = NULL;
enum cap_mode cap_mode = CAP_NONE;
enum crlf_mode crlf_mode = CRLF_NORMAL;
const char *capture_fmt = "bootterm-%Y%m%d-%H%M%S.log";
unsigned char escape  = 0x1d; // Ctrl-]
unsigned char chr_min = 0;
unsigned char chr_max = 255;
const char *port = NULL;
int nbports = 0;
int currport = -1; // last one
int quiet = 0;
int baud = 115200;

char cap_name[PATH_MAX];
int cap_fd = -1; // -1 = no capture in progress
time_t last_cap_sec = 0;
struct timeval start_ts = { }; // timestamp of current line
enum ts_mode ts_mode = TS_NONE;
enum line_state line_state = LS_SL; // anywhere on the line
const char *ts_mode_str = NULL;
int ts_term = 0;

/* list made of port names (without slashes) delimited by commas */
char *exclude_drivers = NULL;
char *exclude_list = NULL;
char *include_list = NULL;
char *restrict_list = NULL;
const char *always_ignore = "pty*,null,zero,rtc*,nvram*,mem,kmem,kbd*,mouse*,psaux*,audio*,dsp*,fb*,random,urandom,mt*,rmt*,console,ptmx,ptc,tty,stderr,stdin,stdout";

/* display the message and exit with the code */
void die(int code, const char *format, ...)
{
	va_list args;

	va_start(args, format);
	vfprintf(stderr, format, args);
	va_end(args);
	exit(code);
}

/* displays the usage message and exits with code <code> */
void usage(int code, const char *progname)
{
	die(code, usage_msg, progname);
}

/* parse a long in <in> (which may be NULL) and copy the result in <out>.
 * Returns 0 on failure to parse an valid number.
 */
int parse_long(const char *in, long *out)
{
	char *endptr;

	if (!in || !*in)
		return 0;

	*out = strtol(in, &endptr, 0);
	return *endptr == 0;
}

/* parse an int in <in> (which may be NULL) and copy the result in <out>.
 * Returns 0 on failure to parse an valid number.
 */
int parse_int(const char *in, int *out)
{
	char *endptr;
	long ret;

	if (!in || !*in)
		return 0;

	ret = strtol(in, &endptr, 0);
	*out = ret;
	return *endptr == 0 && ret >= INT_MIN && ret <= INT_MAX;
}

/* parse a byte in <in> (which may be NULL) and copy the result in <out>.
 * Returns 0 on failure to parse an valid number.
 */
int parse_byte(const char *in, unsigned char *out)
{
	char *endptr;
	long ret;

	if (!in || !*in)
		return 0;

	ret = strtol(in, &endptr, 0);
	*out = ret;
	return *endptr == 0 && ret >= 0 && ret <= 255;
}

/* Look for item <item> in comma-delimited list <list>. A null list is empty.
 * <item> may not contain commas. Returns true if found, false otherwise.
 */
int in_list(const char *list, const char *item)
{
	size_t ilen = strlen(item);
	const char *word;

	while (list && *list) {
		word = list;
		while (*list && *list != ',' && *list != '*')
			list++;

		if (*list == '*' && ilen >= list - word) {
			if (strncmp(word, item, list - word) == 0)
				return 1;
		}

		if (strncmp(word, item, ilen) == 0 &&
		    (word[ilen] == 0 || word[ilen] == ','))
			return 1;

		while (*list && *(list++) != ',')
			;
	}
	return 0;
}

#if defined(TCGETS2) && !defined(NO_TCGETS2) // since linux 2.6.20 (commit edc6afc54)
/* try to set the baud rate and mode on the port corresponding to the fd. The
 * baud rate remains unchanged if <baud> is zero. We have two APIs, one using
 * TCSETS and one using TCSETS2, which allows non-standard baud rates. On
 * failure -1 is returned with errno set, otherwise zero is returned. If a
 * baud rate couldn't be found, -1 is returned and errno set.
 */
int set_port(int fd, int baud)
{
	struct termios2 tio;

	ioctl(fd, TCGETS2, &tio);
	/* c_cflag serves to set both directions at once this way (CBAUD is a
	 * mask for all the Bxxxx speed bits) (see termbits.h):
	 *    (Bxxxx & CBAUD) << IBSHIFT for the input speed
	 *    (Bxxxx & CBAUD) for the output speed.
	 * A special baud rate value, BOTHER, indicates that the speed is to be
	 * taken from the c_ispeed and c_ospeed fields instead.
	 */
	tio.c_cflag  = (tio.c_cflag & ~CSIZE) | CS8; // 8-bit
	tio.c_cflag  = (tio.c_cflag & ~CSTOPB);      // 1-stop
	tio.c_cflag &= ~(PARENB | PARODD);           // no parity
	tio.c_cflag &= ~HUPCL;                       // no DTR/RTS down
	tio.c_cflag &= ~CRTSCTS;                     // no flow control
	tio.c_cflag |= CREAD;                        // enable rx
	tio.c_cflag |= CLOCAL;                       // local echo

	if (baud) {
		tio.c_cflag &= ~(CBAUD | (CBAUD << IBSHIFT));
		tio.c_cflag |= BOTHER | (BOTHER << IBSHIFT);
		tio.c_ospeed = tio.c_ispeed = baud;
	}

	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_lflag = 0;

	return ioctl(fd, TCSETS2, &tio);
}

/* returns the currently configured baud rate for the port, or 0 if unknown */
int get_baud_rate(int fd)
{
	struct termios2 tio;
	int idx;

	ioctl(fd, TCGETS2, &tio);
	if ((tio.c_cflag & (CBAUD|CBAUDEX)) == BOTHER)
		return tio.c_ospeed;

	for (idx = 0; baud_rates[idx].b; idx++)
		if ((tio.c_cflag & (CBAUD|CBAUDEX)) == baud_rates[idx].f)
			return baud_rates[idx].b;
	return 0;
}

void list_baud_rates()
{
	printf("termios2 is supported, so any baud rate is supported\n");
}

/* we have to redefined these ones because GNU libc tries very hard to
 * prevent us from using those above, and notably purposely creates
 * redefinition errors to prevent us from loading termios.h
 */
static int tcgetattr(int fd, struct termios *tio)
{
	return ioctl(fd, TCGETS, tio);
}

static int tcsetattr(int fd, int ignored, const struct termios *tio)
{
	return ioctl(fd, TCSETS, tio);
}

static int tcsendbreak(int fd, int duration)
{
	return ioctl(fd, TCSBRK, duration);
}

#else /* TCSETS2 not defined */

/* returns a combination of Bxxxx flags to set on termios c_cflag depending on
 * the baud rate values supported on the platform. Only exact values match. If
 * no value is found, (tcflag_t)~0 is returned, assuming it will never match an
 * existing value on any platform.
 */
tcflag_t baud_encode(int baud)
{
	int idx;

	for (idx = 0; baud_rates[idx].b; idx++) {
		if (baud_rates[idx].b == baud)
			return baud_rates[idx].f;
	}
	return baud_rates[idx].f;
}

/* try to set the baud rate and mode on the port corresponding to the fd. The
 * baud rate remains unchanged if <baud> is zero. We have two APIs, one using
 * TCSETS and one using TCSETS2, which allows non-standard baud rates. On
 * failure -1 is returned with errno set, otherwise zero is returned. If a
 * baud rate couldn't be found, -1 is returned and errno set.
 */
int set_port(int fd, int baud)
{
	struct termios tio = { };
	tcflag_t baud_flag;

	// see man tty_ioctl
	tcgetattr(fd, &tio);

	/* c_cflag serves to set both directions at once this way (CBAUD is a
	 * mask for all the Bxxxx speed bits) (see termbits.h):
	 *    (Bxxxx & CBAUD) << IBSHIFT for the input speed
	 *    (Bxxxx & CBAUD) for the output speed.
	 * A special baud rate value, BOTHER, indicates that the speed is to be
	 * taken from the c_ispeed and c_ospeed fields instead.
	 */
	tio.c_cflag  = (tio.c_cflag & ~CSIZE) | CS8; // 8-bit
	tio.c_cflag  = (tio.c_cflag & ~CSTOPB);      // 1-stop
	tio.c_cflag &= ~(PARENB | PARODD);           // no parity
	tio.c_cflag &= ~HUPCL;                       // no DTR/RTS down
	tio.c_cflag &= ~CRTSCTS;                     // no flow control
	tio.c_cflag |= CREAD;                        // enable rx
	tio.c_cflag |= CLOCAL;                       // local echo

	if (baud) {
		//tio.c_cflag &= ~(CBAUD | (CBAUD << IBSHIFT));
		//tio.c_cflag |= BOTHER | (BOTHER << IBSHIFT);
		baud_flag = baud_encode(baud);
		if (baud_flag == ~0) { /* baud rate not found */
			errno = EINVAL;
			return -1;
		}
		if (cfsetospeed(&tio, baud_flag) == -1)
			return -1;
		if (cfsetispeed(&tio, baud_flag) == -1)
			return -1;
	}

	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	return tcsetattr(fd, TCSANOW, &tio);
}

/* returns the currently configured baud rate for the port, or 0 if unknown */
int get_baud_rate(int fd)
{
	struct termios tio;
	speed_t spd;
	int idx;

	tcgetattr(fd, &tio);
	spd = cfgetospeed(&tio);

	for (idx = 0; baud_rates[idx].b; idx++)
		if (spd == baud_rates[idx].f)
			return baud_rates[idx].b;
	return 0;
}

void list_baud_rates()
{
	int idx;

	printf("The following baud rates are known (not necessarily supported though):");
	for (idx = 0; baud_rates[idx].b; idx++)
		printf("%s %8d", (idx % 5 == 0) ? "\n    " : "", baud_rates[idx].b);
	putchar('\n');
}

#endif // TCSETS2 not defined


/* returns the value of configuration variable <var_name> or NULL if not found.
 * The variable is first looked up in the environment by prepending "BT_", by
 * turning all letters to upper case, and changing "." and "-" to "_". Example:
 * "scan.exclude-ports" becomes "BT_SCAN_EXCLUDE_PORTS".
 */
const char *get_conf(const char *var_name)
{
	const char *value = NULL;

	if (strlen(var_name) + 3 < 256) {
		char envname[256];
		char *d, c;
		const char *s;

		strcpy(envname, "BT_");
		d = envname + 3;
		s = var_name;
		do {
			c = *(s++);
			if (c == '.' || c == '-')
				c = '_';
			else if (islower(c))
				c = toupper(c);
			*(d++) = c;
		} while (c);
		value = getenv(envname);
		if (!value || !*value)
			value = NULL;
	}

	/* TODO: implement an option to perform a look-up in a config file if
	 * value is still NULL.
	 */

	return value;
}

/* removes all blanks and all occurrences of "/dev/" from var and make it a
 * port list used to replace <list>. NULL is just an empty list and is valid.
 */
void set_port_list(char **list, const char *var)
{
	char *value, *s, *d;

	free(*list);
	*list = NULL;

	value = var ? strdup(var) : NULL;
	if (!value)
		return;

	/* strip off "/dev/" in front of each name, and trim all blanks */
	d = s = value;
	while (*s) {
	        while (isblank((int)(unsigned char)*s))
			s++;
		if (strncmp(s, "/dev/", 5) == 0) {
			s += 5;
			continue;
		}
		*d++ = *s++;
	}
	*d++ = 0;

	if (!*value) {
		free(value);
		value = NULL;
	}
	*list = value;
}

/* read one line from file name assembled from <format>, return the contents
 * copied into allocated memory or NULL in case of error.
 */
char *read_line_from(const char *format, ...)
{
	char ftmp[PATH_MAX];
	char ltmp[4096];
	int needed = 0;
	va_list args;
	char *line = NULL;
	FILE *f;

	va_start(args, format);
	needed = vsnprintf(ftmp, sizeof(ftmp), format, args);
	va_end(args);

	if (needed < sizeof(ftmp)) {
		f = fopen(ftmp, "r");
		if (f) {
			line = fgets(ltmp, sizeof(ltmp), f);
			fclose(f);
			if (line) {
				/* delete the trailing LF */
				if (*line && line[strlen(line) - 1] == '\n')
					line[strlen(line) - 1] = 0;
				line = strdup(line);
			}
		}
	}
	return line;
}

/* read the symlink from the name assembled from <format>, return the contents
 * copied into allocated memory or NULL in case of error.
 */
char *read_link_from(const char *format, ...)
{
	char ftmp[PATH_MAX];
	char ltmp[4096];
	int needed = 0;
	va_list args;
	char *line = NULL;
	ssize_t ret;

	va_start(args, format);
	needed = vsnprintf(ftmp, sizeof(ftmp), format, args);
	va_end(args);

	if (needed < sizeof(ftmp)) {
		ret = readlink(ftmp, ltmp, sizeof(ltmp) - 1);
		if (ret > 0) {
			ltmp[ret] = 0;
			line = strdup(ltmp);
		}
	}
	return line;
}

/* checks if a file exists. returns non-zero if OK, otherwise zero. */
int file_exists(const char *format, ...)
{
	char ftmp[PATH_MAX];
	int needed = 0;
	va_list args;
	struct stat st;

	va_start(args, format);
	needed = vsnprintf(ftmp, sizeof(ftmp), format, args);
	va_end(args);

	if (needed < sizeof(ftmp))
		return stat(ftmp, &st) == 0;

	return 0;
}

/* updates the capture file name if needed, and if it changed, closes the
 * current capture and opens a new one.
 */
void set_capture_name()
{
	char new_name[PATH_MAX];
	time_t now;

	if (cap_mode == CAP_NONE && cap_fd == -1)
		return;

	if (cap_mode == CAP_FIXED && cap_fd >= 0)
		return;

	now = time(NULL);
	if (now == last_cap_sec)
		return;

	if (cap_mode == CAP_NONE ||
	    !strftime(new_name, sizeof(new_name), capture_fmt, localtime(&now))) {
		if (cap_fd >= 0)
			close(cap_fd);
		cap_fd = -1;
		return;
	}

	if (cap_fd == -1 || strcmp(new_name, cap_name) != 0) {
		if (cap_fd >= 0)
			close(cap_fd);
		memcpy(cap_name, new_name, sizeof(cap_name));
		cap_fd = open(cap_name, O_APPEND | O_CREAT | O_WRONLY | O_NOFOLLOW, 0666);
		/* note that cap_fd==-1 is handled as a temporarily disabled capture */
	}
	last_cap_sec = now;
}

/* sorting function for serial ports: ensures the most recently registered port
 * appears first. If some are equal, the device name is used instead for normal
 * ordering with an attempt at respecting numeric ordering.
 */
static int serial_port_cmp(const void *a, const void *b)
{
	const struct serial *pa = (const struct serial *)a;
	const struct serial *pb = (const struct serial *)b;
	long long la, lb;
	int pos;

	if (pa->ctime != pb->ctime)
		return pa->ctime - pb->ctime;

	pos = 0;
	while (pa->name[pos] == pb->name[pos]) {
		if (!pa->name[pos])
			return 0; // identical strings
		pos++;
	}

	/* if one of the string stops here, it comes first. Eg "tty" comes
	 * before "tty1".
	 */
	if (!pa->name[pos])
		return -1;

	if (!pb->name[pos])
		return 1;

	/* if the first different char is not a digit in any of them, that's
	 * OK. Eg "ttyS1" comes before "ttyUSB0".
	 */
	if (!isdigit((int)(unsigned char)pa->name[pos]) &&
	    !isdigit((int)(unsigned char)pb->name[pos]))
		return (int)pa->name[pos] - (int)pb->name[pos];

	/* if only one is a digit and previous char is not a digit, the digit
	 * goes first. Eg. "tty63" comes before "ttyp0".
	 */
	if (isdigit((int)(unsigned char)pa->name[pos]) && (!pos || !isdigit((int)(unsigned char)pa->name[pos-1])) &&
	    !isdigit((int)(unsigned char)pb->name[pos]))
		return -1;

	if (isdigit((int)(unsigned char)pb->name[pos]) && (!pos || !isdigit((int)(unsigned char)pb->name[pos-1])) &&
	    !isdigit((int)(unsigned char)pa->name[pos]))
		return 1;

	/* so both are digits preceeded by digits. We need to roll back to the
	 * full number because we could be on a series of zeroes in a larger
	 * number, e.g. "port100011" comes after "port10012".
	 */

	/* look back for the beginning of a possible number */
	while (pos > 0 && isdigit((int)(unsigned char)pa->name[pos-1]))
		pos--;

	la = strtoll(pa->name + pos, 0, 0);
	lb = strtoll(pb->name + pos, 0, 0);

	if (la < lb)
		return -1;
	else
		return 1;
}

/* Tries to open device node <devname> after verifying that it looks like a
 * char device, and checks if it supports termios. Returns non-zero on success,
 * zero on failure.
 *
 * WT: maybe we should only do that on devices shared with groups or users so
 *     as to limit ourselves to non-dangerous candidates only ?
 */
int file_isatty(const char *devname)
{
	struct termios tio;
	struct winsize ws;
	struct stat st;
	int ret = 0;
	int fd = -1;

	if (stat(devname, &st) != 0)
		goto fail;

	if ((st.st_mode & S_IFMT) != S_IFCHR)
		goto fail;

	fd = open(devname, O_RDONLY | O_NONBLOCK | O_NOCTTY);
	if (fd == -1)
		goto fail;

	ret = isatty(fd);

	/* terminals with columns and rows set are usually local consoles */
	if (ret && (ioctl(fd, TIOCGWINSZ, &ws) != 0 || (ws.ws_row && ws.ws_col)))
		ret = 0;

#ifdef __linux__
	/* On Linux, only keep terminals having CLOCAL set, those without are local consoles */
	if (ret && (ioctl(fd, TCGETS, &tio) != 0 || !(tio.c_cflag & CLOCAL)))
		ret = 0;
#endif

	close(fd);
fail:
	return ret;
}

/* scans available ports, fills serial_ports[] and returns the number of
 * entries filled in (also stored in nbports).
 */
int scan_ports_generic()
{
	/* all entries must start with "/dev" */
	const char *dirs[] = { "/dev", "/dev/usb/tts", "/dev/usb/acm", NULL };
	struct dirent *ent;
	char ftmp[PATH_MAX];
	const char *devname;
	struct stat st;
	int diridx = 0;
	DIR *dir = NULL;

	nbports = 0;

	for (diridx = 0; dirs[diridx]; diridx++) {
		dir = opendir(dirs[diridx]);
		if (!dir)
			continue;

		while (nbports < MAXPORTS) {
			ent = readdir(dir);
			if (!ent)
				break;
			snprintf(ftmp, sizeof(ftmp), "%s/%s", dirs[diridx], ent->d_name);
			devname = ftmp + 5; // skip "/dev/"

			if (in_list(always_ignore, devname))
				continue;

			if (in_list(exclude_list, devname))
				continue;

			if (restrict_list && !in_list(restrict_list, devname))
				continue;

#ifdef __APPLE__
			/* On macOS all serial ports appear as /dev/cu.<name> */
			if (strncmp(devname, "cu.", 3) != 0)
				continue;
#elif __FreeBSD__
			/* On FreeBSD, USB serial ports appear as /dev/cua* and we need
			 * to drop *.lock and *.init.
			 */
			if (strncmp(devname, "cua", 3) == 0) {
				size_t len = strlen(devname);
				const char *end = devname + len;

				if (len > 5 &&
				    (strcmp(end - 5, ".init") == 0 ||
				     strcmp(end - 5, ".lock") == 0))
					continue;
			}
			else {
				/* nothing other than cua* for FreeBSD */
				continue;
			}
#elif __linux__
			/* On Linux, tty[0-9]* are virtual consoles, tty[a-z]* are
			 * pseudo-ttys, and vcs* are virtual consoles.
			 */
			if (strncmp(devname, "tty", 3) == 0 &&
			    (islower(devname[3]) || isdigit(devname[3])))
				continue;

			if (strncmp(devname, "vcs", 3) == 0)
				continue;
#endif

			if (!file_isatty(ftmp))
				continue;

			if (stat(ftmp, &st) == 0) {
				serial_ports[nbports].name = strdup(devname);
				serial_ports[nbports].driver = NULL;
				serial_ports[nbports].desc = NULL;
				serial_ports[nbports].ctime = st.st_ctime;
				nbports++;
				continue;
			}
		}
		closedir(dir);
	}

	if (nbports)
		qsort(serial_ports, nbports, sizeof(serial_ports[0]), serial_port_cmp);

	return nbports;
}

#ifdef __linux__

/* This version relies on /sys/class/tty. If not found, it falls back to the
 * generic version which uses /dev.
 */
int scan_ports()
{
	struct dirent *ent;
	char ftmp[PATH_MAX];
	struct stat st;
	DIR *dir = NULL;
	char *link, *driver, *desc, *name;
	int candidates = 0;

	nbports = 0;

	dir = opendir("/sys/class/tty");
	if (!dir)
		return scan_ports_generic();

	while (nbports < MAXPORTS) {
		ent = readdir(dir);
		if (!ent)
			break;

		if (in_list(always_ignore, ent->d_name))
			continue;

		if (in_list(exclude_list, ent->d_name))
			continue;

		if (restrict_list && !in_list(restrict_list, ent->d_name))
			continue;

		link = driver = desc = name = NULL;
		snprintf(ftmp, sizeof(ftmp), "/sys/class/tty/%s/device/.", ent->d_name);
		if (stat(ftmp, &st) == 0) {
			/* really populated ports have either a "resources"
			 * entry (8250/16550), a "resource" entry (platform),
			 * an "of_node" entry (anything from a DT), an
			 * "interface" entry (e.g. for cdc_acm) or a "port_number"
			 * entry (e.g. for USB), or "bInterfaceClass" for any USB
			 * device with limited implementation. This seems to cover
			 * most cases where auto-detection matters. Some on-board
			 * devices on some non-DT platforms will have nothing but
			 * a "type" entry showing a non-zero value for present
			 * devices. Since we test for the device's presence anyway
			 * we don't even need to check that entry's contents.
			 */
			if (!file_exists("/sys/class/tty/%s/device/resources", ent->d_name) &&
			    !file_exists("/sys/class/tty/%s/device/resource", ent->d_name) &&
			    !file_exists("/sys/class/tty/%s/device/bus", ent->d_name) &&
			    !file_exists("/sys/class/tty/%s/device/of_node", ent->d_name) &&
			    !file_exists("/sys/class/tty/%s/device/interface", ent->d_name) &&
			    !file_exists("/sys/class/tty/%s/device/bInterfaceClass", ent->d_name) &&
			    !file_exists("/sys/class/tty/%s/device/port_number", ent->d_name) &&
			    !file_exists("/sys/class/tty/%s/type", ent->d_name) &&
			    !in_list(include_list, ent->d_name))
				goto fail;

			candidates++;

			link = read_link_from("/sys/class/tty/%s/device/driver", ent->d_name);
			if (link) {
				driver = strrchr(link, '/');
				if (driver)
					driver++;
				else
					driver = link;

				driver = strdup(driver);
				free(link); link = NULL;
			} else if (!in_list(include_list, ent->d_name)) {
				driver = strdup("");
			}
			else
				goto fail;

			if (*driver && in_list(exclude_drivers, driver))
				goto fail;

			snprintf(ftmp, sizeof(ftmp), "/dev/%s", ent->d_name);
			if (!file_isatty(ftmp))
				goto fail;

			name = strdup(ent->d_name);
			if (!name)
				goto fail;

			/* the descrption usually appears in ../interface for ttyUSB or
			 * ./interface for ttyACM. Some devices like CH341 do not provide
			 * anything but they come with a "product" node which identifies
			 * the USB device. Finally on embedded devices using a device tree,
			 * the platform devices sometimes come with a self-explanatory
			 * "compatible" string that is worth showing as a last resort.
			 */
			desc = read_line_from("/sys/class/tty/%s/device/../interface", ent->d_name);
			if (!desc)
				desc = read_line_from("/sys/class/tty/%s/device/interface", ent->d_name);
			if (!desc)
				desc = read_line_from("/sys/class/tty/%s/device/../../product", ent->d_name);
			if (!desc)
				desc = read_line_from("/sys/class/tty/%s/device/of_node/compatible", ent->d_name);
			/* note: the model is not always set, so we accept NULL */

			serial_ports[nbports].name = name;
			serial_ports[nbports].driver = driver;
			serial_ports[nbports].desc = desc;
			serial_ports[nbports].ctime = st.st_ctime;
			nbports++;
			continue;
		}
	fail:
		free(driver);
		free(link);
		free(desc);
		free(name);

	}
	closedir(dir);

	if (nbports)
		qsort(serial_ports, nbports, sizeof(serial_ports[0]), serial_port_cmp);
	else if (!candidates)
		return scan_ports_generic();

	return nbports;
}

#elif __FreeBSD__

/* Use sysctls to enumerate all entries starting with "dev.<driver>.<instance>".
 * Those that have a "ttyname" entry are real ttys. "uart" is one as well and
 * doesn't have a ttyname entry but is mapped as cuau<instance>. The scanning
 * method was figured using "truss sysctl -a" but appears to work fine.
 */
int scan_ports()
{
	/* For lookups, the first two elements contain the request type and the
	 * the rest contains the oids making the key name. When retrieving
	 * values however this starts directly with the oid.
	 */
	int oid[CTL_MAXNAME + 2];
	size_t oidlen, nextlen;

	char ftmp[PATH_MAX];
	struct stat st;

	char name[1024]; // the sysctl entry name
	size_t namelen;  // #bytes
	char *lastword;  // last word of the sysctl name

	char value[1024];
	size_t valuelen;

	/* buffers to keep values that possibly arrive in any order */
	char currtty[1024];
	char currdrv[1024];
	char currdsc[1024];
	char currname[1024];
	size_t currnamelen;

	int done;

	/* This loop is quite odd because we need to collect data and commit
	 * them when detecting a name change, or when failing a next operation.
	 * The first call retrieves an OID. This OID must be turned into a name
	 * which we use to detect that we changed to a new node, and to
	 * retrieve variables. But the variables are then for the new node so
	 * they must not yet be checked, which is why we commit them before
	 * retrieving new values.
	 */
	currnamelen = done = 0;
	*currtty = *currdrv = *currdsc = 0;

	while (nbports < MAXPORTS) {
		if (!currnamelen) {
			/* first call, doesn't use sizeof but #entries! */
			oidlen = CTL_MAXNAME;
			if (sysctlnametomib("dev", &oid[2], &oidlen) != 0)
				return scan_ports_generic();
		} else {
			/* Warning, contrary to the first call, this one takes
			 * and returns a sizeof!
			 */
			oid[0] = CTL_SYSCTL;
			oid[1] = CTL_SYSCTL_NEXT;
			nextlen = CTL_MAXNAME * sizeof(*oid);
			if (sysctl(oid, oidlen + 2, &oid[2], &nextlen, 0, 0) != 0) {
				/* WT: we should only stop on ENOENT, but don't we
				 * face a risk of endless loop if a next fails and
				 * we don't break ?
				 */
				done = 1;
				goto commit;
			}
			oidlen = nextlen / sizeof(*oid);
		}

		/* Now our OID is always set */

		/* this will return the current sysctl name */
		oid[0] = CTL_SYSCTL;
		oid[1] = CTL_SYSCTL_NAME;
		namelen = sizeof(name);
		if (sysctl(oid, oidlen + 2, name, &namelen, 0, 0) != 0) {
			if (errno != ENOENT)
				done = 1;
			goto commit;
		}

                lastword = strrchr(name, '.');
		if (!lastword)
			lastword = name + strlen(name);

		/* "dev" is an entry on its own and appears first */
		if (strcmp(name, "dev") == 0)
			goto commit;

		/* nothing but dev. should appear otherwise it's the end */
		if (strncmp(name, "dev.", 4) != 0) {
			done = 1;
			goto commit;
		}

		/* name is now for example "dev.uart.0.%driver" */

	commit:
		/* If we've reached the end or changed to a new device, we must
		 * first commit what we have before retrieving new values.
		 */
		if (done || !currnamelen || lastword - name != currnamelen ||
		    memcmp(name, currname, currnamelen) != 0) {
			if (*currtty &&
			    snprintf(ftmp, sizeof(ftmp), "/dev/%s", currtty) < sizeof(ftmp) &&
			    !in_list(always_ignore, currtty) &&
			    !in_list(exclude_list, currtty) &&
			    (!restrict_list || in_list(restrict_list, currtty)) &&
			    stat(ftmp, &st) == 0) {
				serial_ports[nbports].name   = strdup(currtty);
				serial_ports[nbports].driver = *currdrv ? strdup(currdrv) : NULL;
				serial_ports[nbports].desc   = *currdsc ? strdup(currdsc) : NULL;
				serial_ports[nbports].ctime  = st.st_ctime;
				nbports++;
			}

			/* prepare new name */
			currnamelen = lastword - name;
			memcpy(currname, name, currnamelen);
			currname[currnamelen] = 0;
			*currtty = *currdrv = *currdsc = 0;
			if (done)
				break;
		}

		/* Let's take only "driver=uart" (then dev.uart.X => /dev/cuauX)
		 * and those having ttyname=X (then dev.foo.X.ttyname => /dev/cuaX)
		 * store last driver, ttyname, desc for each node and compare
		 * when switching to another node.
		 */
		valuelen = sizeof(value);
		if (sysctl(oid + 2, oidlen, value, &valuelen, 0, 0) == 0) {
			if (strcmp(lastword, ".ttyname") == 0)
				snprintf(currtty, sizeof(currtty), "cua%s", value);
			else if (strcmp(lastword, ".%driver") == 0)
				strlcpy(currdrv, value, sizeof(currdrv));
			else if (strcmp(lastword, ".%desc") == 0)
				strlcpy(currdsc, value, sizeof(currdsc));
		}

		*lastword = 0;
		if (!*currtty && strncmp(name, "dev.uart.", 9) == 0) {
			/* preset ttyname from dev.uart.NN to cuauNN */
			snprintf(currtty, sizeof(currtty), "cuau%s", name + 9);
		}
	}

	if (nbports)
		qsort(serial_ports, nbports, sizeof(serial_ports[0]), serial_port_cmp);

	return nbports;
}

#else /* OS-agnostic version, scans /dev */

int scan_ports()
{
	return scan_ports_generic();
}

#endif

/* list all ports if portspec < 0 or just this port. The current port is never
 * reported if a single one is requested.
 */
void list_ports(int portspec)
{
	time_t now;
	int p;

	printf(" port |  age (sec) | device     | driver           | description          \n"
	       "------+------------+------------+------------------+----------------------\n"
		);

	now = time(NULL);
	for (p = (portspec < 0 ? 0 : portspec); p < (portspec < 0 ? nbports : portspec + 1); p++) {
		printf(" %c%3d | %10u | %-10s | %-16s | %-16s \n",
		       portspec < 0 && p == currport ? '*' : ' ',
		       p,
		       (unsigned int)(now - serial_ports[p].ctime),
		       serial_ports[p].name,
		       serial_ports[p].driver ? serial_ports[p].driver : "",
		       serial_ports[p].desc ? serial_ports[p].desc : "");
	}
	putchar('\n');
}

/* Try to open this port (prepending /dev if the path doesn't start with '/').
 * The FD is returned on success, otherwise -1 with errno set appropriately.
 * It also deals with the occasional case where the port just appears but the
 * permissions are not yet there.
 */
int open_port(const char *port)
{
	char ftmp[PATH_MAX];
	int retry;
	int fd;

	if (snprintf(ftmp, sizeof(ftmp),
		     (*port == '/') ? "%s" : "/dev/%s",
		     port) > sizeof(ftmp)) {
		errno = ENAMETOOLONG;
		return -1;
	}

	for (retry = 3; retry > 0; retry--) {
		fd = open(ftmp, O_RDWR | O_NONBLOCK | O_NOCTTY);
		if (fd == -1 && errno == EPERM) {
			/* maybe the port was just connected and permissions
			 * not yet adjusted.
			 */
			usleep(100000);
			continue;
		}
		break;
	}
	return fd;
}

/* returns the first pending char in the buffer, or <0 if none */
int b_peek(const struct buffer *buf)
{
	if (!buf->len)
		return -1;
	return buf->b[buf->data];
}

/* inserts one character into the buffer if possible, always leaving at least
 * <margin> chars unused. Returns the number of chars added (0 or 1).
 */
int b_putchar(struct buffer *buf, int c, int margin)
{
	if (buf->len + margin >= BUFSIZE)
		return 0;
	buf->b[buf->room] = c;
	buf->len++;
	buf->room++;
	if (buf->room >= BUFSIZE)
		buf->room = 0;
	return 1;
}

/* atomically inserts one string into the buffer if possible, always leaving at
 * least <margin> chars unused. Returns the number of chars added (0 or len).
 */
int b_puts(struct buffer *buf, const char *str, int len, int margin)
{
	int i;

	if (buf->len + margin + len >= BUFSIZE)
		return 0;

	for (i = 0; i < len; i++) {
		buf->b[buf->room] = str[i];
		buf->len++;
		buf->room++;
		if (buf->room >= BUFSIZE)
			buf->room = 0;
	}
	return len;
}

/* skips <num> characters from the buffer. <num> must not be larger than len */
void b_skip(struct buffer *buf, int num)
{
	buf->len  -= num;
	buf->data += num;
	if (buf->data >= BUFSIZE)
		buf->data = 0;
	if (!buf->len)
		buf->data = buf->room = 0;
}

/* make a timeval from <sec>, <usec> */
static inline struct timeval tv_set(time_t sec, suseconds_t usec)
{
	struct timeval ret = { .tv_sec = sec, .tv_usec = usec };
	return ret;
}

/* used to unset a timeout */
static inline struct timeval tv_unset()
{
	return tv_set(0, ~0);
}

/* used to zero a timeval */
static inline struct timeval tv_zero()
{
	return tv_set(0, 0);
}

/* returns true if the timeval is set */
static inline int tv_isset(struct timeval tv)
{
	return tv.tv_usec != ~0;
}

/* returns true if <a> is before <b>, taking account unsets */
static inline int tv_isbefore(const struct timeval a, const struct timeval b)
{
	return !tv_isset(b) ? 1 :
	       !tv_isset(a) ? 0 :
	       ( a.tv_sec < b.tv_sec || (a.tv_sec == b.tv_sec && a.tv_usec < b.tv_usec));
}

/* returns the lowest of the two timers, for use in delay computation */
static inline struct timeval tv_min(const struct timeval a, const struct timeval b)
{
	if (tv_isbefore(a, b))
		return a;
	else
		return b;
}

/* returns the normalized sum of the <from> plus <off> */
static inline struct timeval tv_add(const struct timeval from, const struct timeval off)
{
	struct timeval ret;

	ret.tv_sec  = from.tv_sec  + off.tv_sec;
	ret.tv_usec = from.tv_usec + off.tv_usec;

	if (ret.tv_usec >= 1000000) {
		ret.tv_usec -= 1000000;
		ret.tv_sec  += 1;
	}
	return ret;
}

/* returns the delay between <past> and <now> or zero if <past> is after <now> */
static inline struct timeval tv_diff(const struct timeval past, const struct timeval now)
{
	struct timeval ret = { .tv_sec = 0, .tv_usec = 0 };

	if (tv_isbefore(past, now)) {
		ret.tv_sec  = now.tv_sec  - past.tv_sec;
		ret.tv_usec = now.tv_usec - past.tv_usec;

		if ((signed)ret.tv_usec < 0) { // overflow
			ret.tv_usec += 1000000;
			ret.tv_sec  -= 1;
		}
	}
	return ret;
}

/* attempts to write a timestamp into output buffer <buf> which is <size> bytes
 * long, based on <mode>, start_ts, <prev> and <line>. The return value is the
 * snprintf() return value, that is, larger than <size> if it failed. A 30-char
 * long buffer will never fail.
 */
int write_ts(char *out, int size, enum ts_mode mode, const struct timeval *prev, const struct timeval *line)
{
	struct timeval  t;
	int ret;

	if (mode == TS_INIT) {
		t = tv_diff(start_ts, *line);
		ret = snprintf(out, size, "[%6u.%06u] ", (unsigned)t.tv_sec, (unsigned)t.tv_usec);
	} else if (mode == TS_LINE) {
		if (!tv_isset(*prev)) { // first line
			t = tv_zero();
		} else
			t = tv_diff(*prev, *line);
		ret = snprintf(out, size, "[%6u.%06u] ", (unsigned)t.tv_sec, (unsigned)t.tv_usec);
	} else { // TS_ABS
		ret = strftime(out, size, "[%Y%m%d-%H%M%S.", localtime(&line->tv_sec));
		ret += snprintf(out+ret, size-ret, "%06u] ", (unsigned int)line->tv_usec);
	}
	return ret;
}

void write_to_capture(int fd, const struct buffer *buf, int len)
{
	static struct timeval prev_ts; // timestamp of previous line
	static struct timeval line_ts; // timestamp of current line
	static int cap_bol = 1;        // beginning of line
	const unsigned char *lf;
	char hdr[30];
	int pos = 0;
	int max;
	int ret;

	while (len > 0) {
		if (ts_mode > TS_NONE && cap_bol) {
			/* at beginning of line, must write a timestamp */
			gettimeofday(&line_ts, NULL);
			ret = write_ts(hdr, sizeof(hdr), ts_mode, &prev_ts, &line_ts);
			ret = write(fd, hdr, ret);
			cap_bol = 0;
		}

		lf = memchr(buf->b + buf->room + pos, '\n', len);
		if (lf) {
			cap_bol = 1;
			max = lf - (buf->b + buf->room + pos) + 1;
		}
		else
			max = len;

		ret = write(fd, buf->b + buf->room + pos, max);
		if (ret < 0)
			break;

		pos += ret;
		len -= ret;
		prev_ts = line_ts;
	}
}

/* send as much as possible of a buffer to the fd */
void buf_to_fd(int fd, struct buffer *buf)
{
	ssize_t ret;

	if (buf->data < 0)
		return;

	ret = write(fd, buf->b + buf->data, MIN(buf->len, BUFSIZE - buf->data));
	if (ret < 0) {
		/* error */
		if (errno != EAGAIN)
			buf->data = -1;
		return;
	}

	b_skip(buf, ret);
}

/* receive as much as possible from the fd to the buffer */
void fd_to_buf(int fd, struct buffer *buf, int capfd)
{
	ssize_t ret;

	if (buf->room < 0)
		return;

	ret = read(fd, buf->b + buf->room, MIN(BUFSIZE - buf->len, BUFSIZE - buf->room));
	if (ret <= 0) {
		/* error or close */
		if (ret == 0 || errno != EAGAIN)
			buf->room = -1;
		return;
	}

	if (capfd >= 0)
		write_to_capture(capfd, buf, ret);

	buf->len  += ret;
	buf->room += ret;
	if (buf->room >= BUFSIZE)
		buf->room = 0;
}

/* receive as much as possible from the fd to the buffer, and applies CRLF
 * transformations. In order to make sure we'll always have enough room to
 * insert the missing CRs or LFs, we read no more than 1/2 of the buffer
 * room at a time.
 */
void fd_to_buf_slow(int fd, struct buffer *buf, int capfd)
{
	unsigned char tmpbuf[(BUFSIZE+1)/2];
	int room;
	ssize_t ret;
	int i;
	int wpos;
	int wtot;

	if (buf->room < 0)
		return;

	room = MIN(BUFSIZE - buf->len, BUFSIZE - buf->room);
	if (room < 2)
		return;
	room /= 2;

	ret = read(fd, tmpbuf, room);
	if (ret <= 0) {
		/* error or close */
		if (ret == 0 || errno != EAGAIN)
			buf->room = -1;
		return;
	}

	wtot = 0; wpos = buf->room;
	for (i = 0; i < ret; i++) {
		if ((tmpbuf[i] == '\n' && crlf_mode == CRLF_LFONLY) ||
		    (tmpbuf[i] == '\r' && crlf_mode == CRLF_CRONLY)) {
			buf->b[wpos++] = '\r';
			if (wpos == BUFSIZE)
				wpos = 0;
			buf->b[wpos++] = '\n';
			if (wpos == BUFSIZE)
				wpos = 0;
			wtot += 2;
		} else {
			buf->b[wpos++] = tmpbuf[i];
			if (wpos == BUFSIZE)
				wpos = 0;
			wtot++;
		}
	}

	if (capfd >= 0)
		write_to_capture(capfd, buf, wtot);

	buf->len  += wtot;
	buf->room  = wpos;
}

/* transfer as many bytes as possible from port buffer <in> to user buffer
 * <out>, encoding bytes that are not between <chr_min> and <chr_max>. Input
 * buffer is always realigned once empty. The forwarding stops after each new
 * line. If in_utf8 is not null, it will be set to either 0 or 1 depending on
 * the sequence. The function always returns zero for regular characters, >0
 * if it returns after copying a line feed.
 */
int xfer_port_to_user(struct buffer *in, struct buffer *out, int chr_min, int chr_max, int *in_utf8)
{
	enum line_state new_state;
	static struct timeval prev_ts; // timestamp of previous line
	static struct timeval line_ts; // timestamp of current line
	char hdr[30];
	int len;
	int c;

	while (in->len && out->len < BUFSIZE) {
		new_state = line_state;
		c = b_peek(in);

		/* retrieve current time when entering BONL */
		if ((c == '\n' && (line_state == LS_BONL || line_state == LS_BOSL)) ||
		    (c == '\r' && line_state == LS_NL)) {
			new_state = LS_BONL;
		}
		else if (line_state == LS_BONL && c == '\r') {
			new_state = LS_BOSL; /* TS going to be overwritten */
		}
		else if (line_state != LS_NL && c != '\n' && c != '\r') {
			new_state = LS_SL;
		}
		else if (line_state == LS_SL) {
			if (c == '\n')
				new_state = LS_NL;
			else if (c == '\r')
				new_state = LS_BOSL;
		}

		/* If we're leaving BOSL for SL, we're going to overwrite the
		 * TS header so we must print it again before attempting to
		 * emit the character.
		 */
		if (line_state == LS_BOSL && new_state == LS_SL) {
			/* rewrite current line's TS */
			if (ts_term) {
				len = write_ts(hdr, sizeof(hdr),
					       (ts_mode > TS_NONE) ? ts_mode : TS_ABS, &prev_ts, &line_ts);
				b_puts(out, hdr, len, 0);
				/* commit the change since printed already */
				line_state = new_state;
			}
		}

		/* only transfer the unprotected chars, unless they belong to
		 * the unescaped C1 set of control characters which tends to
		 * make some terminals choke and which easily happen when using
		 * incorrect baud rates.
		 */
		if (c >= chr_min && c <= chr_max &&
		    (!in_utf8 || *in_utf8 || c <= 0x80 || c >= 0x9f)) {
			/* transfer as-is */
			if (!b_putchar(out, c, MARGIN))
				break;
		} else {
			/* transcode to "<0xHH>" (6 chars) */
			char tmp[7];

			snprintf(tmp, sizeof(tmp), "<0x%02X>", c);
			if (!b_puts(out, tmp, 6, MARGIN))
				break;
		}

		/* need to print the TS at the beginning of this new line */
		if (new_state == LS_BONL) {
			prev_ts = line_ts;
			gettimeofday(&line_ts, NULL);
			/* print */
			if (ts_term) {
				len = write_ts(hdr, sizeof(hdr),
					       (ts_mode > TS_NONE) ? ts_mode : TS_ABS, &prev_ts, &line_ts);
				b_puts(out, hdr, len, 0);
			}
		}

		/* commit the new state */
		line_state = new_state;

		if (in_utf8) {
			if (*in_utf8 && (c < 0x80 || c >= 0xc0))
				*in_utf8 = 0;
			if (!*in_utf8 && c >= 0xc0 && c <= 0xf7)
				*in_utf8 = 1;
		}

		b_skip(in, 1);
		if (c == '\n')
			return 1;
	}
	return 0;
}

/* transfer as many bytes as possible from user buffer <in> to port buffer
 * <out>. Input buffer is always realigned once empty. The forwarding stops
 * before character <esc>. Set esc to -1 to ignore it. The function always
 * returns zero for regular characters, <0 if it stops before an escape
 * character.
 */
int xfer_user_to_port(struct buffer *in, struct buffer *out, int esc)
{
	int c;

	while (in->len && out->len < BUFSIZE) {
		c = b_peek(in);
		if (c == esc)
			return -1;

		if (!b_putchar(out, c, MARGIN))
			break;

		b_skip(in, 1);
	}
	return 0;
}

/* show the port status into <resp> for size <size> in a way suitable for
 * sending to a raw terminal.
 */
void show_port(int fd, char *resp, size_t size)
{
	int pins;

	ioctl(fd, TIOCMGET, &pins);
	snprintf(resp, size,
	         "\r\nPort name: %s    Speed: %d bps    Pins: %s %s %s %s %s %s\r\n",
	         port, get_baud_rate(fd),
	         (pins & TIOCM_DTR) ? "DTR" : "dtr",
	         (pins & TIOCM_RTS) ? "RTS" : "rts",
	         (pins & TIOCM_CTS) ? "CTS" : "cts",
	         (pins & TIOCM_DSR) ? "DSR" : "dsr",
	         (pins & TIOCM_CD)  ? "CD"  : "cd",
	         (pins & TIOCM_RI)  ? "RI"  : "ri");
}

/* Bidirectional forwarding between stdin/stdout and fd. We take a great care
 * of making sure stdin and stdout are both a terminal before switching to raw
 * mode, in which case they are assumed to be the same. Otherwise they are used
 * as independent, regular files, allowing to inject/receive from/to stdio.
 */
void forward(int fd)
{
	int stdio_is_term = 0;
	fd_set rfds, wfds;
	struct termios tio_bck;
	struct buffer port_ibuf = { };
	struct buffer port_obuf = { };
	struct buffer user_ibuf = { };
	struct buffer user_obuf = { };
	struct timeval escape_timeout = tv_unset();
	struct timeval flash_timeout = tv_unset();
	struct timeval now;
	int term_flashing = 0;
	int in_utf8 = 0;
	int in_esc = 0;

	if (isatty(0) && isatty(1)) {
		struct termios tio;

		if (tcgetattr(0, &tio) != 0)
			die(4, "Failed to retrieve stdio settings.\n");

		memcpy(&tio_bck, &tio, sizeof(tio));

		/* turn the terminal to raw (character mode, no interrupt etc) */
		tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		tio.c_oflag &= ~OPOST;
		tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		tio.c_cflag &= ~(CSIZE | PARENB);
		tio.c_cflag |= CS8;

		if (tcsetattr(0, TCSANOW, &tio) != 0)
			die(4, "Failed to set stdio to raw mode.\n");
		stdio_is_term = 1;
	}

	/* Note: we're going to use select(). We should theoretically switch
	 * both FDs to non-blocking mode. The problem is that it usually is a
	 * bad idea (files are not pollable, and terminals may misbehave once
	 * once this is done, and will be hard to fix). Since select() is level
	 * triggered, it will still report events on blocking devices and on
	 * files. We must just take care not to try to read more than once on
	 * each wakeup.
	 */

	FD_ZERO(&rfds);
	FD_ZERO(&wfds);

	/* forward until both sides are closed and buffers empty. The terminal
	 * is detected as closed on either a write error or a read end.
	 */
	while (((user_obuf.len + port_ibuf.len || port_ibuf.room >= 0) && (user_obuf.data >= 0)) ||
	       ((port_obuf.len + user_ibuf.len || user_ibuf.room >= 0) && (port_obuf.data >= 0 && port_ibuf.room >= 0))) {
		struct timeval interval;
		int cnt;

		if (user_ibuf.room >= 0 && user_ibuf.len < BUFSIZE)
			FD_SET(0, &rfds);
		else
			FD_CLR(0, &rfds);

		if (user_obuf.data >= 0 && user_obuf.len > 0)
			FD_SET(1, &wfds);
		else
			FD_CLR(1, &wfds);

		if (port_ibuf.room >= 0 && port_ibuf.len < BUFSIZE)
			FD_SET(fd, &rfds);
		else
			FD_CLR(fd, &rfds);

		if (port_obuf.data >= 0 && port_obuf.len > 0)
			FD_SET(fd, &wfds);
		else
			FD_CLR(fd, &wfds);

		gettimeofday(&now, NULL);
		interval = tv_unset();

		if (in_esc)
			interval = tv_min(interval, tv_diff(now, escape_timeout));

		if (term_flashing)
			interval = tv_min(interval, tv_diff(now, flash_timeout));

		cnt = select(FD_SETSIZE, &rfds, &wfds, NULL, tv_isset(interval) ? &interval : NULL);

		gettimeofday(&now, NULL);

		if (in_esc && !tv_isbefore(now, escape_timeout)) {
			/* abort escape mode */
			in_esc = 0;
			/* flash the screen by reversing it */
			if (stdio_is_term) {
				b_puts(&user_obuf, "\e[?5h", 5, 0);
				term_flashing = 1;
				flash_timeout = tv_add(now, tv_set(0, 100000)); // 100ms flash
			}
		}

		if (term_flashing && !tv_isbefore(now, flash_timeout)) {
			/* restore the terminal */
			b_puts(&user_obuf, "\e[?5l", 5, 0);
			term_flashing = 0;
		}

		if (cnt <= 0) // signal or timeout
			continue;

		set_capture_name();

		/* flush pending data */
		if (FD_ISSET(1, &wfds))
			buf_to_fd(1, &user_obuf);

		if (FD_ISSET(fd, &wfds))
			buf_to_fd(fd, &port_obuf);

		/* receive new data */
		if (FD_ISSET(0, &rfds))
			fd_to_buf(0, &user_ibuf, -1);

		if (FD_ISSET(fd, &rfds))
			fd_to_buf_slow(fd, &port_ibuf, cap_fd);

		if (in_esc) {
			char resp[BUFSIZE];
			int c = b_peek(&user_ibuf);

			if (c == 'q' || c == 'Q' || c == '.') {
				/* quit */
				break;
			}
			else if (c == 'h' || c == 'H' || c == '?') {
				if (b_puts(&user_obuf, menu_str, strlen(menu_str), 0)) {
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
			}
			else if (c == 'd' || c == 'D' ||
				 c == 'f' || c == 'F' ||
				 c == 'r' || c == 'R') {  /* flip DTR/RTS/both pin */
				int pins;

				/* note that technically if we can't write a
				 * response we could flip multiple times but
				 * practically speaking this will not happen
				 * and almost never be an issue.
				 */
				ioctl(fd, TIOCMGET, &pins);
				if (c == 'd' || c == 'D' || c == 'f' || c == 'F')
					pins ^= TIOCM_DTR;
				if (c == 'r' || c == 'R' || c == 'f' || c == 'F')
					pins ^= TIOCM_RTS;
				ioctl(fd, TIOCMSET, &pins);

				show_port(fd, resp, sizeof(resp));
				if (b_puts(&user_obuf, resp, strlen(resp), 0)) {
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
			}
			else if (c == 'b' || c == 'B') {
				tcsendbreak(fd, 0);
				if (b_puts(&user_obuf, "Sent break\r\n", 12, 0)) {
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
			}
			else if (c == 'c' || c == 'C') {
				if (cap_mode == CAP_NONE &&
				    b_puts(&user_obuf, "Enabling capture\r\n", 18, 0)) {
					cap_mode = CAP_FIXED;
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
				else if (cap_mode > CAP_NONE &&
				         b_puts(&user_obuf, "Disabling capture\r\n", 19, 0)) {
					cap_mode = CAP_NONE;
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
			}
			else if (c == 't' || c == 'T') {
				if (!ts_term &&
				    b_puts(&user_obuf, "Enabling terminal timestamps\r\n", 30, 0)) {
					ts_term = 1;
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
				else if (ts_term &&
				         b_puts(&user_obuf, "Disabling terminal timestamps\r\n", 31, 0)) {
					ts_term = 0;
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
			}
			else if (c == 'p' || c == 'P') { /* show port status */
				show_port(fd, resp, sizeof(resp));
				if (b_puts(&user_obuf, resp, strlen(resp), 0)) {
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
			}
			else if (c == escape) {
				/* two escape chars cause one to be sent */
				if (b_putchar(&port_obuf, c, 0)) {
					in_esc = 0;
					b_skip(&user_ibuf, 1);
				}
			}
			else if (c >= 0 && b_putchar(&port_obuf, escape, 0)) {
				/* other chars will be sent as-is, preceeded by escape */
				in_esc = 0;
			}
		}

		/* transfer between IN and opposite OUT buffers */
		if (!in_esc && xfer_user_to_port(&user_ibuf, &port_obuf, escape) < 0) {
			/* we stopped in front of the escape character, let's
			 * wait up to two seconds for the sequence to be entered.
			 */
			b_skip(&user_ibuf, 1);
			escape_timeout = tv_add(now, tv_set(2, 0));
			in_esc = 1;
		}

		while (xfer_port_to_user(&port_ibuf, &user_obuf, chr_min, chr_max, &in_utf8) > 0) {
			/* xfer everything */
		}
	}

	if (stdio_is_term && term_flashing) {
		/* restore the terminal now */
		write(1, "\e[?5l", 5);
	}

	if (stdio_is_term) {
		/* restore settings and skip current line */
		if (tcsetattr(0, TCSANOW, &tio_bck) != 0)
			die(4, "Failed to restore stdio settings. Try 'stty sane'\n");
		putchar('\n');
	}
}

int main(int argc, char **argv)
{
	const char *progname;
	const char *curr = NULL;
	const char *arg = NULL;
	int idx, opt;
	int do_list = 0;
	int do_wait_new = 0;
	int do_wait_any = 0;
	int do_print = 0;
	int do_send_break = 0;
	int no_term = 0;
	int forced = 0;
	int usepath = 0;
	int retries = 0;
	int fd, ret;

	progname = strrchr(argv[0], '/');
	if (!progname)
		progname = argv[0];
	else
		progname++;

	/* read environment variables */
	set_port_list(&exclude_drivers,  get_conf("scan.exclude-drivers"));
	set_port_list(&exclude_list,  get_conf("scan.exclude-ports"));
	set_port_list(&include_list,  get_conf("scan.include-ports"));
	set_port_list(&restrict_list, get_conf("scan.restrict-ports"));
	do_wait_any = !!get_conf("scan.wait-any");
	do_wait_new = !do_wait_any && !!get_conf("scan.wait-new");

	baud_rate_str = get_conf("port.baud-rate");
	arg = get_conf("port.crlf");
	if (arg)
		crlf_mode = atoi(arg);
	cap_mode_str = get_conf("capture.mode");
	ts_mode_str = get_conf("timestamp.mode");

	/* simple parsing loop, stops before isolated '-', before words
	 * starting with other chars, but after '--', in which case <curr>
	 * points to whatever follows in the same argument. <arg> is reset to
	 * NULL when consumed. <curr> points to first unparsed word at end of
	 * loop.
	 */
	for (idx = 0;;) {
		if (!idx || !*curr || !arg) {
			if (idx && !arg && !*curr)
				idx++; // skip previously consumed separate arg
			idx++;
			curr = (idx < argc) ? argv[idx] : NULL;
			if (!curr)
				break;

			if (curr[0] != '-' || !curr[1])
				break;
			curr++;
		}

		opt = *(curr++);
		arg = *curr ? curr : (idx+1 < argc) ? argv[idx+1] : NULL;

		/* now we have the current option in <opt> and its optional
		 * argument in <arg> which may be NULL if the end of string
		 * was reached.
		 */
		if (opt == '-') {
			if (*curr)
				usage(1, progname);
			idx++;
			curr = arg;
			break;
		}

		switch (opt) {
		case 'h':
			usage(0, progname);
			break;

		case 'V':
			die(0, version_str);
			break;

		case 'B':
			do_send_break = 1;
			break;

		case 'N':
			no_term = 1;
			break;

		case 'l':
			do_list = 1;
			break;

		case 'p':
			do_print = 1;
			break;

		case 'q':
			quiet = 1;
			break;

		case 'T':
			ts_term = 1;
			break;

		case 'a':
			do_wait_any = 1; do_wait_new = 0;
			break;

		case 'n':
			do_wait_new = 1; do_wait_any = 0;
			break;

		case 'b':
			if (!arg)
				die(1, "missing argument for -%c\n", opt);
			baud_rate_str = arg;
			arg = NULL;
			break;

		case 'c':
			if (!arg)
				die(1, "missing argument for -%c\n", opt);
			cap_mode_str = arg;
			arg = NULL;
			break;

		case 't':
			if (!arg)
				die(1, "missing argument for -%c\n", opt);
			ts_mode_str = arg;
			arg = NULL;
			break;

		case 'e':
			if (!arg)
				die(1, "missing argument for -%c\n", opt);

			if (!parse_byte(arg, &escape)) {
				if (*arg && arg[1] == 0)
					escape = *arg;
				else
					die(1, "failed to parse escape code for -%c\n", opt);
			}
			arg = NULL;
			break;

		case 'f':
			if (!arg)
				die(1, "missing argument for -%c\n", opt);

			capture_fmt = arg;
			arg = NULL;
			break;

		case 'm':
			if (!parse_byte(arg, &chr_min))
				die(1, "failed to parse argument for -%c\n", opt);
			arg = NULL;
			break;

		case 'M':
			if (!parse_byte(arg, &chr_max))
				die(1, "failed to parse argument for -%c\n", opt);
			arg = NULL;
			break;

		case 'L':
			if (!arg)
				die(1, "missing argument for -%c\n", opt);
			crlf_mode = atoi(arg);
			arg = NULL;
			break;

		default:
			die(1, "Unknown option '%c'. Use -h to get help\n", opt);
			break;
		}
	}

	if (argc > idx + 1)
		die(1, "Too many arguments: '%s'. Use -h to get help\n", argv[idx + 1]);

	/* these are the remaining arguments not starting with "-" */
	port = curr && *curr ? curr : NULL;

	if (cap_mode_str) {
		if (strcmp(cap_mode_str, "none") == 0)
			cap_mode = CAP_NONE;
		else if (strcmp(cap_mode_str, "fixed") == 0)
			cap_mode = CAP_FIXED;
		else if (strcmp(cap_mode_str, "timed") == 0)
			cap_mode = CAP_TIMED;
		else
			die(1, "Unknown capture mode <%s>. Use -h for help.\n", cap_mode_str);
	}

	if (ts_mode_str) {
		if (strcmp(ts_mode_str, "none") == 0)
			ts_mode = TS_NONE;
		else if (strcmp(ts_mode_str, "abs") == 0)
			ts_mode = TS_ABS;
		else if (strcmp(ts_mode_str, "init") == 0)
			ts_mode = TS_INIT;
		else if (strcmp(ts_mode_str, "line") == 0)
			ts_mode = TS_LINE;
		else
			die(1, "Unknown timestamp mode <%s>. Use -h for help.\n", ts_mode_str);
	}

	if (baud_rate_str) {
		if (strcmp(baud_rate_str, "?") == 0 ||
		    strcmp(baud_rate_str, "help") == 0) {
			list_baud_rates();
			exit(0);
		}
		if (!parse_int(baud_rate_str, &baud))
			die(1, "Failed to parse baud rate <%s>\n", baud_rate_str);
	}

	/* Possibilities:
	 *   - no port (NULL)          : automatically use last one (=-1)
	 *   - numeric port (negative) : use that relative port after discovery
	 *   - numeric port (positive) : use that absolute port after discovery
	 *   - device path             : use that path without discovery
	 */
	if (port && !parse_int(port, &currport)) {
		/* device path */
		currport = -1;
		usepath = 1;
		forced = 1;
	}

	if (currport >= 0)
		forced = 1;

	/* we may need to scan the ports on the system for listing and
	 * automatic discovery.
	 */
	if (!usepath || do_wait_any || do_wait_new || do_list) {
		/* find the first eligible port */
		scan_ports();
	}

	/* when waiting for a port to be available, we check that the port
	 * specified in the path is listed, or that the requested number is
	 * available, otherwise that any port is available.
	 */
	if (do_wait_any && !do_list) {
		if (currport >= 0 && currport < nbports) {
			port = serial_ports[currport].name;
			if (!quiet)
				printf("Port %d (%s) available, using it.\n", currport, port);
			goto done_scan;
		}

		if (usepath) {
			fd = open_port(port);
			if (fd < 0) {
				if (!quiet)
					printf("Waiting for port %s to appear...\n", port);

				do {
					usleep(100000);
					fd = open_port(port);
				} while (fd == -1);
			}

			if (!quiet)
				printf("Trying port %s...", port);
			goto go_with_fd;
		}

		if (!nbports) {
			if (!quiet)
				printf("Waiting for one port to appear...\n");
			do {
				usleep(100000);
				scan_ports();
			} while (!nbports);
		}

		currport = nbports - 1;
		port = serial_ports[currport].name;
		if (!quiet)
			printf("Port %s available, using it.\n", port);

		if (!quiet && !do_list && !do_print)
			list_ports(nbports - 1);
	}

	/* when waiting for new ports, we wait for at least one port to be
	 * added since the previous 100ms. We do explicitly support unplugging
	 * and replugging.
	 */
	if (!usepath && do_wait_new && !do_list) {
		int prev_nbports = nbports;

		if (!quiet)
			printf("%d ports found, waiting for a new one...\n", nbports);

		do {
			usleep(100000);
			scan_ports();
			/* detect if a port was disconnected */
			if (nbports < prev_nbports)
				prev_nbports = nbports;
		} while (nbports <= prev_nbports || nbports <= currport);

		if (!quiet && !do_list && !do_print)
			list_ports(nbports - 1);
	}

	/* port was designated using a number */
	if (currport >= 0 && currport < nbports)
		port = serial_ports[currport].name;

	/* it's time to figure what port we're going to use. Check if the port
	 * is designated by a number. Note that currport remains -1 if the
	 * port was forced via a path.
	 */
	if (!usepath) {
		if (currport >= nbports)
			die(1, "No such port number %d. Use -l to list ports.\n", currport);
		else if (currport < 0) {
			currport = nbports + currport;
			if (currport < 0)
				currport = -1;
		}
	}

done_scan:
	if (do_list) {
		/* list available ports and quit */
		list_ports(-1);
		return 0;
	}

	/* get a real name for the port (if numeric or absent) and warn unless
	 * latest port was explicitly requested.
	 */
	if (currport >= 0) {
		if (!port && !do_print && !quiet && !do_wait_new)
			printf("No port specified, using %s (last registered). Use -l to list ports.\n",
			       serial_ports[currport].name);
		port = serial_ports[currport].name;
	}

	if (do_print) {
		/* print the selected port and quit */
		if (port) {
			printf("%s\n", port);
			return 0;
		}
		return 1;
	}

	if (!port)
		die(2, "No port found nor specified. Use -a to wait or -h for help.\n");

	/* try to open the port */
	do {
		if (!quiet && !(forced && do_wait_new))
			printf("Trying port %s...", port);

		fd = open_port(port);

		if (fd == -1) {
			/* failures have different fallbacks:
			 *  - ENOENT when the port is forced and new is set
			 *    will be retried.
			 *  - EBUSY, ENODEV, ENOENT, ENOMEM will automatically
			 *    be retried if the port was forced.
			 *  - other errors always cause an abort
			 */
			if (errno == ENOENT && forced && do_wait_new) {
				if (!retries)
					printf("Waiting for port %s to appear.\n", port);
				usleep(200000);
				retries++;
				continue;
			}

			if (!forced && (currport > 0) &&
			    (errno == EBUSY || errno == ENODEV || errno == ENOENT || errno == ENOMEM)) {
				if (!quiet) {
					if (forced && do_wait_new)
						printf("Failed to open port %s: %s\n", port, strerror(errno));
					else
						printf(" Failed! (%s)\n", strerror(errno));
				}
				currport--;
				port = serial_ports[currport].name;
				continue;
			}

			if (!quiet) {
				if (forced && do_wait_new)
					printf("Failed to open port %s: %s\n", port, strerror(errno));
				else
					printf(" Failed! (%s)\n", strerror(errno));
			}

			die(2, "Failed to open port: %s\n", strerror(errno));
		}
	} while (fd == -1);

	if (!quiet && (forced && do_wait_new))
		printf("Trying port %s...", port);

go_with_fd:
	ret = set_port(fd, baud);
	if (ret == -1) {
		int err = errno;

		if (!quiet)
			printf(" Failed.\n");

		fprintf(stderr, "Failed to configure port: %s\n", strerror(err));
		if (!quiet && err == EINVAL) {
			printf("Hint: maybe the baud rate is not supported by the port.\n");
			list_baud_rates();
		} else if (!quiet && err == EIO) {
			printf("Hint: the port might be listed but not physically present.\n");
		}
		exit(3);
	}

	if (!quiet) {
		char esc[8];

		printf(" Connected to %s at %d bps.\n", port, get_baud_rate(fd));

		if (escape < 32) {
			switch (escape) {
			case  9: snprintf(esc, sizeof(esc), "Tab"); break;
			case 27: snprintf(esc, sizeof(esc), "Esc"); break;
			default: snprintf(esc, sizeof(esc), "Ctrl-%c", escape + '@'); break;
			}
		}
		else if (isprint(escape))
			snprintf(esc, sizeof(esc), "%c", escape);
		else
			snprintf(esc, sizeof(esc), "0x%02X", escape);

		printf("Escape character is '%s'. Use escape followed by '?' for help.\n", esc);
	}

	if (do_send_break)
		tcsendbreak(fd, 0);

	if (no_term)
		return 0;

	/* set start time of capture just before forwarding */
	gettimeofday(&start_ts, NULL);

	/* perform the actual forwarding */
	forward(fd);

	return 0;
}
