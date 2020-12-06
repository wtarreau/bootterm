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
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(__linux__) && !defined(NO_TCGETS2)
#include <sys/ioctl.h>
#include <asm/termbits.h>
#else
#include <termios.h>
#endif
#include <time.h>
#include <unistd.h>

#define MAXPORTS 100

#ifndef VERSION
#define VERSION "0.0.1"
#endif

#ifndef CRTSCTS
#define CRTSCTS 0
#endif

const char version_str[] =
	"BootTerm " VERSION " -- the terminal written for users by its users\n"
#ifdef TCGETS2
	"Built with support for custom baud rates (TCGETS2).\n"
#endif
	"Copyright (C) 2020 Willy Tarreau <w@1wt.eu>\n"
	"This is free software: you are encouraged to improve and redistribute it.\n"
	"There is no warranty of any kind, see the source for license details.\n";

const char usage_msg[] =
	"Usage: %s [options] [/dev/ttyXXX | port#]\n"
	"Options:\n"
	"  -h           display this help\n"
	"  -q           quiet mode: do not report events\n"
	"  -p           only print selected port name and quit\n"
	"  -l           list detected serial ports and quit\n"
	"  -n           wait for a new port to be registered\n"
	"  -m <min>     specify lowest printable character  (default: 0)\n"
	"  -M <max>     specify highest printable character (default: 255)\n"
	"  -b <baud>    specify serial port's baud rate (default=0: port's current)\n"
	"  -c {none|fixed|timed} capture to file (default: none)\n"
	"  -e <escape>  escape character or ASCII code  (default: 29 = Ctrl-])\n"
	"  -f <fmt>     capture file name (default: 'bootterm-%%Y%%m%%d-%%H%%M%%S.log')\n"
	"  -V           show version and license\n"
	"\n"
	"Ports are sorted in reverse registration order so that port 0 is the most\n"
	"recently added one. A number may be set instead of the port. With no name nor\n"
	"number, port 0 is automatically used.\n"
	"";

enum cap_mode {
	CAP_NONE = 0,
	CAP_FIXED,
	CAP_TIMED,
};

struct serial {
	char *name;            // device name without /dev
	char *driver;          // driver name (usually a short word)
	char *model;           // model name when reported
	time_t ctime;          // device attachment date
};

/* operating modes */
struct serial serial_ports[MAXPORTS];
enum cap_mode cap_mode = CAP_NONE;
const char *capture_fmt = "bootterm-%Y%m%d-%H%M%S.log";
unsigned char escape  = 0x1d; // Ctrl-]
unsigned char chr_min = 0;
unsigned char chr_max = 255;
const char *port = NULL;
int nbports = 0;
int currport = -1; // last one
int quiet = 0;
int baud = 0;

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


/* reverse-sorting function for serial ports: ensures the most recently
 * registered port appears first. If some are equal, the device name is
 * used instead for normal ordering.
 */
static int serial_port_cmp(const void *a, const void *b)
{
	const struct serial *pa = (const struct serial *)a;
	const struct serial *pb = (const struct serial *)b;
	if (pb->ctime != pa->ctime)
		return pb->ctime - pa->ctime;
	return strcmp(pa->name, pb->name);
}

/* scans available ports, fills serial_ports[] and returns the number of
 * entries filled in (also stored in nbports).
 */
int scan_ports()
{
	struct dirent *ent;
	char ftmp[PATH_MAX];
	struct stat st;
	DIR *dir = NULL;
	char *link, *driver, *model, *name;

	nbports = 0;

	dir = opendir("/sys/class/tty");
	if (!dir)
		goto end;

	while (nbports < MAXPORTS) {
		ent = readdir(dir);
		if (!ent)
			break;

		link = driver = model = name = NULL;
		snprintf(ftmp, sizeof(ftmp), "/sys/class/tty/%s/device/.", ent->d_name);
		if (stat(ftmp, &st) == 0) {
			link = read_link_from("/sys/class/tty/%s/device/driver", ent->d_name);
			if (!link)
				goto fail;
			driver = strrchr(link, '/');
			if (driver)
				driver++;
			else
				driver = link;

			driver = strdup(driver);
			free(link); link = NULL;

			name = strdup(ent->d_name);
			if (!name)
				goto fail;

			/* the model name usually appears in ../interface for ttyUSB or
			 * ./interface for ttyACM
			 */
			model = read_line_from("/sys/class/tty/%s/device/../interface", ent->d_name);
			if (!model)
				model = read_line_from("/sys/class/tty/%s/device/interface", ent->d_name);
			/* note: the model is not always set, so we accept NULL */

			serial_ports[nbports].name = name;
			serial_ports[nbports].driver = driver;
			serial_ports[nbports].model = model;
			serial_ports[nbports].ctime = st.st_ctime;
			nbports++;
			continue;
		}
	fail:
		free(driver);
		free(link);
		free(model);
		free(name);

	}
end:
	if (dir)
		closedir(dir);

	if (nbports)
		qsort(serial_ports, nbports, sizeof(serial_ports[0]), serial_port_cmp);

	return nbports;
}

/* list all ports if portspec < 0 or just this port. The current port is never
 * reported if a single one is requested.
 */
void list_ports(int portspec)
{
	time_t now;
	int p;

	printf(" port |  age (sec) | device     | driver           | model                \n"
	       "------+------------+------------+------------------+----------------------\n"
		);

	now = time(NULL);
	for (p = (portspec < 0 ? 0 : portspec); p < (portspec < 0 ? nbports : portspec + 1); p++) {
		printf(" %c%3d | %10u | %-10s | %-16s | %-16s \n",
		       portspec < 0 && p == currport ? '*' : ' ',
		       p,
		       (unsigned int)(now - serial_ports[p].ctime),
		       serial_ports[p].name,
		       serial_ports[p].driver,
		       serial_ports[p].model ? serial_ports[p].model : "");
	}
	putchar('\n');
}

/* Try to open this port (prepending /dev if the path doesn't start with '/').
 * The FD is returned on success, otherwise -1 with errno set appropriately.
 */
int open_port(const char *port)
{
	char ftmp[PATH_MAX];
	int fd;

	if (snprintf(ftmp, sizeof(ftmp),
		     (*port == '/') ? "%s" : "/dev/%s",
		     port) > sizeof(ftmp)) {
		errno = ENAMETOOLONG;
		return -1;
	}

	fd = open(ftmp, O_RDWR | O_NONBLOCK | O_NOCTTY);
	return fd;
}

#ifdef TCSETS2 // since linux 2.6.20 (commit edc6afc54)
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

	return ioctl(fd, TCSETS2, &tio);
}

#else /* TCSETS2 not defined */

/* returns a combination of Bxxxx flags to set on termios c_cflag depending on
 * the baud rate values supported on the platform. Only exact values match. If
 * no value is found, (tcflag_t)~0 is returned, assuming it will never match an
 * existing value on any platform.
 */
tcflag_t baud_encode(int baud)
{
	struct {
		int b;      // baud rate
		tcflag_t f; // corresponding flag
	} speeds[] = {
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
	int idx;

	for (idx = 0; speeds[idx].b; idx++) {
		if (speeds[idx].b == baud)
			return speeds[idx].f;
	}
	return speeds[idx].f;
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
	return tcsetattr(fd, TCSANOW, &tio);
}
#endif // TCSETS2 not defined

int main(int argc, char **argv)
{
	const char *progname;
	char *curr = NULL;
	char *arg = NULL;
	int idx, opt;
	int do_list = 0;
	int do_wait_new = 0;
	int do_print = 0;
	int fd, ret;

	progname = strrchr(argv[0], '/');
	if (!progname)
		progname = argv[0];
	else
		progname++;

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

		case 'l':
			do_list = 1;
			break;

		case 'p':
			do_print = 1;
			break;

		case 'q':
			quiet = 1;
			break;

		case 'n':
			do_wait_new = 1;
			break;

		case 'b':
			if (!parse_int(arg, &baud))
				die(1, "failed to parse argument for -%c\n", opt);
			arg = NULL;
			break;

		case 'c':
			if (!arg)
				die(1, "missing argument for -%c\n", opt);
			if (strcmp(arg, "none") == 0)
				cap_mode = CAP_NONE;
			else if (strcmp(arg, "fixed") == 0)
				cap_mode = CAP_FIXED;
			else if (strcmp(arg, "timed") == 0)
				cap_mode = CAP_TIMED;
			else
				die(1, "unknown argument for -%c\n", opt);
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

		default:
			die(1, "Unknown option '%c'. Use -h to get help\n", opt);
			break;
		}
	}

	if (argc > idx + 1)
		die(1, "Too many arguments: '%s'. Use -h to get help\n", argv[idx + 1]);

	/* these are the remaining arguments not starting with "-" */
	port = curr && *curr ? curr : NULL;

	if (port && !parse_int(port, &currport))
		currport = -1;

	/* we may need to scan the ports on the system for listing and
	 * automatic discovery.
	 */
	if (!port || currport >= 0 || do_wait_new || do_list) {
		/* find the first eligible port */
		scan_ports();
	}

	/* when waiting for new ports, we wait for at least one port to be
	 * added since the previous 100ms. We do explicitly support unplugging
	 * and replugging.
	 */
	if (do_wait_new) {
		int prev_nbports = nbports;

		if (!quiet)
			printf("%d ports found, waiting for a new one...\n", nbports);

		do {
			usleep(100000);
			scan_ports();
			/* detect if a port was disconnected */
			if (nbports < prev_nbports)
				prev_nbports = nbports;
		} while (nbports <= prev_nbports);

		if (!quiet && !do_list && !do_print)
			list_ports(nbports - 1);
	}

	/* port was designated using a number */
	if (currport >= 0 && currport < nbports)
		port = serial_ports[currport].name;

	/* it's time to figure what port we're going to use. Check if the port
	 * is designated by a number.
	 */
	if (currport < 0) {
		currport = nbports + currport;
		if (currport < 0)
			currport = 0;
	}

	if (do_list) {
		/* list available ports and quit */
		list_ports(-1);
		return 0;
	}

	/* try to open the port */
	do {
		if (!port && nbports > 0) {
			port = serial_ports[currport].name;
			if (!do_print && !quiet && !do_wait_new) {
				printf("No port specified, using %s (last registered). Use -l to list ports.\n", port);
			}
		}

		if (do_print) {
			/* print the selected port and quit */
			if (port) {
				printf("%s\n", port);
				return 0;
			}
			return 1;
		}

		if (!quiet)
			printf("Trying port %s...", port);

		fd = open_port(port);

		if (fd == -1) {
			/* failures have different fallbacks:
			 *  - EBUSY, ENODEV, ENOENT, ENOMEM will automatically
			 *    be retried if the port was chosen automatically.
			 *  - other errors always cause an abort
			 */
			if ((curr && *curr) && (currport > 0) &&
			    (errno == EBUSY || errno == ENODEV || errno == ENOENT || errno == ENOMEM)) {
				currport--;
				if (!quiet)
					printf(" Failed! (%s)\n", strerror(errno));
				continue;
			}
			die(2, "Failed to open port: %s\n", strerror(errno));
		}
	} while (fd == -1);

	if (!quiet)
		printf(" Connected.\n");

	ret = set_port(fd, baud);
	if (ret == -1)
		die(3, "Failed to configure port: %s\n", strerror(errno));

	return 0;
}
