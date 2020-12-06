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
#include <time.h>
#include <unistd.h>

#define MAXPORTS 100

#ifndef VERSION
#define VERSION "0.0.1"
#endif

const char version_str[] =
	"BootTerm " VERSION " -- the terminal written for users by its users\n"
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
int currport = 0;
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

int main(int argc, char **argv)
{
	const char *progname;
	char *curr = NULL;
	char *arg = NULL;
	int idx, opt;
	int do_list = 0;
	int do_wait_new = 0;
	int do_print = 0;

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

	/* we may need to scan the ports on the system for listing and
	 * automatic discovery.
	 */
	if (!port || do_wait_new || do_list) {
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

		if (!quiet) {
			printf("New port connected:\n");
			list_ports(0);
		}
	}

	/* it's time to figure what port we're going to use. Check if the port
	 * is designated by a number.
	 */
	if (port && !parse_int(port, &currport))
		currport = 0;

	if (!port && nbports > 0)
		port = serial_ports[currport].name;

	if (strncmp(port, "/dev/", 5) == 0)
		port += 5;

	if (do_print) {
		/* print the selected port and quit */
		if (port) {
			printf("%s\n", port);
			return 0;
		}
		return 1;
	}

	if (do_list) {
		/* list available ports and quit */
		list_ports(-1);
		return 0;
	}

	return 0;
}
