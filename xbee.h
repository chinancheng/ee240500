#ifndef XBEE_H
#define XBEE_H
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <limits.h>
#include <sys/select.h>

#define BUFLEN (255)
#define ZDEBUG (1)
void clean_buffer ();
int openserial (char *sdevfile);
int send (char s);

#endif // XBEE_H

