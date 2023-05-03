//*************************************************************************
// JANUS is a simple, robust, open standard signalling method for         *
// underwater communications. See <http://www.januswiki.org> for details. *
//*************************************************************************
// Example software implementations provided by STO CMRE are subject to   *
// Copyright (C) 2008-2013 STO Centre for Maritime Research and           *
// Experimentation (CMRE)                                                 *
//                                                                        *
// This is free software: you can redistribute it and/or modify it        *
// under the terms of the GNU General Public License version 3 as         *
// published by the Free Software Foundation.                             *
//                                                                        *
// This program is distributed in the hope that it will be useful, but    *
// WITHOUT ANY WARRANTY; without even the implied warranty of FITNESS     *
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for       *
// more details.                                                          *
//                                                                        *
// You should have received a copy of the GNU General Public License      *
// along with this program. If not, see <http://www.gnu.org/licenses/>.   *
//*************************************************************************
// Author: Oleksiy Kebkal                                                 *
//*************************************************************************

// ISO C headers.
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>             /* strerror */
#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>          /* waitpid() */
#include <sys/un.h>
#include <sys/wait.h>           /* waitpid() */

// JANUS headers.
#include <janus/janus.h>

#define ERR_STR_BUFFER_SIZE 128

/* server side TCP */

struct private_data
{
  // Code of last error.
  int error;
  // Last error operation.
  const char* error_op;

  // socket handle.
  int fd;
  // socket parameters
  struct sockaddr_in saun;
  //! Error string buffer.
  char error_str[ERR_STR_BUFFER_SIZE];
};

static int
ostream_open_connect(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  if ((pdata->fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    pdata->error = errno;
    pdata->error_op = "socket creation error";
    return JANUS_ERROR_STREAM;
  }

  /* TODO: add retry count here */
  if (connect(pdata->fd, (struct sockaddr *)&pdata->saun, sizeof(pdata->saun)) == -1)
  {
    close(pdata->fd);
    pdata->error = errno;
    pdata->error_op = "connecting socket";
    return JANUS_ERROR_STREAM;
  }
  return JANUS_ERROR_NONE;
}

static int
ostream_open_listen(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  int wait_conn_fd = -1;
  int opt;

  if ((wait_conn_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    pdata->error = errno;
    pdata->error_op = "socket creation error";
    goto ostream_listen_error;
  }

  opt = 1;
  if (setsockopt(wait_conn_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof (opt)) < 0)
  {
    pdata->error = errno;
    pdata->error_op = "setting socket parameters";
    goto ostream_listen_error;
  }
  
  if (bind(wait_conn_fd,(struct sockaddr *)&pdata->saun, sizeof(pdata->saun)) < 0)
  {
    pdata->error = errno;
    pdata->error_op = "binding socket";
    goto ostream_listen_error;
  }

  if (listen(wait_conn_fd, 1) == -1)
  {
    pdata->error = errno;
    pdata->error_op = "listening socket";
    goto ostream_listen_error;
  }
  
  pdata->fd = accept(wait_conn_fd, NULL, NULL);
  if (pdata->fd < 0)
  {
    pdata->error = errno;
    pdata->error_op = "accepting socket connection";
    goto ostream_listen_error;
  }

  return JANUS_ERROR_NONE;

ostream_listen_error:
  if (wait_conn_fd >= 0) {
      close(wait_conn_fd);
  }
  return JANUS_ERROR_STREAM;

}

static int
ostream_open(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  int rv = JANUS_ERROR_STREAM, port;
  char *args;
  char *socket_type, *ip_s, *port_s;

  /* args: [connect|listen]:<ip>:<port> */
  if (ostream->args == NULL) {
    pdata->error = EINVAL;
    pdata->error_op = "tcp arguments undefined";
    return rv;
  }
  args = strdup(ostream->args);
  socket_type = strtok(args, ":");
  ip_s = strtok(NULL, ":");
  port_s = strtok(NULL, ":");
  if (!socket_type || !ip_s || !port_s) {
    pdata->error = EINVAL;
    pdata->error_op = "arguments parsing error";
    goto stream_open_finish;
  }
  port = atoi(port_s);
  
  pdata->saun.sin_family = AF_INET;
  pdata->saun.sin_addr.s_addr = inet_addr(ip_s);
  pdata->saun.sin_port = htons(port);

  if (strcmp(socket_type, "connect") == 0) {
    rv = ostream_open_connect(ostream);
  } else if (strcmp(socket_type, "listen") == 0) {
    rv = ostream_open_listen(ostream);
  } else {
    pdata->error = EINVAL;
    pdata->error_op = "connection type undefiend";
  }
  stream_open_finish:
  free(args);
  return rv;
}

static int
ostream_close(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;

  if (pdata->fd >= 0) {
    close(pdata->fd);
  }
  return JANUS_ERROR_NONE;
}

static void
ostream_free(janus_ostream_t ostream)
{
  free(ostream->pdata);
}

static int
ostream_read(const janus_ostream_t ostream, void* frames, unsigned frame_count)
{
  return 0;
}

static int
ostream_write(janus_ostream_t ostream, void* frames, unsigned frame_count)
{
  JANUS_OSTREAM_PDATA;
  int rv, offset = 0;
  int requested_length = janus_ostream_get_frame_size(ostream) * frame_count;

  do {
    rv = write(pdata->fd, &((char*)frames)[offset], requested_length - offset);
    if (rv > 0) {
      offset += rv;
    } else {
      if (rv < 0 && (errno == EAGAIN || errno == EINTR)) continue;
      break;
    }
  } while (offset < requested_length);
  
  if (offset == requested_length) {
    return frame_count;
  } else {
    return JANUS_ERROR_STREAM;
  }
}

static const char* 
ostream_get_error(janus_ostream_t ostream) 
{ 
  JANUS_OSTREAM_PDATA;
  return strerror(pdata->error);
}

static const char*
ostream_get_error_op(janus_ostream_t ostream)
{
  JANUS_OSTREAM_PDATA;
  return pdata->error_op;
}

static int
ostream_reset(janus_ostream_t ostream)
{
  return JANUS_ERROR_NONE;
}

int
janus_ostream_tcp_new(janus_ostream_t ostream)
{
  ostream->pdata = calloc(1, sizeof(struct private_data));
  ostream->open = ostream_open;
  ostream->close = ostream_close;
  ostream->reset = ostream_reset;
  ostream->free = ostream_free;
  ostream->read = ostream_read;
  ostream->write = ostream_write;
  ostream->get_error = ostream_get_error;
  ostream->get_error_op = ostream_get_error_op;
  strcpy(ostream->name, "TCP");

  return JANUS_ERROR_NONE;
}
