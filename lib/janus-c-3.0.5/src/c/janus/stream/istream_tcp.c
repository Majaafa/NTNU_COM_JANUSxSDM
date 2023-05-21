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
#include <signal.h>

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
istream_open_connect(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

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
istream_open_listen(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  int wait_conn_fd = -1;
  int opt;
  void (* signal_handler)(int);

  if ((wait_conn_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    pdata->error = errno;
    pdata->error_op = "socket creation error";
    goto istream_listen_error;
  }

  opt = 1;
  if (setsockopt(wait_conn_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof (opt)) < 0)
  {
    pdata->error = errno;
    pdata->error_op = "setting socket parameters";
    goto istream_listen_error;
  }
  
  if (bind(wait_conn_fd,(struct sockaddr *)&pdata->saun, sizeof(pdata->saun)) < 0)
  {
    pdata->error = errno;
    pdata->error_op = "binding socket";
    goto istream_listen_error;
  }

  if (listen(wait_conn_fd, 1) == -1)
  {
    pdata->error = errno;
    pdata->error_op = "listening socket";
    goto istream_listen_error;
  }
  
  signal_handler = signal(SIGINT, SIG_DFL);
  pdata->fd = accept(wait_conn_fd, NULL, NULL);
  signal(SIGTERM, signal_handler);

  if (pdata->fd < 0)
  {
    pdata->error = errno;
    pdata->error_op = "accepting socket connection";
    goto istream_listen_error;
  }

  return JANUS_ERROR_NONE;

istream_listen_error:
  if (wait_conn_fd >= 0) {
      close(wait_conn_fd);
  }
  return JANUS_ERROR_STREAM;

}

static int
istream_open(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  int rv = JANUS_ERROR_STREAM, port;
  char *args;
  char *socket_type, *ip_s, *port_s;

  /* args: [connect|listen]:<ip>:<port> */
  if (istream->args == NULL) {
    pdata->error = EINVAL;
    pdata->error_op = "tcp arguments undefined";
    return rv;
  }
  args = strdup(istream->args);
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
    rv = istream_open_connect(istream);
  } else if (strcmp(socket_type, "listen") == 0) {
    rv = istream_open_listen(istream);
  } else {
    pdata->error = EINVAL;
    pdata->error_op = "connection type undefiend";
  }
  stream_open_finish:
  free(args);
  return rv;
}

static int
istream_close(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;

  if (pdata->fd >= 0)
    close(pdata->fd);

  return JANUS_ERROR_NONE;
}

static void
istream_free(janus_istream_t istream)
{
  free(istream->pdata);
}

static int
istream_read(const janus_istream_t istream, void* frames, unsigned frame_count)
{
  JANUS_ISTREAM_PDATA;
  int rv, offset = 0;
  int requested_length = janus_istream_get_frame_size(istream) * frame_count;

  do {
    rv = read(pdata->fd, &((char*)frames)[offset], requested_length - offset);
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

static int
istream_write(janus_istream_t istream, void* frames, unsigned frame_count)
{
  return 0;
}

static const char* 
istream_get_error(janus_istream_t istream) 
{ 
  JANUS_ISTREAM_PDATA;
  return strerror(pdata->error);
}

static const char*
istream_get_error_op(janus_istream_t istream)
{
  JANUS_ISTREAM_PDATA;
  return pdata->error_op;
}

int
janus_istream_tcp_new(janus_istream_t istream)
{
  istream->pdata = calloc(1, sizeof(struct private_data));
  istream->open = istream_open;
  istream->close = istream_close;
  istream->free = istream_free;
  istream->read = istream_read;
  istream->write = istream_write;
  istream->get_error = istream_get_error;
  istream->get_error_op = istream_get_error_op;
  strcpy(istream->name, "TCP");

  return JANUS_ERROR_NONE;
}
