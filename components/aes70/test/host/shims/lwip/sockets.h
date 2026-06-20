/* Host shim: map lwIP sockets onto the host BSD sockets (only send() is used
 * by the tested sources, for the wake self-pipe, and the tests never exercise
 * that path). */
#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
