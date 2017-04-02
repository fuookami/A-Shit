#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_io.h"
#include <sys/timeb.h>
#include <time.h>

void deploy_server(char * graph[MAX_EDGE_NUM], int edge_num, char * filename);
unsigned long timeb2ms(const timeb &t);

#endif
