#ifndef SQL_IFLYTEK_H_
#define SQL_IFLYTEK_H_

#define DEBUG_SR_SQL

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include "sqlite3.h"

#include <iostream>
#include <cstring>
#include <sstream>

using namespace std;

int create_database(string name);
int create_table(char *sql_create_table);
int create_index(char *sql_create_index);
int table_insert_weight(string time, double weight, string rgbfilename, string pcdfilename, string depthfilename,string batchnumber);

#endif
