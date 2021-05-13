#include "sql_ifly.h"
#include <time.h>
#include <sys/time.h>

sqlite3 *db = 0;

int create_database(string name)
{
	int res = sqlite3_open(name.c_str(), &db);
	if (res != SQLITE_OK)
	{
		printf("���ݿ��޷���\n");

		res = sqlite3_open(name.c_str(), &db);
		if (res != SQLITE_OK)
		{
			printf("���ݿ��޷���\n");
			system(name.c_str());
			res = sqlite3_open(name.c_str(), &db);
			{
				printf("�޷���������\n");
				return -1;
			}
			return 0;
		}
		return 0;
	}
	return 0;
}

int create_table(char *sql_create_table)
{
	char *error;
	int res = sqlite3_exec(db, sql_create_table, NULL, NULL, &error);
	if (res != SQLITE_OK)
	{
		printf("warning create_table \n");
		return -1;
	}

	return 0;
}

int create_index(char *sql_create_index)
{
	char *error;
	int res = sqlite3_exec(db, sql_create_index, NULL, NULL, &error);
	if (res != SQLITE_OK)
	{
		printf("warning create_index");
		return -1;
	}

	return 0;
}

int table_insert_weight(string time, double weight, string rgbfilename, string pcdfilename, string depthfilename, string batchnumber, double height)
{
	char strSql[1024] = {"\0"};
	stringstream stringSql;

	stringSql << "INSERT INTO weight_records (time,weight,height,batchnumber,rgbfilename,pointfilename,depthfilename) VALUES ("
			  << "'" << time << "',"
			  << weight << ","
			  << height << ","
			  << "'" << batchnumber << "',"
			  << "'" << rgbfilename << "',"
			  << "'" << pcdfilename << "',"
			  << "'" << depthfilename << "');";

	strncpy(strSql, const_cast<char *>(stringSql.str().c_str()), stringSql.str().length());

	char *cErrMsg;
	int nRes = sqlite3_exec(db, strSql, NULL, NULL, &cErrMsg);
	if (nRes != SQLITE_OK)
	{
		printf("sqlite3_exec sql=%s,error=%d:%s\n", strSql, nRes, cErrMsg);
		return -1;
	}

	return 0;
}
