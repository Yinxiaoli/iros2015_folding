#ifndef __FILEPATH_H__
#define __FILEPATH_H__

#include <string>
using namespace std;

namespace _BlackBill{
	namespace _FilePath{

		void concatFilePath(char* out_Path, const char* in_PathA, const char* in_PathB);
		string getFileExt(const char* in_FileName);
		string getFilePath(const char* in_FullPath);
		string getFileName(const char* in_FullPath);

		bool checkFileExt(const char* in_FullPath, const char* in_Ext);
	};
};

#endif
