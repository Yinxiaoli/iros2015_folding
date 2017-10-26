#include "./filepath.h"
#include <string.h>

namespace _BlackBill{
	namespace _FilePath{

		void concatFilePath(char* out_Path, const char* in_PathA, const char* in_PathB)
		{
			strcpy(out_Path, in_PathA);

			int plast = strlen(out_Path)-1;

			if(out_Path[plast] != '/')
			{
				out_Path[plast+1] = '/';
				out_Path[plast+2] = 0;
			}

			if((strlen(in_PathB) >= 2) && (in_PathB[0] == '.') && (in_PathB[1] == '/'))
			{
				strcat(out_Path, &in_PathB[2]);
			}
			else if((strlen(in_PathB) >= 3) && (in_PathB[0] == '.') && (in_PathB[1] == '.') && (in_PathB[2] == '/'))
			{

			}
			else
			{
				strcat(out_Path, in_PathB);
			}
		}

		string getFileExt(const char* in_FileName)
		{
			const char* the_Pos = strrchr(in_FileName, '.');
			if(the_Pos != NULL)
			{
				the_Pos++;
				char* buf = new char[strlen(the_Pos)+1];
				strcpy(buf, the_Pos);
				string the_res = buf;
				delete[] buf;
				return the_res;
			}
			else
				return "";
		}

		string getFilePath(const char* in_FullPath)
		{
			const char* the_Pos = strrchr(in_FullPath, '/');
			if(the_Pos != NULL)
			{
				the_Pos++;
				char* buf = new char[strlen(in_FullPath)-strlen(the_Pos)+1];
				strncpy(buf, in_FullPath, strlen(in_FullPath)-strlen(the_Pos));
				buf[strlen(in_FullPath)-strlen(the_Pos)] = 0;
				string the_res = buf;
				delete[] buf;
				return the_res;
			}

			the_Pos = strrchr(in_FullPath, '\\');
			if(the_Pos != NULL)
			{
				the_Pos++;
				char* buf = new char[strlen(in_FullPath)-strlen(the_Pos)+1];
				strncpy(buf, in_FullPath, strlen(in_FullPath)-strlen(the_Pos));
				buf[strlen(in_FullPath)-strlen(the_Pos)] = 0;
				string the_res = buf;
				delete[] buf;
				return the_res;
			}
			
			return "";
		}

		string getFileName(const char* in_FullPath)
		{
			const char* the_Pos = strrchr(in_FullPath, '/');
			if(the_Pos != NULL)
			{
				the_Pos++;
				char* buf = new char[strlen(the_Pos)+1];
				strcpy(buf, the_Pos);
				string the_res = buf;
				delete[] buf;
				return the_res;
			}

			the_Pos = strrchr(in_FullPath, '\\');
			if(the_Pos != NULL)
			{
				the_Pos++;
				char* buf = new char[strlen(the_Pos)+1];
				strcpy(buf, the_Pos);
				string the_res = buf;
				delete[] buf;
				return the_res;
			}
			
			return "";
		}

		bool checkFileExt(const char* in_FullPath, const char* in_Ext)
		{
			const string the_Ext = getFileExt(in_FullPath);
			char* the_Buf = new char[the_Ext.length() + 1];
			strcpy(the_Buf, the_Ext.data());

			char* the_Buf2 = new char[strlen(in_Ext) + 1];
			strcpy(the_Buf2, in_Ext);

            const bool the_Ret = (strcmp(the_Buf, the_Buf2) == 0);

			delete[] the_Buf;
			delete[] the_Buf2;

			return the_Ret;
		}
	};
};

