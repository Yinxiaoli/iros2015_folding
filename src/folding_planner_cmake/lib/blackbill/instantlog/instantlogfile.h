#ifndef __INSTANT_LOG_FILE_H__
#define __INSTANT_LOG_FILE_H__

#include "instantlog.h"
#include "instantlogformat.h"
#include <fstream>
using namespace std;

namespace _BlackBill{
	namespace _InstantLog{

		template<typename T>
		class CInstantLogFile : public CInstantLog
		{
			CInstantLogFile() {}
		public:
			CInstantLogFile(const char* in_FileName, bool in_Reset = false)
				: m_FileName(NULL)
			{
				m_FileName = new char[strlen(in_FileName)+1];
				strcpy(m_FileName, in_FileName);

				if(in_Reset)
				{
					ofstream ofs;
					ofs.open(in_FileName, ios::out | ios::trunc);
					ofs.close();
				}
			}

			~CInstantLogFile()
			{
				delete[] m_FileName;
			}

			bool putLog(const char* in_Comment, const bool in_Continue = false)
			{
				ofstream ofs;
				ofs.open(m_FileName, ios::out | ios::app);
				
				T the_Format(ofs);
				bool the_ret = the_Format.putLog(in_Comment, in_Continue);

				ofs.close();
				return the_ret;
			}

		private:
			char* m_FileName;
		};

	};
};

#endif
