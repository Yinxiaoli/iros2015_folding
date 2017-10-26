#include "instantlogformat.h"
#include <time.h>
#include <iomanip>

namespace _BlackBill{
	namespace _InstantLog{

		CInstantLogFormat::CInstantLogFormat(ostream& os)
			: m_CR(false), m_InitialCR(false), m_OS(os)
		{

		}

		CInstantLogFormat::~CInstantLogFormat()
		{

		}

		void CInstantLogFormat::timeStampF()
		{
			char  wday_name[][4] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };

			time_t the_Time;
			time(&the_Time);

			struct tm  local;
			localtime_s(&local, &the_Time); 

			m_OS << setw(4) << local.tm_year + 1900 << "/"
				<< setfill('0') << setw(2) << local.tm_mon + 1 << "/"
				<< setfill('0') << setw(2) << local.tm_mday << " [" << wday_name[local.tm_wday] << "], "
				<< setfill('0') << setw(2) << local.tm_hour << ":"
				<< setfill('0') << setw(2) << local.tm_min << ":"
				<< setfill('0') << setw(2) << local.tm_sec << ":" << " -BGN- ";
		}

		void CInstantLogFormat::timeStamp()
		{
			char  wday_name[][4] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };

			time_t the_Time;
			time(&the_Time);

			struct tm  local;
			localtime_s(&local, &the_Time); 

			m_OS << setw(4) << local.tm_year + 1900 << "/"
				<< setfill('0') << setw(2) << local.tm_mon + 1 << "/"
				<< setfill('0') << setw(2) << local.tm_mday << " [" << wday_name[local.tm_wday] << "], "
				<< setfill('0') << setw(2) << local.tm_hour << ":"
				<< setfill('0') << setw(2) << local.tm_min << ":"
				<< setfill('0') << setw(2) << local.tm_sec << ":" << " -   - ";
		}

		bool CInstantLogFormat::putLog(const char *in_Comment, const bool in_Continue)
		{
			if(in_Comment == NULL)
				return true;

			if(!m_InitialCR)
			{
				m_OS << endl;
				m_CR = true;
				m_InitialCR = true;

				timeStampF();
				m_CR = false;
			}
			else
			{
				if((!m_CR) && (!in_Continue))
				{
					m_OS << endl;
					m_CR = true;
				}

				if(!in_Continue)
				{
					timeStampF();
					m_CR = false;
				}
			}

			const int len = strlen(in_Comment)+1;
			char* buf = new char[len];
			strcpy_s(buf, len, in_Comment);

			bool isLastCR;
			if((strlen(in_Comment) > 0) && (in_Comment[strlen(in_Comment)-1] == '\n')) isLastCR = true; else isLastCR = false;

			char* p = strtok(buf, "\n");
			while(p != NULL)
			{
				m_OS << p;
				m_CR = false;
				p = strtok(NULL, "\n");
				if(p != NULL)
				{
					m_OS << endl;
					m_CR = true;
					timeStamp();
					m_CR = false;
				}
			}

			if(isLastCR)
			{
				m_OS << endl;
				m_CR = true;
				timeStamp();
				m_CR = false;
			}

			delete[] buf;

			return true;
		}
	};
};
