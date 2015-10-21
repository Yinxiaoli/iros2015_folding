#ifndef __INSTANT_LOG_FORMAT_H__
#define __INSTANT_LOG_FORMAT_H__

#include <iostream>
using namespace std;

namespace _BlackBill{
	namespace _InstantLog{

		class CInstantLogFormat
		{
			CInstantLogFormat(): m_OS(cout) {}
		public:
			CInstantLogFormat(ostream& os);
			~CInstantLogFormat();

			bool putLog(const char* in_Comment, const bool in_Continue = false);

		protected:
			void timeStampF();
			void timeStamp();

			ostream& m_OS;
			bool m_CR;
			bool m_InitialCR;
		};
	};
};

#endif
