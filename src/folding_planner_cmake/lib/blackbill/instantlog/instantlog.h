#ifndef __INSTANT_LOG_H__
#define __INSTANT_LOG_H__

#include <iostream>
using namespace std;

namespace _BlackBill{
	namespace _InstantLog{

		class CInstantLog
		{
		public:
			virtual bool putLog(const char* in_Comment, const bool in_Continue = false) = 0;
		};

		inline void putLog(const char* in_Comment, CInstantLog* in_InstantLog = NULL, const bool in_Continue = false)
		{
			if(in_InstantLog == NULL) cout << in_Comment << endl; 
			else in_InstantLog->putLog(in_Comment, in_Continue);
		}
	};
};

#endif
