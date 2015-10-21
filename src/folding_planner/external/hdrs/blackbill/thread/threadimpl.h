#ifndef __THREADIMPL_H__
#define __THREADIMPL_H__

#include "../delegator/delegatorchain.h"

#ifdef _WIN32
#include <windows.h>
#endif

namespace _BlackBill{
	namespace _Thread{

		class CThread;

		class CThreadImpl
		{
			CThreadImpl();
		public:
			CThreadImpl(CThread* in_Parent);
			~CThreadImpl();

			void begin();
			void join();
			bool tryJoin();
			void close();

		protected:
#ifdef _WIN32
			HANDLE m_hThread;
			DWORD m_ThreadID;

			static DWORD WINAPI _ThreadFunc(LPVOID in_Pointer);
#endif

			CThread* m_Parent;
		};
	};
};

#endif
