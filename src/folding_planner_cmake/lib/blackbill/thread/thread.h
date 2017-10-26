#ifndef __THREAD_H__
#define __THREAD_H__

#include "../delegator/delegatorchain.h"
#include "../delegator/eventhandler2.h"

namespace _BlackBill{
	namespace _Thread{

		class CThreadImpl;

		class CThread
		{
		public:
			CThread();
			~CThread();

			event<> OnThreadRun;

			void begin();
			void join();
			bool tryJoin();
			void close();

		protected:
			CThreadImpl* m_Impl;
		};
	};
};

#endif
