#include "thread.h"
#include "threadimpl.h"

#include <cstddef>

namespace _BlackBill{
	namespace _Thread{

		CThread::CThread()
		{
			m_Impl = new CThreadImpl(this);
		}

		CThread::~CThread()
		{
			delete m_Impl;
			m_Impl = NULL;
		}

		void CThread::begin()
		{
			m_Impl->begin();
		}

		void CThread::join()
		{
			m_Impl->join();
		}

		bool CThread::tryJoin()
		{
			return m_Impl->tryJoin();
		}

		void CThread::close()
		{
			m_Impl->close();
		}
	};
};
