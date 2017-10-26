#include "threadimpl.h"
#include "thread.h"

namespace _BlackBill{
	namespace _Thread{

		CThreadImpl::CThreadImpl(CThread* in_Parent)
#ifdef _WIN32
			: m_hThread(0), m_Parent(in_Parent)
#endif
		{

		}

		CThreadImpl::~CThreadImpl()
		{
#ifdef _WIN32
			if(0 != m_hThread)
				::CloseHandle(m_hThread);
#endif
		}

		void CThreadImpl::begin()
		{
#ifdef _WIN32
			m_hThread = ::CreateThread(NULL, 0, &_ThreadFunc, m_Parent, 0, &m_ThreadID);
#endif
		}

		void CThreadImpl::join()
		{
#ifdef _WIN32
			::WaitForSingleObject(m_hThread, INFINITE);
#endif
		}

		bool CThreadImpl::tryJoin()
		{
#ifdef _WIN32
			DWORD ret = ::WaitForSingleObject(m_hThread, 0);
			if(ret == WAIT_OBJECT_0)
				return true;
			else
				return false;
#endif
		}

		void CThreadImpl::close()
		{
#ifdef _WIN32
			if(0 != m_hThread)
				::CloseHandle(m_hThread);
			m_hThread = 0;
#endif
		}

#ifdef _WIN32
		DWORD WINAPI CThreadImpl::_ThreadFunc(LPVOID in_Pointer)
		{
			CThread* the_Parent = reinterpret_cast<CThread*>(in_Pointer);
			the_Parent->OnThreadRun();

			return 0;
		}
#endif
	};
};