#include "criticalsectionimpl.h"

namespace _BlackBill{
	namespace _Thread{

		CCriticalSectionImpl::CCriticalSectionImpl()
		{
#ifdef _WIN32
			InitializeCriticalSection(&m_CriticalSection);
#endif
		}

		CCriticalSectionImpl::~CCriticalSectionImpl()
		{
#ifdef _WIN32
			DeleteCriticalSection(&m_CriticalSection);
#endif
		}

		void CCriticalSectionImpl::enter()
		{
#ifdef _WIN32
			EnterCriticalSection(&m_CriticalSection);
#endif
		}

		void CCriticalSectionImpl::leave()
		{
#ifdef _WIN32
			LeaveCriticalSection(&m_CriticalSection);
#endif
		}
	};
};