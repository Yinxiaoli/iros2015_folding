#include "criticalsection.h"
#include "criticalsectionimpl.h"

#include <cstddef>

namespace _BlackBill{
	namespace _Thread{

		CCriticalSection::CCriticalSection()
		{
			m_Impl = new CCriticalSectionImpl();
		}

		CCriticalSection::~CCriticalSection()
		{
			delete m_Impl;
			m_Impl = NULL;
		}

		void CCriticalSection::enter()
		{
			m_Impl->enter();
		}

		void CCriticalSection::leave()
		{
			m_Impl->leave();
		}
	};
};