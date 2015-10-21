#ifndef __CRITICAL_SECTION_IMPL_H__
#define __CRITICAL_SECTION_IMPL_H__

#ifdef _WIN32
#include <windows.h>
#endif

namespace _BlackBill{
	namespace _Thread{

		class CCriticalSectionImpl
		{
		public:
			CCriticalSectionImpl();
			~CCriticalSectionImpl();

			void enter();
			void leave();

		protected:
#ifdef _WIN32
			CRITICAL_SECTION m_CriticalSection;
#endif
		};
	};
};

#endif
