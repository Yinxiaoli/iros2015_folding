#ifndef __CRITICAL_SECTION_H__
#define __CRITICAL_SECTION_H__

namespace _BlackBill{
	namespace _Thread{

		class CCriticalSectionImpl;

		class CCriticalSection
		{
		public:
			CCriticalSection();
			~CCriticalSection();

			void enter();
			void leave();

		protected:
			CCriticalSectionImpl* m_Impl;
		};
	};
};

#endif
