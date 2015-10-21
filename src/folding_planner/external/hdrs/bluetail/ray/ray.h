#ifndef __RAY_H__
#define __RAY_H__

namespace _BlueTail
{
	namespace _Ray
	{
		template<typename T> T _BT_RAY_FAR();

		template<>
		inline float _BT_RAY_FAR() { return 1.0e37f; }
		
		template<>
		inline double _BT_RAY_FAR() { return 1.0e307; }


		template<typename T> T _BT_RAY_FAR_THR();

		template<>
		inline float _BT_RAY_FAR_THR() { return 1.0e36f; }

		template<>
		inline double _BT_RAY_FAR_THR() { return 1.0e306; }


		template<typename T>
		struct SRay
		{
			T o[3];
			T d[3];
			T tMin;
			T tMax;
		};
	};
};

#endif
