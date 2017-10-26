#ifndef __IMAGE_IO_UTIL_H__
#define __IMAGE_IO_UTIL_H__

#include <blackbill/result/result.h>
using namespace _BlackBill::_Result;

#include <stdint.h>
#include <cmath>
#include <iostream>
using namespace std;

namespace _BlueTail
{
	namespace _Image
	{
		template<typename T, int N>
		struct color2RGB
		{
			BBRESULT operator()(T c[N], uint8_t rgb[3]) { return BR_SUCCESS; }
		};
		
		template<typename T>
		struct color2RGB<T,1>
		{
			BBRESULT operator()(T c[1], uint8_t rgb[3])
			{
				int v = max(0, min(255, int(c[0] * 256)));
				rgb[0] = rgb[1] = rgb[2] = v;
				return BR_SUCCESS; 
			}
		};
		
		template<typename T>
		struct color2RGB<T,2>
		{
			BBRESULT operator()(T c[2], uint8_t rgb[3])
			{
				int v1 = max(0, min(255, int(c[0] * 256)));
				int v2 = max(0, min(255, int(c[1] * 256)));
				rgb[0] = v1; 
				rgb[1] = v2;
				rgb[2] = 0;
				return BR_SUCCESS; 
			}
		};
		
		template<typename T>
		struct color2RGB<T,3>
		{
			BBRESULT operator()(T c[3], uint8_t rgb[3])
			{
				int v1 = max(0, min(255, int(c[0] * 256)));
				int v2 = max(0, min(255, int(c[1] * 256)));
				int v3 = max(0, min(255, int(c[2] * 256)));
				rgb[0] = v1; 
				rgb[1] = v2;
				rgb[2] = v3;
				return BR_SUCCESS; 
			}
		};
	};
};

#endif
