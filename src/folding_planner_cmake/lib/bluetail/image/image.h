#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <stdint.h>
#include <blackbill/result/result.h>
using namespace _BlackBill::_Result;

#include <iostream>
using namespace std;

namespace _BlueTail
{
	namespace _Image
	{
		template<typename T, int N>
		struct SImage
		{
			int32_t width;
			int32_t height;
			T* ptr;

			enum {NC = N};

			SImage(): width(0), height(0), ptr(NULL) {}
		};

		template<typename T, int N>
		inline BBRESULT initImage(const int32_t in_Width, const int32_t in_Height, SImage<T, N>& io_Image)
		{
			if((in_Width <= 0) || (in_Height <= 0))
			{
				io_Image.width = 0;
				io_Image.height = 0;
				io_Image.ptr = NULL;
			}
			else
			{
				io_Image.width = in_Width;
				io_Image.height = in_Height;
				io_Image.ptr = (T*)malloc(sizeof(T)*in_Width*in_Height*N);
			}

			return BR_SUCCESS;
		}

		template<typename T, int N>
		inline BBRESULT reallocImage(const int32_t in_Width, const int32_t in_Height, SImage<T, N>& io_Image)
		{
			if((in_Width <= 0) || (in_Height <= 0))
			{
				
			}
			else
			{
				io_Image.width = in_Width;
				io_Image.height = in_Height;
				io_Image.ptr = (T*)realloc(io_Image.ptr, sizeof(T)*in_Width*in_Height*N);
			}

			return BR_SUCCESS;
		}

		template<typename T, int N>
		inline BBRESULT clearImage(SImage<T, N>& io_Image)
		{
			if((io_Image.width <= 0) || (io_Image.height <= 0) || (io_Image.ptr == NULL)) {}
			else
				memset(io_Image.ptr, 0, sizeof(T)*io_Image.width*io_Image.height*N);

			return BR_SUCCESS;
		}

		template<typename T, int N>
		inline BBRESULT finalizeImage(SImage<T, N>& io_Image)
		{
			io_Image.width = 0;
			io_Image.height = 0;
			if(io_Image.ptr != NULL)
			{
				free(io_Image.ptr);
				io_Image.ptr = NULL;
			}

			return BR_SUCCESS;
		}

		template<typename T, int N>
		inline BBRESULT flipImageY(SImage<T, N>& io_Image)
		{
			for(int j=0; j<io_Image.height/2; j++)
			{
				int jb = io_Image.height-1-j;
				for(int i=0; i<io_Image.width; i++)
				{
					for(int k=0; k<N; k++)
					{
						T temp = io_Image.ptr[(j*io_Image.width+i)*N+k];
						io_Image.ptr[(j*io_Image.width+i)*N+k] = io_Image.ptr[(jb*io_Image.width+i)*N+k];
						io_Image.ptr[(jb*io_Image.width+i)*N+k] = temp;
					}
				}
			}

			return BR_SUCCESS;
		}
	};
};

#endif
