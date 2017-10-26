#ifndef __IMAGE_IO_HDR_H__
#define __IMAGE_IO_HDR_H__

#include <blackbill/result/result.h>
using namespace _BlackBill::_Result;
#include "./image.h"

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <string.h>
using namespace std;

namespace _BlueTail
{
	namespace _Image
	{
		template<typename T>
		inline void float2rgbe(unsigned char rgbe[4], T red, T green, T blue)
		{
			const T v = max(max(red, blue), green);
			if(v < T(1e-32)) {
				rgbe[0] = rgbe[1] = rgbe[2] = rgbe[3] = 0;
			}
			else
			{
				int32_t e;
				const T _v = frexp(v, &e) * T(256.0)/v;
				rgbe[0] = (unsigned char) (red * _v);
				rgbe[1] = (unsigned char) (green * _v);
				rgbe[2] = (unsigned char) (blue * _v);
				rgbe[3] = (unsigned char) (e + 128);
			}
		}

		template<typename T>
		inline void rgbe2float(T *red, T *green, T *blue, unsigned char rgbe[4])
		{
			if(rgbe[3])
			{   /*nonzero pixel*/
				const T f = ldexp(T(1.0),rgbe[3]-(int32_t)(128+8));
				*red = rgbe[0] * f;
				*green = rgbe[1] * f;
				*blue = rgbe[2] * f;
			}
			else
				*red = *green = *blue = T(0.0);
		}

		template<typename T>
		inline BBRESULT hdrWriteHeader(FILE* in_F, int32_t in_Width, int32_t in_Height, T in_Gamma, T in_Exposure)
		{
			if(fprintf(in_F,"#?RGBE\n") < 0)
				return BR_FAIL;
			if(fprintf(in_F,"GAMMA=%g\n", in_Gamma) < 0)
				return BR_FAIL;
			if(fprintf(in_F,"EXPOSURE=%g\n", in_Exposure) < 0)
				return BR_FAIL;
			if(fprintf(in_F,"FORMAT=32-bit_rle_rgbe\n\n") < 0)
				return BR_FAIL;
			if(fprintf(in_F, "-Y %d +X %d\n", in_Height, in_Width) < 0)
				return BR_FAIL;
			return BR_SUCCESS;
		}

		template<typename T, int N>
		struct hdrWritePixels
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_NumPixels) { return BR_SUCCESS; }
		};

		template<typename T>
		struct hdrWritePixels<T, 1>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];
				while(in_NumPixels-- > 0)
				{
					float2rgbe<T>(rgbe, in_Data[0], in_Data[0], in_Data[0]);
					in_Data += 1;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
				}
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrWritePixels<T, 2>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];
				while(in_NumPixels-- > 0)
				{
					float2rgbe<T>(rgbe, in_Data[0], in_Data[1], 0.0);
					in_Data += 2;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
				}
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrWritePixels<T, 3>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];
				while(in_NumPixels-- > 0)
				{
					float2rgbe<T>(rgbe, in_Data[0], in_Data[1], in_Data[2]);
					in_Data += 3;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
				}
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrWritePixels<T, 4>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];
				while(in_NumPixels-- > 0)
				{
					float2rgbe<T>(rgbe, in_Data[0], in_Data[1], in_Data[2]);
					in_Data += 4;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
				}
				return BR_SUCCESS;
			}
		};

		inline BBRESULT hdrWriteBytesRLE(FILE* in_F, unsigned char* in_Bytes, int32_t in_NumBytes)
		{
			const int32_t MINRUNLENGTH = 4;
			int32_t cur, beg_run, run_count, old_run_count, nonrun_count;
			unsigned char buf[2];

			cur = 0;
			while(cur < in_NumBytes) {
				beg_run = cur;
				/* find next run of length at least 4 if one exists */
				run_count = old_run_count = 0;
				while((run_count < MINRUNLENGTH) && (beg_run < in_NumBytes)) {
					beg_run += run_count;
					old_run_count = run_count;
					run_count = 1;
					while( (beg_run + run_count < in_NumBytes) && (run_count < 127)
						&& (in_Bytes[beg_run] == in_Bytes[beg_run + run_count]))
						run_count++;
				}
				/* if data before next big run is a short run then write it as such */
				if((old_run_count > 1)&&(old_run_count == beg_run - cur)) {
					buf[0] = 128 + old_run_count;   /*write short run*/
					buf[1] = in_Bytes[cur];
					if(fwrite(buf,sizeof(buf[0])*2,1,in_F) < 1)
						return BR_FAIL;
					cur = beg_run;
				}
				/* write out bytes until we reach the start of the next run */
				while(cur < beg_run) {
					nonrun_count = beg_run - cur;
					if(nonrun_count > 128) 
						nonrun_count = 128;
					buf[0] = nonrun_count;
					if(fwrite(buf, sizeof(buf[0]), 1, in_F) < 1)
						return BR_FAIL;
					if(fwrite(&in_Bytes[cur], sizeof(in_Bytes[0])*nonrun_count, 1, in_F) < 1)
						return BR_FAIL;
					cur += nonrun_count;
				}
				/* write out next run if one was found */
				if(run_count >= MINRUNLENGTH) {
					buf[0] = 128 + run_count;
					buf[1] = in_Bytes[beg_run];
					if(fwrite(buf, sizeof(buf[0])*2, 1, in_F) < 1)
						return BR_FAIL;
					cur += run_count;
				}
			}
			return BR_SUCCESS;
		}

		template<typename T, int N>
		struct hdrWritePixelsRLE
		{
			BBRESULT operator()(FILE* in_F, T* in_Pixels, int32_t in_LineWidth, int32_t in_NumLines) { return BR_SUCCESS; }
		};

		template<typename T>
		struct hdrWritePixelsRLE<T, 1>
		{
			BBRESULT operator()(FILE* in_F, T* in_Pixels, int32_t in_LineWidth, int32_t in_NumLines)
			{
				unsigned char rgbe[4];
				unsigned char *buffer;
				int32_t i;

				if((in_LineWidth < 8)||(in_LineWidth > 0x7fff))
					/* run length encoding is not allowed so write flat*/
					return hdrWritePixels<T, 1>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				buffer = (unsigned char*)malloc(sizeof(unsigned char) * 4 * in_LineWidth);
				if(buffer == NULL)
					/* no buffer space so write flat */
					return hdrWritePixels<T, 1>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				while(in_NumLines-- > 0)
				{
					rgbe[0] = 2;
					rgbe[1] = 2;
					rgbe[2] = in_LineWidth >> 8;
					rgbe[3] = in_LineWidth & 0xFF;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(buffer);
						return BR_FAIL;
					}
					for(i=0; i<in_LineWidth; i++)
					{
						float2rgbe(rgbe, in_Pixels[0], in_Pixels[0], in_Pixels[0]);
						buffer[i] = rgbe[0];
						buffer[i+in_LineWidth] = rgbe[1];
						buffer[i+2*in_LineWidth] = rgbe[2];
						buffer[i+3*in_LineWidth] = rgbe[3];
						in_Pixels += 1;
					}
					/* write out each of the four channels separately run length encoded */
					/* first red, then green, then blue, then exponent */
					for(i=0; i<4; i++)
					{
						if(hdrWriteBytesRLE(in_F, &buffer[i*in_LineWidth], in_LineWidth) != 0)
						{
							free(buffer);
							return BR_FAIL;
						}
					}
				}
				free(buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrWritePixelsRLE<T, 2>
		{
			BBRESULT operator()(FILE* in_F, T* in_Pixels, int32_t in_LineWidth, int32_t in_NumLines)
			{
				unsigned char rgbe[4];
				unsigned char *buffer;
				int32_t i;

				if((in_LineWidth < 8)||(in_LineWidth > 0x7fff))
					/* run length encoding is not allowed so write flat*/
					return hdrWritePixels<T, 2>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				buffer = (unsigned char*)malloc(sizeof(unsigned char) * 4 * in_LineWidth);
				if(buffer == NULL)
					/* no buffer space so write flat */
					return hdrWritePixels<T, 2>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				while(in_NumLines-- > 0)
				{
					rgbe[0] = 2;
					rgbe[1] = 2;
					rgbe[2] = in_LineWidth >> 8;
					rgbe[3] = in_LineWidth & 0xFF;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(buffer);
						return BR_FAIL;
					}
					for(i=0; i<in_LineWidth; i++)
					{
						float2rgbe(rgbe, in_Pixels[0], in_Pixels[1], 0.0);
						buffer[i] = rgbe[0];
						buffer[i+in_LineWidth] = rgbe[1];
						buffer[i+2*in_LineWidth] = rgbe[2];
						buffer[i+3*in_LineWidth] = rgbe[3];
						in_Pixels += 2;
					}
					/* write out each of the four channels separately run length encoded */
					/* first red, then green, then blue, then exponent */
					for(i=0; i<4; i++)
					{
						if(hdrWriteBytesRLE(in_F, &buffer[i*in_LineWidth], in_LineWidth) != 0)
						{
							free(buffer);
							return BR_FAIL;
						}
					}
				}
				free(buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrWritePixelsRLE<T, 3>
		{
			BBRESULT operator()(FILE* in_F, T* in_Pixels, int32_t in_LineWidth, int32_t in_NumLines)
			{
				unsigned char rgbe[4];
				unsigned char *buffer;
				int32_t i;

				if((in_LineWidth < 8)||(in_LineWidth > 0x7fff))
					/* run length encoding is not allowed so write flat*/
					return hdrWritePixels<T, 3>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				buffer = (unsigned char*)malloc(sizeof(unsigned char) * 4 * in_LineWidth);
				if(buffer == NULL)
					/* no buffer space so write flat */
					return hdrWritePixels<T, 3>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				while(in_NumLines-- > 0)
				{
					rgbe[0] = 2;
					rgbe[1] = 2;
					rgbe[2] = in_LineWidth >> 8;
					rgbe[3] = in_LineWidth & 0xFF;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(buffer);
						return BR_FAIL;
					}
					for(i=0; i<in_LineWidth; i++)
					{
						float2rgbe(rgbe, in_Pixels[0], in_Pixels[1], in_Pixels[2]);
						buffer[i] = rgbe[0];
						buffer[i+in_LineWidth] = rgbe[1];
						buffer[i+2*in_LineWidth] = rgbe[2];
						buffer[i+3*in_LineWidth] = rgbe[3];
						in_Pixels += 3;
					}
					/* write out each of the four channels separately run length encoded */
					/* first red, then green, then blue, then exponent */
					for(i=0; i<4; i++)
					{
						if(hdrWriteBytesRLE(in_F, &buffer[i*in_LineWidth], in_LineWidth) != 0)
						{
							free(buffer);
							return BR_FAIL;
						}
					}
				}
				free(buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrWritePixelsRLE<T, 4>
		{
			BBRESULT operator()(FILE* in_F, T* in_Pixels, int32_t in_LineWidth, int32_t in_NumLines)
			{
				unsigned char rgbe[4];
				unsigned char *buffer;
				int32_t i;

				if((in_LineWidth < 8)||(in_LineWidth > 0x7fff))
					/* run length encoding is not allowed so write flat*/
					return hdrWritePixels<T, 4>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				buffer = (unsigned char*)malloc(sizeof(unsigned char) * 4 * in_LineWidth);
				if(buffer == NULL)
					/* no buffer space so write flat */
					return hdrWritePixels<T, 4>()(in_F, in_Pixels, in_LineWidth*in_NumLines);
				while(in_NumLines-- > 0)
				{
					rgbe[0] = 2;
					rgbe[1] = 2;
					rgbe[2] = in_LineWidth >> 8;
					rgbe[3] = in_LineWidth & 0xFF;
					if(fwrite(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(buffer);
						return BR_FAIL;
					}
					for(i=0; i<in_LineWidth; i++)
					{
						float2rgbe(rgbe, in_Pixels[0], in_Pixels[1], in_Pixels[2]);
						buffer[i] = rgbe[0];
						buffer[i+in_LineWidth] = rgbe[1];
						buffer[i+2*in_LineWidth] = rgbe[2];
						buffer[i+3*in_LineWidth] = rgbe[3];
						in_Pixels += 4;
					}
					/* write out each of the four channels separately run length encoded */
					/* first red, then green, then blue, then exponent */
					for(i=0; i<4; i++)
					{
						if(hdrWriteBytesRLE(in_F, &buffer[i*in_LineWidth], in_LineWidth) != 0)
						{
							free(buffer);
							return BR_FAIL;
						}
					}
				}
				free(buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T>
		BBRESULT hdrReadHeader(FILE* in_F, int32_t* out_Width, int32_t* out_Height, T* out_Gamma, T* out_Exposure)
		{
			char buf[128];
			float tempf;

			*out_Gamma = 0;
			*out_Exposure = 0;

			if(fgets(buf, sizeof(buf)/sizeof(buf[0]), in_F) == NULL)
				return BR_FAIL;
			if((buf[0] != '#')||(buf[1] != '?'))
			{
				/* if you want to require the magic token then uncomment the next line */
				/*return rgbe_error(rgbe_format_error,"bad initial token"); */
			}
			for(;;)
			{
				if(buf[0] == 0)
					return BR_FAIL;
				else if(buf[0] == '\n')
					break;
				else if (strcmp(buf, "FORMAT=32-bit_rle_rgbe\n") == 0)
					;       /* format found so break out of loop */
#ifdef _MSC_VER
				else if (sscanf_s(buf, "GAMMA=%g", &tempf) == 1)
#else
				else if (sscanf(buf, "GAMMA=%g", &tempf) == 1)
#endif
					*out_Gamma = tempf;
#ifdef _MSC_VER
				else if (sscanf_s(buf, "EXPOSURE=%g", &tempf) == 1)
#else
				else if (sscanf(buf, "EXPOSURE=%g", &tempf) == 1)
#endif
					*out_Exposure = tempf;
				if (fgets(buf, sizeof(buf)/sizeof(buf[0]), in_F) == 0)
					return BR_FAIL;
			}
			if(fgets(buf, sizeof(buf)/sizeof(buf[0]), in_F) == 0)
				return BR_FAIL;
#ifdef _MSC_VER
			if (sscanf_s(buf, "-Y %d +X %d", out_Height, out_Width) < 2)
#else
			if (sscanf(buf, "-Y %d +X %d", out_Height, out_Width) < 2)
#endif
				return BR_FAIL;
			return BR_SUCCESS;
		}

		template<typename T, int N>
		struct hdrReadPixels
		{
			BBRESULT operator()(FILE* in_F, T* out_Data, int32_t in_NumPixels) { return BR_SUCCESS; }
		};

		template<typename T>
		struct hdrReadPixels<T, 1>
		{
			BBRESULT operator()(FILE* in_F, T* out_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];
				T data[3];

				while(in_NumPixels-- > 0) {
					if (fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
					rgbe2float<T>(&data[0], &data[1], &data[2], rgbe);
					*out_Data = (data[0] + data[1] + data[2]) / 3.0;
					out_Data++;
				}
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrReadPixels<T, 2>
		{
			BBRESULT operator()(FILE* in_F, T* out_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];
				T data[3];

				while(in_NumPixels-- > 0) {
					if (fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
					rgbe2float<T>(&data[0], &data[1], &data[2], rgbe);
					out_Data[0] = data[0]; out_Data[1] = data[1];
					out_Data += 2;
				}
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrReadPixels<T, 3>
		{
			BBRESULT operator()(FILE* in_F, T* out_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];

				while(in_NumPixels-- > 0) {
					if (fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
					rgbe2float<T>(&out_Data[0], &out_Data[1], &out_Data[2], rgbe);
					out_Data += 3;
				}
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrReadPixels<T, 4>
		{
			BBRESULT operator()(FILE* in_F, T* out_Data, int32_t in_NumPixels)
			{
				unsigned char rgbe[4];

				while(in_NumPixels-- > 0) {
					if (fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
						return BR_FAIL;
					rgbe2float<T>(&out_Data[0], &out_Data[1], &out_Data[2], rgbe);
					out_Data[3] = 0.0;
					out_Data += 4;
				}
				return BR_SUCCESS;
			}
		};

		template<typename T, int N>
		struct hdrReadPixelsRLE
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_ScanlineWidth, int32_t in_NumScanlines) { return BR_SUCCESS; }
		};

		template<typename T>
		struct hdrReadPixelsRLE<T, 1>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_ScanlineWidth, int32_t in_NumScanlines)
			{
				unsigned char rgbe[4], *scanline_buffer, *ptr, *ptr_end;
				int32_t i, count;
				unsigned char buf[2];

				if((in_ScanlineWidth < 8)||(in_ScanlineWidth > 0x7fff))
					/* run length encoding is not allowed so read flat*/
					return hdrReadPixels<T, 1>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines);
				scanline_buffer = NULL;
				/* read in each successive scanline */
				while(in_NumScanlines > 0)
				{
					if(fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(scanline_buffer);
						return BR_FAIL;
					}
					if((rgbe[0] != 2)||(rgbe[1] != 2)||(rgbe[2] & 0x80)) {
						/* this file is not run length encoded */
						T data[3];
						rgbe2float<T>(&data[0], &data[1], &data[2], rgbe);
						*in_Data = (data[0] + data[1] + data[2]) / 3.0;
						in_Data++;
						free(scanline_buffer);
						return hdrReadPixels<T, 1>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines-1);
					}
					if((((int)rgbe[2])<<8 | rgbe[3]) != in_ScanlineWidth) {
						free(scanline_buffer);
						return BR_FAIL;
					}
					if(scanline_buffer == NULL)
						scanline_buffer = (unsigned char *)malloc(sizeof(unsigned char)*4*in_ScanlineWidth);
					if(scanline_buffer == NULL) 
						return BR_FAIL;

					ptr = &scanline_buffer[0];
					/* read each of the four channels for the scanline into the buffer */
					for(i=0; i<4; i++)
					{
						ptr_end = &scanline_buffer[(i+1)*in_ScanlineWidth];
						while(ptr < ptr_end)
						{
							if(fread(buf, sizeof(buf[0]) * 2, 1, in_F) < 1)
							{
								free(scanline_buffer);
								return BR_FAIL;
							}
							if(buf[0] > 128)
							{
								/* a run of the same value */
								count = buf[0]-128;
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								while(count-- > 0)
									*ptr++ = buf[1];
							}
							else
							{
								/* a non-run */
								count = buf[0];
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								*ptr++ = buf[1];
								if(--count > 0)
								{
									if(fread(ptr, sizeof(*ptr)*count, 1, in_F) < 1)
									{
										free(scanline_buffer);
										return BR_FAIL;
									}
									ptr += count;
								}
							}
						}
					}
					/* now convert data from buffer into floats */
					for(i=0; i<in_ScanlineWidth; i++)
					{
						rgbe[0] = scanline_buffer[i];
						rgbe[1] = scanline_buffer[i+in_ScanlineWidth];
						rgbe[2] = scanline_buffer[i+2*in_ScanlineWidth];
						rgbe[3] = scanline_buffer[i+3*in_ScanlineWidth];
						T data[3];
						rgbe2float<T>(&data[0], &data[1], &data[2], rgbe);
						*in_Data = (data[0] + data[1] + data[2]) / 3.0;
						in_Data++;
					}
					in_NumScanlines--;
				}
				free(scanline_buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrReadPixelsRLE<T, 2>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_ScanlineWidth, int32_t in_NumScanlines)
			{
				unsigned char rgbe[4], *scanline_buffer, *ptr, *ptr_end;
				int32_t i, count;
				unsigned char buf[2];

				if((in_ScanlineWidth < 8)||(in_ScanlineWidth > 0x7fff))
					/* run length encoding is not allowed so read flat*/
					return hdrReadPixels<T, 2>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines);
				scanline_buffer = NULL;
				/* read in each successive scanline */
				while(in_NumScanlines > 0)
				{
					if(fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(scanline_buffer);
						return BR_FAIL;
					}
					if((rgbe[0] != 2)||(rgbe[1] != 2)||(rgbe[2] & 0x80)) {
						/* this file is not run length encoded */
						T data[3];
						rgbe2float<T>(&data[0], &data[1], &data[2], rgbe);
						in_Data[0] = data[0]; in_Data[1] = data[1];
						in_Data += 2;
						free(scanline_buffer);
						return hdrReadPixels<T, 2>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines-1);
					}
					if((((int)rgbe[2])<<8 | rgbe[3]) != in_ScanlineWidth) {
						free(scanline_buffer);
						return BR_FAIL;
					}
					if(scanline_buffer == NULL)
						scanline_buffer = (unsigned char *)malloc(sizeof(unsigned char)*4*in_ScanlineWidth);
					if(scanline_buffer == NULL) 
						return BR_FAIL;

					ptr = &scanline_buffer[0];
					/* read each of the four channels for the scanline into the buffer */
					for(i=0; i<4; i++)
					{
						ptr_end = &scanline_buffer[(i+1)*in_ScanlineWidth];
						while(ptr < ptr_end)
						{
							if(fread(buf, sizeof(buf[0]) * 2, 1, in_F) < 1)
							{
								free(scanline_buffer);
								return BR_FAIL;
							}
							if(buf[0] > 128)
							{
								/* a run of the same value */
								count = buf[0]-128;
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								while(count-- > 0)
									*ptr++ = buf[1];
							}
							else
							{
								/* a non-run */
								count = buf[0];
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								*ptr++ = buf[1];
								if(--count > 0)
								{
									if(fread(ptr, sizeof(*ptr)*count, 1, in_F) < 1)
									{
										free(scanline_buffer);
										return BR_FAIL;
									}
									ptr += count;
								}
							}
						}
					}
					/* now convert data from buffer into floats */
					for(i=0; i<in_ScanlineWidth; i++)
					{
						rgbe[0] = scanline_buffer[i];
						rgbe[1] = scanline_buffer[i+in_ScanlineWidth];
						rgbe[2] = scanline_buffer[i+2*in_ScanlineWidth];
						rgbe[3] = scanline_buffer[i+3*in_ScanlineWidth];
						T data[3];
						rgbe2float<T>(&data[0], &data[1], &data[2], rgbe);
						in_Data[0] = data[0]; in_Data[1] = data[1];
						in_Data += 2;
					}
					in_NumScanlines--;
				}
				free(scanline_buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrReadPixelsRLE<T, 3>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_ScanlineWidth, int32_t in_NumScanlines)
			{
				unsigned char rgbe[4], *scanline_buffer, *ptr, *ptr_end;
				int32_t i, count;
				unsigned char buf[2];

				if((in_ScanlineWidth < 8)||(in_ScanlineWidth > 0x7fff))
					/* run length encoding is not allowed so read flat*/
					return hdrReadPixels<T, 3>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines);
				scanline_buffer = NULL;
				/* read in each successive scanline */
				while(in_NumScanlines > 0)
				{
					if(fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(scanline_buffer);
						return BR_FAIL;
					}
					if((rgbe[0] != 2)||(rgbe[1] != 2)||(rgbe[2] & 0x80)) {
						/* this file is not run length encoded */
						rgbe2float<T>(&in_Data[0], &in_Data[1], &in_Data[2], rgbe);
						in_Data += 3;
						free(scanline_buffer);
						return hdrReadPixels<T, 3>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines-1);
					}
					if((((int)rgbe[2])<<8 | rgbe[3]) != in_ScanlineWidth) {
						free(scanline_buffer);
						return BR_FAIL;
					}
					if(scanline_buffer == NULL)
						scanline_buffer = (unsigned char *)malloc(sizeof(unsigned char)*4*in_ScanlineWidth);
					if(scanline_buffer == NULL) 
						return BR_FAIL;

					ptr = &scanline_buffer[0];
					/* read each of the four channels for the scanline into the buffer */
					for(i=0; i<4; i++)
					{
						ptr_end = &scanline_buffer[(i+1)*in_ScanlineWidth];
						while(ptr < ptr_end)
						{
							if(fread(buf, sizeof(buf[0]) * 2, 1, in_F) < 1)
							{
								free(scanline_buffer);
								return BR_FAIL;
							}
							if(buf[0] > 128)
							{
								/* a run of the same value */
								count = buf[0]-128;
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								while(count-- > 0)
									*ptr++ = buf[1];
							}
							else
							{
								/* a non-run */
								count = buf[0];
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								*ptr++ = buf[1];
								if(--count > 0)
								{
									if(fread(ptr, sizeof(*ptr)*count, 1, in_F) < 1)
									{
										free(scanline_buffer);
										return BR_FAIL;
									}
									ptr += count;
								}
							}
						}
					}
					/* now convert data from buffer into floats */
					for(i=0; i<in_ScanlineWidth; i++)
					{
						rgbe[0] = scanline_buffer[i];
						rgbe[1] = scanline_buffer[i+in_ScanlineWidth];
						rgbe[2] = scanline_buffer[i+2*in_ScanlineWidth];
						rgbe[3] = scanline_buffer[i+3*in_ScanlineWidth];
						rgbe2float<T>(&in_Data[0], &in_Data[1], &in_Data[2], rgbe);
						in_Data += 3;
					}
					in_NumScanlines--;
				}
				free(scanline_buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T>
		struct hdrReadPixelsRLE<T, 4>
		{
			BBRESULT operator()(FILE* in_F, T* in_Data, int32_t in_ScanlineWidth, int32_t in_NumScanlines)
			{
				unsigned char rgbe[4], *scanline_buffer, *ptr, *ptr_end;
				int32_t i, count;
				unsigned char buf[2];

				if((in_ScanlineWidth < 8)||(in_ScanlineWidth > 0x7fff))
					/* run length encoding is not allowed so read flat*/
					return hdrReadPixels<T, 4>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines);
				scanline_buffer = NULL;
				/* read in each successive scanline */
				while(in_NumScanlines > 0)
				{
					if(fread(rgbe, sizeof(rgbe), 1, in_F) < 1)
					{
						free(scanline_buffer);
						return BR_FAIL;
					}
					if((rgbe[0] != 2)||(rgbe[1] != 2)||(rgbe[2] & 0x80)) {
						/* this file is not run length encoded */
						rgbe2float<T>(&in_Data[0], &in_Data[1], &in_Data[2], rgbe);
						in_Data[3] = 0.0;
						in_Data += 4;
						free(scanline_buffer);
						return hdrReadPixels<T, 4>()(in_F, in_Data, in_ScanlineWidth * in_NumScanlines-1);
					}
					if((((int)rgbe[2])<<8 | rgbe[3]) != in_ScanlineWidth) {
						free(scanline_buffer);
						return BR_FAIL;
					}
					if(scanline_buffer == NULL)
						scanline_buffer = (unsigned char *)malloc(sizeof(unsigned char)*4*in_ScanlineWidth);
					if(scanline_buffer == NULL) 
						return BR_FAIL;

					ptr = &scanline_buffer[0];
					/* read each of the four channels for the scanline into the buffer */
					for(i=0; i<4; i++)
					{
						ptr_end = &scanline_buffer[(i+1)*in_ScanlineWidth];
						while(ptr < ptr_end)
						{
							if(fread(buf, sizeof(buf[0]) * 2, 1, in_F) < 1)
							{
								free(scanline_buffer);
								return BR_FAIL;
							}
							if(buf[0] > 128)
							{
								/* a run of the same value */
								count = buf[0]-128;
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								while(count-- > 0)
									*ptr++ = buf[1];
							}
							else
							{
								/* a non-run */
								count = buf[0];
								if((count == 0)||(count > ptr_end - ptr))
								{
									free(scanline_buffer);
									return BR_FAIL;
								}
								*ptr++ = buf[1];
								if(--count > 0)
								{
									if(fread(ptr, sizeof(*ptr)*count, 1, in_F) < 1)
									{
										free(scanline_buffer);
										return BR_FAIL;
									}
									ptr += count;
								}
							}
						}
					}
					/* now convert data from buffer into floats */
					for(i=0; i<in_ScanlineWidth; i++)
					{
						rgbe[0] = scanline_buffer[i];
						rgbe[1] = scanline_buffer[i+in_ScanlineWidth];
						rgbe[2] = scanline_buffer[i+2*in_ScanlineWidth];
						rgbe[3] = scanline_buffer[i+3*in_ScanlineWidth];
						rgbe2float<T>(&in_Data[0], &in_Data[1], &in_Data[2], rgbe);
						in_Data[3] = 0.0;
						in_Data += 4;
					}
					in_NumScanlines--;
				}
				free(scanline_buffer);
				return BR_SUCCESS;
			}
		};

		template<typename T, int N>
		inline BBRESULT loadImageHdr(const char* in_Filename, SImage<T, N>& io_Image)
		{
			FILE* f;
#ifdef _MSC_VER
			const errno_t err = fopen_s(&f, in_Filename, "rb");
			if(err != 0) return BR_FAIL;
#else
			f = fopen(in_Filename, "rb");
			if(f == NULL) return BR_FAIL;
#endif
			T gamma, exposure;
			if(0!=hdrReadHeader<T>(f, &io_Image.width, &io_Image.height, &gamma, &exposure))
			{
				fclose(f);
				return BR_FAIL;
			}
			if(io_Image.ptr != NULL)
			{
				free(io_Image.ptr);
				io_Image.ptr = NULL;
			}
			io_Image.ptr = (T*)malloc(sizeof(T)*io_Image.width*io_Image.height*N);
			if(0!=hdrReadPixelsRLE<T, N>()(f, io_Image.ptr, io_Image.width, io_Image.height))
			{
				fclose(f);
				return BR_FAIL;
			}
			else
			{
				fclose(f);
				return BR_SUCCESS;
			}
		}

		template<typename T, int N>
		inline BBRESULT saveImageHdr(const char* in_Filename, const SImage<T, N>& in_Image)
		{
			FILE* f;
#ifdef _MSC_VER
			const errno_t err = fopen_s(&f, in_Filename, "wb");
			if(err != 0) return BR_FAIL;
#else
			f = fopen(in_Filename, "wb");
			if(f == NULL) return BR_FAIL;
#endif
			hdrWriteHeader<T>(f, in_Image.width, in_Image.height, T(1.0), T(1.0));
			//hdrWritePixels(f, in_PixelColors3, in_Width*in_Height);
			hdrWritePixelsRLE<T, N>()(f, in_Image.ptr, in_Image.width, in_Image.height);
			fclose(f);
			return BR_SUCCESS;
		}
	};
};

#endif
