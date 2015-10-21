#ifndef __ENDIAN_UTIL_H__
#define __ENDIAN_UTIL_H__

inline unsigned short _SwapTwoBytes (unsigned short w)
{
	unsigned short tmp;
	tmp =  (w & 0x00ff);
	tmp = ((w & 0xff00) >> 0x08) | (tmp << 0x08);
	return tmp;
}

inline unsigned long _SwapFourBytes (unsigned long w)
{
	unsigned long tmp;
	tmp =  (w & 0x000000ff);
	tmp = ((w & 0x0000ff00) >> 0x08) | (tmp << 0x08);
	tmp = ((w & 0x00ff0000) >> 0x10) | (tmp << 0x08);
	tmp = ((w & 0xff000000) >> 0x18) | (tmp << 0x08);
	return tmp;
}

#define _LITTLE_ENDIAN

#ifdef _LITTLE_ENDIAN
#define MSB2			_SwapTwoBytes
#define MSB4			_SwapFourBytes 
#define LSB2(w)			(w)
#define LSB4(w)			(w)
#else
#define MSB2(w)			(w)
#define MSB4(w)			(w)
#define LSB2			_SwapTwoBytes
#define LSB4			_SwapFourBytes 
#endif

#endif
