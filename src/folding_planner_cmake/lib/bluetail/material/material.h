#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include "../image/image.h"
using namespace _BlueTail::_Image;

namespace _BlueTail
{
	namespace _Material
	{
		enum MATERIAL_TYPE
		{
			MT_SINGLESIDED,
			MT_DOUBLESIDED
		};

		template<typename T> struct SMaterial
		{
			MATERIAL_TYPE type;
			T color[3];
			T diffuse;
			T specularity;
			T glossiness;
			T luminosity;
			T reflection;
			T transparency;
			T refraction_index;
			T smoothing_threshold;
			char* material_name;
			SImage<T, 3> color_tex;
			SImage<T, 3> normal_tex;
			SImage<T, 3> diff_tex;
			SImage<T, 3> transparent_tex;
		};
	};
};

#endif
