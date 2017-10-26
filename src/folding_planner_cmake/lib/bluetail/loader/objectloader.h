#ifndef __OBJECT_LOADER_H__
#define __OBJECT_LOADER_H__

#include "../objectmodel/triangleset.h"
using namespace _BlueTail::_ObjectModel;

#include <blackbill/filepath/filepath.h>
using namespace _BlackBill::_FilePath;

#include "lwoloader.h"

namespace _BlueTail{
	namespace _Loader{

		template<typename T>
		bool loadObjects(const char* in_currentdir, const char* in_filename, STriangleSet<T>* io_Objects)
		{
			string the_FileExt = getFileExt(in_filename);
			if(the_FileExt == "LWO")
			{
				printf("Loading LWO File...\n");
				//loadLwo(in_currentdir, in_filename, io_Scene, NULL);
				loadLwo(in_currentdir, in_filename, io_Objects);
				return true;
			}

			return false;
		}
	};
};

#endif
