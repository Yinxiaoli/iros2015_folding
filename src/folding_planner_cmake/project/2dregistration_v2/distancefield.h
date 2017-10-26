#ifndef __DISTANCE_FIELD_H__
#define __DISTANCE_FIELD_H__

#include <bluetail/image/image.h>
using namespace _BlueTail::_Image;

void generateSDF(const SImage<double, 1>& in_BinaryImage, SImage<double, 1>& out_SDF);
void generateDF(const SImage<double, 1>& in_BinaryImage, SImage<double, 1>& out_DF);

#endif
