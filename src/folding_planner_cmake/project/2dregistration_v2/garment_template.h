#ifndef __GARMENT_TEMPLATE_H__
#define __GARMENT_TEMPLATE_H__

#include "datatypes.h"
#include "ImageUtil.h"

void initGarmentTemplate(SCurve* io_Curve, SVar& io_Vars, GarmentType type);

void initSweaterTemplate(SCurve* io_Curve, SVar& io_Vars);
void initPantsTemplate(SCurve* io_Curve, SVar& io_Vars);
void initTowelTemplate(SCurve* io_Curve, SVar& io_Vars);

#endif
