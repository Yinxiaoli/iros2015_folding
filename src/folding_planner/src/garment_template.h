// Pre-defined garment template as the initialization.
// The binary garment mask has to be within the template contour.

#ifndef __GARMENT_TEMPLATE_H__
#define __GARMENT_TEMPLATE_H__

#include "datatypes.h"
#include "ImageUtil.h"

void initGarmentTemplate(SCurve* io_curve, SVar& io_vars, GarmentType type);

void initSweaterTemplate(SCurve* io_curve, SVar& io_vars);
void initPantsTemplate(SCurve* io_curve, SVar& io_vars);
void initTowelTemplate(SCurve* io_curve, SVar& io_vars);

#endif
