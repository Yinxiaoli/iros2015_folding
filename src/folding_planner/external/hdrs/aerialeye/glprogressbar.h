#ifndef __GL_PROGRESS_BAR_H__
#define __GL_PROGRESS_BAR_H__

#include "gldrawobject.h"

class CGLProgressBar : public CGLDrawObject
{
public:
	CGLProgressBar();
	~CGLProgressBar();

	property<double> Progress;

protected:
	double m_Progress;

	double getProgress();
	void setProgress(double in_Progress);
	void draw();
};

#endif
