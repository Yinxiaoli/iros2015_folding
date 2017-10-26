#ifndef __GL_PANEL_H__
#define __GL_PANEL_H__

#include "gldrawobject.h"
#include <vector>
using namespace std;

class CGLPanel : public CGLDrawObject
{
public:
	CGLPanel();
	~CGLPanel();

	void attachChild(CGLDrawObject* in_Child);

	CGLDrawObject* __inRegion(int in_GlobalX, int in_GlobalY);

protected:
	vector<CGLDrawObject*> m_Children;

	void drawChildren();
};

#endif
