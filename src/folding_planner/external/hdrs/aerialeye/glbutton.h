#ifndef __GL_BUTTON_H__
#define __GL_BUTTON_H__

#include "gldrawobject.h"

class CGLButton : public CGLDrawObject
{
public:
	CGLButton();
	~CGLButton();

protected:
	void draw();

	int m_State;

	void mouseIn();
	void mouseOut();
	void mouseOver(int x, int y);
	void mouseDrag(int x, int y);
	void mouseDown(int x, int y);
	void mouseUp(int x, int y);
};

#endif
