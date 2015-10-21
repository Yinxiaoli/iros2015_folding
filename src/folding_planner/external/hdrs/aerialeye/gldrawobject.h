#ifndef __GL_DRAW_OBJECT_H__
#define __GL_DRAW_OBJECT_H__

#include <blackbill/delegator/property.h>
#include <blackbill/delegator/eventhandler2.h>

enum EALIGN
{
	A_DEFAULT,
	A_LEFT,
	A_TOP,
	A_RIGHT,
	A_BOTTOM,
	A_ALL
};

class CGLDrawObject
{
public:
	CGLDrawObject();
	virtual ~CGLDrawObject();

	event<> OnMouseIn;
	event<> OnMouseOut;
	event<int, int> OnMouseOver;
	event<int, int> OnMouseDrag;
	event<int, int> OnMouseDown;
	event<int, int> OnMouseUp;
	event<int, int> OnClick;
	event<> OnDraw;

	property<int> Left;
	property<int> Top;
	property<int> Width;
	property<int> Height;
	property<EALIGN> Align;

	void __setOrigin(int in_OX, int in_OY);
	void __getOrigin(int* io_OX, int* io_OY);
	virtual CGLDrawObject* __inRegion(int in_GlobalX, int in_GlobalY);

	void __mouseOver(int in_GlobalX, int in_GlobalY);
	void __mouseDrag(int in_GlobalX, int in_GlobalY);
	void __mouseDown(int in_GlobalX, int in_GlobalY);
	void __mouseUp(int in_GlobalX, int in_GlobalY);
	void __click(int in_GlobalX, int in_GlobalY);

protected:
	int m_Left;
	int m_Top;
	int m_Width;
	int m_Height;
	int m_OriginX;
	int m_OriginY;
	EALIGN m_Align;

	int getLeft();
	void setLeft(int in_Left);

	int getTop();
	void setTop(int in_Top);

	int getWidth();
	void setWidth(int in_Width);

	int getHeight();
	void setHeight(int in_Height);

	EALIGN getAlign();
	void setAlign(EALIGN in_Align);
};

#endif
