#include "gldrawobject.h"
#include <GL/freeglut.h>

CGLDrawObject::CGLDrawObject()
	: m_OriginX(0), m_OriginY(0)
{
	Left.read = bind1st(mem_func(&CGLDrawObject::getLeft), this);
	Left.write = bind1st(mem_func(&CGLDrawObject::setLeft), this);
	Top.read = bind1st(mem_func(&CGLDrawObject::getTop), this);
	Top.write = bind1st(mem_func(&CGLDrawObject::setTop), this);
	Width.read = bind1st(mem_func(&CGLDrawObject::getWidth), this);
	Width.write = bind1st(mem_func(&CGLDrawObject::setWidth), this);
	Height.read = bind1st(mem_func(&CGLDrawObject::getHeight), this);
	Height.write = bind1st(mem_func(&CGLDrawObject::setHeight), this);
	Align.read = bind1st(mem_func(&CGLDrawObject::getAlign), this);
	Align.write = bind1st(mem_func(&CGLDrawObject::setAlign), this);

	m_Align = A_DEFAULT;
	m_Left = 0;
	m_Top = 0;
	m_Width = 256;
	m_Height = 16;
}

CGLDrawObject::~CGLDrawObject()
{

}

void CGLDrawObject::__setOrigin(int in_OX, int in_OY)
{
	m_OriginX = in_OX; m_OriginY = in_OY;
}

void CGLDrawObject::__getOrigin(int* io_OX, int* io_OY)
{
	*io_OX = m_OriginX; *io_OY = m_OriginY;
}

CGLDrawObject* CGLDrawObject::__inRegion(int in_GlobalX, int in_GlobalY)
{
	if((in_GlobalX >= m_OriginX + m_Left) && (in_GlobalX < m_OriginX + m_Left + m_Width)
		&& (in_GlobalY >= m_OriginY + m_Top) && (in_GlobalY < m_OriginY + m_Top + m_Height))
		return this;
	else
		return NULL;
}

void CGLDrawObject::__mouseOver(int in_GlobalX, int in_GlobalY)
{
	OnMouseOver(in_GlobalX - m_OriginX - m_Left, in_GlobalY - m_OriginY - m_Top);
}

void CGLDrawObject::__mouseDrag(int in_GlobalX, int in_GlobalY)
{
	OnMouseDrag(in_GlobalX - m_OriginX - m_Left, in_GlobalY - m_OriginY - m_Top);
}

void CGLDrawObject::__mouseDown(int in_GlobalX, int in_GlobalY)
{
	OnMouseDown(in_GlobalX - m_OriginX - m_Left, in_GlobalY - m_OriginY - m_Top);
}

void CGLDrawObject::__mouseUp(int in_GlobalX, int in_GlobalY)
{
	OnMouseUp(in_GlobalX - m_OriginX - m_Left, in_GlobalY - m_OriginY - m_Top);
}

void CGLDrawObject::__click(int in_GlobalX, int in_GlobalY)
{
	OnClick(in_GlobalX - m_OriginX - m_Left, in_GlobalY - m_OriginY - m_Top);
}

int CGLDrawObject::getLeft() { return m_Left; }
void CGLDrawObject::setLeft(int in_Left) { m_Left = in_Left; }

int CGLDrawObject::getTop() { return m_Top; }
void CGLDrawObject::setTop(int in_Top) { m_Top = in_Top; }

int CGLDrawObject::getWidth() { return m_Width; }
void CGLDrawObject::setWidth(int in_Width) { m_Width = in_Width; }

int CGLDrawObject::getHeight() { return m_Height; }
void CGLDrawObject::setHeight(int in_Height) { m_Height = in_Height; }

EALIGN CGLDrawObject::getAlign() { return m_Align; }
void CGLDrawObject::setAlign(EALIGN in_Align) { m_Align = in_Align; }
