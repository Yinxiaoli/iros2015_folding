#include "glbutton.h"
#include "glframework.h"

#include <GL/freeglut.h>

enum EBUTTONSTATE
{
	BS_DEFAULT,
	BS_FOCUS,
	BS_DOWN
};

CGLButton::CGLButton() : CGLDrawObject()
{
	OnDraw += bind1st(mem_func(&CGLButton::draw), this);

	OnMouseOver += bind1st(mem_func(&CGLButton::mouseOver), this);
	OnMouseDrag += bind1st(mem_func(&CGLButton::mouseDrag), this);
	OnMouseIn += bind1st(mem_func(&CGLButton::mouseIn), this);
	OnMouseOut += bind1st(mem_func(&CGLButton::mouseOut), this);
	OnMouseDown += bind1st(mem_func(&CGLButton::mouseDown), this);
	OnMouseUp += bind1st(mem_func(&CGLButton::mouseUp), this);

	m_State = BS_DEFAULT;
}

CGLButton::~CGLButton()
{

}

void CGLButton::draw()
{
	float R, G, B;
	if(m_State == BS_FOCUS)
	{
		R = 1.0; G = 1.0; B = 0.7;
	}
	else if(m_State == BS_DOWN)
	{
		R = 0.95; G = 0.95; B = 0.6;
	}

	if((m_State == BS_FOCUS) || (m_State == BS_DOWN))
	{
		glBegin(GL_QUADS);

		glColor3f(R, G, B);
		glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top);
		glVertex2f(m_OriginX+m_Left+m_Width, m_OriginY+m_Top);
		glVertex2f(m_OriginX+m_Left+m_Width, m_OriginY+m_Top+m_Height);
		glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top+m_Height);
	
		glEnd();
	}

	if(m_State == BS_DOWN)
	{
		glBegin(GL_LINES);

		glColor3f(0.8, 0.7, 0.35);
		glVertex2f(m_OriginX+m_Left+1, m_OriginY+m_Top+1);
		glVertex2f(m_OriginX+m_Left+m_Width-1, m_OriginY+m_Top+1);
		glVertex2f(m_OriginX+m_Left+1, m_OriginY+m_Top+1);
		glVertex2f(m_OriginX+m_Left+1, m_OriginY+m_Top+m_Height-1);

		glColor3f(1.0, 0.95, 0.8);
		glVertex2f(m_OriginX+m_Left+m_Width-1, m_OriginY+m_Top+m_Height-1);
		glVertex2f(m_OriginX+m_Left+1, m_OriginY+m_Top+m_Height-1);
		glVertex2f(m_OriginX+m_Left+m_Width-1, m_OriginY+m_Top+m_Height-1);
		glVertex2f(m_OriginX+m_Left+m_Width-1, m_OriginY+m_Top+1);
		glEnd();
	}

	glBegin(GL_LINE_STRIP);

	glColor3f(0.0, 0.0, 0.0);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top);
	glVertex2f(m_OriginX+m_Left+m_Width, m_OriginY+m_Top);
	glVertex2f(m_OriginX+m_Left+m_Width, m_OriginY+m_Top+m_Height);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top+m_Height);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top);

	glEnd();
}

void CGLButton::mouseIn()
{
	m_State = BS_FOCUS;
	g_GLFramework->redisplayQuest();
}

void CGLButton::mouseOut()
{
	m_State = BS_DEFAULT;
	g_GLFramework->redisplayQuest();
}

void CGLButton::mouseOver(int x, int y)
{
	if(m_State != BS_FOCUS)
	{
		m_State = BS_FOCUS;
		g_GLFramework->redisplayQuest();
	}
}

void CGLButton::mouseDrag(int x, int y)
{
	if(m_State != BS_DOWN)
	{
		m_State = BS_DOWN;
		g_GLFramework->redisplayQuest();
	}
}

void CGLButton::mouseDown(int x, int y)
{
	if(m_State != BS_DOWN)
	{
		m_State = BS_DOWN;
		g_GLFramework->redisplayQuest();
	}
}

void CGLButton::mouseUp(int x, int y)
{
	if(m_State != BS_FOCUS)
	{
		m_State = BS_FOCUS;
		g_GLFramework->redisplayQuest();
	}
}
