#include "glprogressbar.h"
#include "glframework.h"

#include <GL/freeglut.h>

CGLProgressBar::CGLProgressBar() : CGLDrawObject()
{
	Progress.read = bind1st(mem_func(&CGLProgressBar::getProgress), this);
	Progress.write = bind1st(mem_func(&CGLProgressBar::setProgress), this);

	OnDraw += bind1st(mem_func(&CGLProgressBar::draw), this);
}

CGLProgressBar::~CGLProgressBar()
{

}

double CGLProgressBar::getProgress() { return m_Progress; }
void CGLProgressBar::setProgress(double in_Progress) { m_Progress = in_Progress; g_GLFramework->redisplayQuest(); }

void CGLProgressBar::draw()
{
	glBegin(GL_QUADS);

	glColor3f(1.0, 1.0, 0.0);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top);
	glVertex2f(m_OriginX+m_Left+m_Width*m_Progress/100.0, m_OriginY+m_Top);
	glVertex2f(m_OriginX+m_Left+m_Width*m_Progress/100.0, m_OriginY+m_Top+m_Height);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top+m_Height);

	glEnd();

	glBegin(GL_LINE_STRIP);

	glColor3f(0.0, 0.0, 0.0);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top);
	glVertex2f(m_OriginX+m_Left+m_Width, m_OriginY+m_Top);
	glVertex2f(m_OriginX+m_Left+m_Width, m_OriginY+m_Top+m_Height);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top+m_Height);
	glVertex2f(m_OriginX+m_Left, m_OriginY+m_Top);

	glEnd();
}
