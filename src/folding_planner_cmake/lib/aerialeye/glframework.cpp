#include "glframework.h"
#include "glpanel.h"

#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <unistd.h>

CGLFramework* CGLFramework::m_Instance = NULL;
CGLFramework* g_GLFramework = NULL;

CGLFramework::CGLFramework()
{
	MainPanel.read = bind1st(mem_func(&CGLFramework::getMainPanel), this);
	MainPanel.write = bind1st(mem_func(&CGLFramework::setMainPanel), this);
	Caption.read = bind1st(mem_func(&CGLFramework::getCaption), this);
	Caption.write = bind1st(mem_func(&CGLFramework::setCaption), this);

	m_MainPanel = new CGLPanel;
	m_MainPanel->Left = 0;
	m_MainPanel->Top = 0;
	m_MainPanel->Width = 256;
	m_MainPanel->Height = 256;

	m_Caption = "Default";

	m_RedisplayQuest = false;
	m_WindowCreated = false;
	m_MouseFocusObject = false;
}

CGLFramework::CGLFramework(const CGLFramework& in_i)
{

}

CGLFramework::~CGLFramework()
{

}

CGLFramework* CGLFramework::getInstance()
{
	if(m_Instance == NULL)
		m_Instance = new CGLFramework();

	return m_Instance;
}

void CGLFramework::redisplayQuest()
{
	m_RedisplayQuest = true;
}

bool CGLFramework::popRedisplayQuest()
{
	bool ret = m_RedisplayQuest;
	m_RedisplayQuest = false;
	return ret;
}

void CGLFramework::__passiveMouseMotion(int in_GlobalX, int in_GlobalY)
{
	CGLDrawObject* the_Obj = m_MainPanel->__inRegion(in_GlobalX, in_GlobalY);

	if(the_Obj != m_MouseFocusObject)
	{
		if(m_MouseFocusObject != NULL)
			m_MouseFocusObject->OnMouseOut();
		if(the_Obj != NULL)
			the_Obj->OnMouseIn();

		m_MouseFocusObject = the_Obj;
	}

	if(the_Obj != NULL)
		the_Obj->__mouseOver(in_GlobalX, in_GlobalY);
}

void CGLFramework::__mouseMotion(int in_GlobalX, int in_GlobalY)
{
	if(m_MouseFocusObject != NULL)
		m_MouseFocusObject->__mouseDrag(in_GlobalX, in_GlobalY);
}

void CGLFramework::__mouse(int button, int state, int in_GlobalX, int in_GlobalY)
{
	if(button == GLUT_LEFT_BUTTON)
	{
		if(state == GLUT_UP)
		{
			CGLDrawObject* the_Obj = m_MainPanel->__inRegion(in_GlobalX, in_GlobalY);
			if(the_Obj != m_MouseFocusObject)
			{
				if(m_MouseFocusObject != NULL)
				{
					m_MouseFocusObject->__mouseUp(in_GlobalX, in_GlobalY);
					m_MouseFocusObject->OnMouseOut();
				}
			}
			else
			{
				if(m_MouseFocusObject != NULL)
				{
					m_MouseFocusObject->__mouseUp(in_GlobalX, in_GlobalY);
					m_MouseFocusObject->__click(in_GlobalX, in_GlobalY);
				}
			}

			m_MouseFocusObject = NULL;
		}
		else if(state == GLUT_DOWN)
		{
			m_MouseFocusObject = m_MainPanel->__inRegion(in_GlobalX, in_GlobalY);
			if(m_MouseFocusObject != NULL)
				m_MouseFocusObject->__mouseDown(in_GlobalX, in_GlobalY);
		}
	}
}

void CGLFramework::__windowInitialized() { m_WindowCreated = true; }

CGLPanel* CGLFramework::getMainPanel() { return m_MainPanel; }
void CGLFramework::setMainPanel(const CGLPanel& in_Panel) { *m_MainPanel = in_Panel; }

string CGLFramework::getCaption() { return m_Caption; }
void CGLFramework::setCaption(const string& in_Caption) { m_Caption = in_Caption; if(m_WindowCreated) glutSetWindowTitle(m_Caption.c_str()); }

void __display()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	::glMatrixMode(GL_PROJECTION);
	::glLoadIdentity();
	::gluOrtho2D(0.0, g_GLFramework->MainPanel->Width, g_GLFramework->MainPanel->Height, 0.0);
	::glMatrixMode(GL_MODELVIEW);
	::glLoadIdentity();
	glViewport(0, 0, g_GLFramework->MainPanel->Width, g_GLFramework->MainPanel->Height);

	g_GLFramework->MainPanel->OnDraw();

	glutSwapBuffers();
}

void __resize(int w, int h)
{
	g_GLFramework->MainPanel->Width = w;
	g_GLFramework->MainPanel->Height = h;

	g_GLFramework->redisplayQuest();
}

void __mouse(int button, int state, int x, int y)
{
	g_GLFramework->__mouse(button, state, x, y);
}

void __motion(int x, int y)
{
	g_GLFramework->__mouseMotion(x, y);
}

void __passiveMotion(int x, int y)
{
	g_GLFramework->__passiveMouseMotion(x, y);
}

void __keybord(unsigned char key, int x, int y)
{
	g_GLFramework->OnKey(key, x, y);
}

void __idle()
{
	g_GLFramework->OnIdle();
	if(g_GLFramework->popRedisplayQuest())
		glutPostRedisplay();

	sleep(20);
}


int main(int argc, char* argv[])
{
	g_GLFramework = CGLFramework::getInstance();
	GLFInitialize(argc, argv);
	g_GLFramework->redisplayQuest();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowSize(g_GLFramework->MainPanel->Width, g_GLFramework->MainPanel->Height);
	glutCreateWindow(string(g_GLFramework->Caption).data());
	g_GLFramework->__windowInitialized();
	g_GLFramework->OnCreatedWindow();
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutDisplayFunc(__display);
	glutReshapeFunc(__resize);
	glutMouseFunc(__mouse);
	glutMotionFunc(__motion);
	glutPassiveMotionFunc(__passiveMotion);
	glutKeyboardFunc(__keybord);
	glutIdleFunc(__idle);
	glutMainLoop();
	return 0;
}
