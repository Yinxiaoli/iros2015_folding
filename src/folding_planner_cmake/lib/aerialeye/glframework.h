#ifndef __GL_FRAMEWORK_H__
#define __GL_FRAMEWORK_H__

#include <blackbill/delegator/property.h>
#include <blackbill/delegator/eventhandler2.h>

#include <string>
using namespace std;

class CGLPanel;
class CGLDrawObject;

class CGLFramework
{
	static CGLFramework* m_Instance;
	CGLFramework();
	CGLFramework(const CGLFramework& in_i);
	~CGLFramework();

public:
	static CGLFramework* getInstance();
	void redisplayQuest();
	bool popRedisplayQuest();

	event<unsigned char, int, int> OnKey;
	event<> OnIdle;
	event<> OnCreatedWindow;

	property<string> Caption;
	propertyobj<CGLPanel> MainPanel;

	void __passiveMouseMotion(int in_GlobalX, int in_GlobalY);
	void __mouseMotion(int in_GlobalX, int in_GlobalY);
	void __mouse(int button, int state, int in_GlobalX, int in_GlobalY);

	void __windowInitialized();

protected:
	CGLPanel* m_MainPanel;
	string m_Caption;
	bool m_RedisplayQuest;

	bool m_WindowCreated;

	CGLPanel* getMainPanel();
	void setMainPanel(const CGLPanel& in_Panel);

	string getCaption();
	void setCaption(const string& in_Caption);

	CGLDrawObject* m_MouseFocusObject;
};

extern CGLFramework* g_GLFramework;

extern void GLFInitialize(int argc, char* argv[]);

#endif
