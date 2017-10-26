#include "glpanel.h"
#include "glframework.h"

#include <GL/freeglut.h>

CGLPanel::CGLPanel() : CGLDrawObject()
{
	OnDraw += bind1st(mem_func(&CGLPanel::drawChildren), this);
}

CGLPanel::~CGLPanel()
{

}

void CGLPanel::attachChild(CGLDrawObject* in_Child)
{
	m_Children.push_back(in_Child);
}

void CGLPanel::drawChildren()
{
	vector<CGLDrawObject*>::iterator p;
	vector<CGLDrawObject*> the_PriorityAlignObj;
	vector<CGLDrawObject*> the_AlignAllObj;
	vector<CGLDrawObject*> the_AlignDefaultObj;

	for(p=m_Children.begin(); p!=m_Children.end(); p++)
	{
		if((*p)->Align == A_DEFAULT) the_AlignDefaultObj.push_back(*p);
		else if((*p)->Align == A_ALL) the_AlignAllObj.push_back(*p);
		else the_PriorityAlignObj.push_back(*p);
	}

	if(the_AlignAllObj.size() > 1)
		printf("There are more than 2 objects having the property A_ALL. The objects may not be displayed as what was intended.\n");

	int region_left = 0;
	int region_top = 0;
	int region_width = m_Width;
	int region_height = m_Height;

	for(p=the_PriorityAlignObj.begin(); p!=the_PriorityAlignObj.end(); p++)
	{
		if((*p)->Align == A_LEFT)
		{
			(*p)->Left = region_left;
			(*p)->Top = region_top;
			(*p)->Height = region_height;
			region_left += (*p)->Width;
			region_width -= (*p)->Width;
		}
		else if((*p)->Align == A_TOP)
		{
			(*p)->Left = region_left;
			(*p)->Top = region_top;
			(*p)->Width = region_width;
			region_top += (*p)->Height;
			region_height -= (*p)->Height;
		}
		else if((*p)->Align == A_RIGHT)
		{
			(*p)->Left = region_width - (*p)->Width;
			(*p)->Top = region_top;
			(*p)->Height = region_height;
			region_width -= (*p)->Width;
		}
		else if((*p)->Align == A_BOTTOM)
		{
			(*p)->Left = region_left;
			(*p)->Top = region_height - (*p)->Height;
			(*p)->Width = region_width;
			region_height -= (*p)->Height;
		}
	}

	for(p=the_AlignAllObj.begin(); p!=the_AlignAllObj.end(); p++)
	{
		(*p)->Left = region_left;
		(*p)->Top = region_top;
		(*p)->Width = region_width;
		(*p)->Height = region_height;
	}

	for(p=m_Children.begin(); p!=m_Children.end(); p++)
	{
		(*p)->__setOrigin(m_OriginX+m_Left, m_OriginY+m_Top);
		(*p)->OnDraw();
	}
}

CGLDrawObject* CGLPanel::__inRegion(int in_GlobalX, int in_GlobalY)
{
	vector<CGLDrawObject*>::iterator p = m_Children.begin();
	CGLDrawObject* the_Obj = NULL;
	for(; p!=m_Children.end(); p++)
	{
		if((the_Obj = (*p)->__inRegion(in_GlobalX, in_GlobalY)) != NULL)
			break;
	}
	if(the_Obj != NULL)
		return the_Obj;
	else
		return CGLDrawObject::__inRegion(in_GlobalX, in_GlobalY);
}
