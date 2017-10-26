#include "distancefield.h"
#include <cmath>
using namespace std;

//http://www.codersnotes.com/notes/signed-distance-fields

struct SSDFPoint
{
	double dx, dy;
	double distSq() const { return dx*dx+dy*dy; }
};

struct SSDFGrid
{
	SSDFPoint* p;
	int width;
	int height;
};

const SSDFPoint INSIDE = {0, 0};
const SSDFPoint EMPTY = {1<<14, 1<<14};

SSDFPoint getPoint(SSDFGrid& g, int x, int y)
{
	if((x>=0) && (y>=0) && (x<g.width) && (y<g.height))
		return g.p[y*g.width+x];
	else
		return EMPTY;
}

void putPoint(SSDFGrid& g, int x, int y, const SSDFPoint& p)
{
	g.p[y*g.width+x] = p;
}

void compareValue(SSDFGrid& g, SSDFPoint& p, int x, int y, int offsetx, int offsety)
{
	SSDFPoint other = getPoint(g, x+offsetx, y+offsety);
	other.dx += offsetx;
	other.dy += offsety;
	
	if(other.distSq() < p.distSq())
		p = other;
}

void generateSDFGrid(SSDFGrid& g)
{
	for(int y=0; y<g.height; y++)
	{
		for(int x=0; x<g.width; x++)
		{
			SSDFPoint p = getPoint(g, x, y);
			compareValue(g, p, x, y, -1,  0);
			compareValue(g, p, x, y,  0, -1);
			compareValue(g, p, x, y, -1, -1);
			compareValue(g, p, x, y,  1, -1);
			putPoint(g, x, y, p);
		}
		
		for(int x=g.width-1; x>=0; x--)
		{
			SSDFPoint p = getPoint(g, x, y);
			compareValue(g, p, x, y, 1, 0);
			putPoint(g, x, y, p);
		}
	}
	
	for(int y=g.height-1; y>=0; y--)
	{
		for(int x=g.width-1; x>=0; x--)
		{
			SSDFPoint p = getPoint(g, x, y);
			compareValue(g, p, x, y,  1,  0);
			compareValue(g, p, x, y,  0,  1);
			compareValue(g, p, x, y, -1,  1);
			compareValue(g, p, x, y,  1,  1);
			putPoint(g, x, y, p);
		}
		
		for(int x=0; x<g.width; x++)
		{
			SSDFPoint p = getPoint(g, x, y);
			compareValue(g, p, x, y, -1, 0);
			putPoint(g, x, y, p);
		}
	}
}

void generateSDF(const SImage<double, 1>& in_BinaryImage, SImage<double, 1>& out_SDF)
{
	if((in_BinaryImage.width != out_SDF.width) || (in_BinaryImage.height != out_SDF.height)) 
		reallocImage(in_BinaryImage.width, in_BinaryImage.height, out_SDF);
		
	SSDFGrid g1, g2;
	g1.width = in_BinaryImage.width;
	g1.height = in_BinaryImage.height;
	g1.p = (SSDFPoint*)malloc(sizeof(SSDFPoint)*g1.width*g1.height);
	
	g2.width = in_BinaryImage.width;
	g2.height = in_BinaryImage.height;
	g2.p = (SSDFPoint*)malloc(sizeof(SSDFPoint)*g2.width*g2.height);
	
	for(int y=0; y<in_BinaryImage.height; y++)
	{
		for(int x=0; x<in_BinaryImage.width; x++)
		{
			if(in_BinaryImage.ptr[y*in_BinaryImage.width+x] < 0.5)
			{
				putPoint(g1, x, y, INSIDE);
				putPoint(g2, x, y, EMPTY);
			}
			else
			{
				putPoint(g2, x, y, INSIDE);
				putPoint(g1, x, y, EMPTY);
			}
		}
	}
	
	generateSDFGrid(g1);
	generateSDFGrid(g2);
	
	for(int y=0; y<in_BinaryImage.height; y++)
	{
		for(int x=0; x<in_BinaryImage.width; x++)
		{
			double dist1 = sqrt(getPoint(g1, x, y).distSq());
			double dist2 = sqrt(getPoint(g2, x, y).distSq());
			double dist = dist1 - dist2;
			out_SDF.ptr[y*in_BinaryImage.width+x] = dist;
		}
	}
	
	free(g1.p);
	free(g2.p);
}

void generateDF(const SImage<double, 1>& in_BinaryImage, SImage<double, 1>& out_DF)
{
	if((in_BinaryImage.width != out_DF.width) || (in_BinaryImage.height != out_DF.height)) 
		reallocImage(in_BinaryImage.width, in_BinaryImage.height, out_DF);
		
	SSDFGrid g1, g2;
	g1.width = in_BinaryImage.width;
	g1.height = in_BinaryImage.height;
	g1.p = (SSDFPoint*)malloc(sizeof(SSDFPoint)*g1.width*g1.height);
	
	g2.width = in_BinaryImage.width;
	g2.height = in_BinaryImage.height;
	g2.p = (SSDFPoint*)malloc(sizeof(SSDFPoint)*g2.width*g2.height);
	
	for(int y=0; y<in_BinaryImage.height; y++)
	{
		for(int x=0; x<in_BinaryImage.width; x++)
		{
			if(in_BinaryImage.ptr[y*in_BinaryImage.width+x] < 0.5)
			{
				putPoint(g1, x, y, INSIDE);
				putPoint(g2, x, y, EMPTY);
			}
			else
			{
				putPoint(g2, x, y, INSIDE);
				putPoint(g1, x, y, EMPTY);
			}
		}
	}
	
	generateSDFGrid(g1);
	generateSDFGrid(g2);
	
	for(int y=0; y<in_BinaryImage.height; y++)
	{
		for(int x=0; x<in_BinaryImage.width; x++)
		{
			double dist1 = sqrt(getPoint(g1, x, y).distSq());
			double dist2 = sqrt(getPoint(g2, x, y).distSq());
			double dist = dist1 - dist2;
			out_DF.ptr[y*in_BinaryImage.width+x] = fabs(dist);
		}
	}
	
	free(g1.p);
	free(g2.p);
}
