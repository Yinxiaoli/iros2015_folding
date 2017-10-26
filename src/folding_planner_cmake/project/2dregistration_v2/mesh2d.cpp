#include "mesh2d.h"

#include <iostream>
#include <cstdio>
#include <set>
using namespace std;

//from a to b
SHalfEdge* findHalfEdge(SMesh* in_Mesh, SVertex* a, SVertex* b)
{
	SVertexPair the_Pair;
	the_Pair.a = a; the_Pair.b = b;
	map<SVertexPair, SHalfEdge*>::iterator p = in_Mesh->halfedgeMap.find(the_Pair);
	if(p != in_Mesh->halfedgeMap.end()) return p->second;
	else return NULL;
}

//if v is on the boundary, set the related halfedge to the last one whose source is v, i.e., 
//that halfedge does not have its opposite halfedge.
//all halfedges pointing from v can be enumerated by utilizing backward search.
void resetHalfEdgeofVertex(SVertex* a)
{
	if(!a->boundary) return;
	SHalfEdge* startEdge = a->e;
	SHalfEdge* e = startEdge;
	while(1)
	{
		SHalfEdge* opposite = e->opposite;
		if(opposite == NULL) { a->e = e; return; }
		e = opposite->next;
		if(e == startEdge) { a->boundary = false; return; }
	}
}

void clearMesh(SMesh* io_Mesh)
{
	io_Mesh->halfedgeMap.clear();
	for(unsigned int i=0; i<io_Mesh->vertices.size(); i++) delete io_Mesh->vertices[i];
	for(unsigned int i=0; i<io_Mesh->halfEdges.size(); i++) delete io_Mesh->halfEdges[i];
	for(unsigned int i=0; i<io_Mesh->faces.size(); i++) delete io_Mesh->faces[i];

	io_Mesh->vertices.clear();
	io_Mesh->halfEdges.clear();
	io_Mesh->faces.clear();

	io_Mesh->nEdges = 0;
}

void removeInvalidElements(SMesh* io_Mesh, vector<int>& io_VidRef)
{
	io_VidRef.clear();
	
	int nv = 0;
	for(unsigned int i=0; i<io_Mesh->vertices.size(); i++)
	{
		if(!io_Mesh->vertices[i]->valid) continue;
		io_Mesh->vertices[nv++] = io_Mesh->vertices[i];
		io_VidRef.push_back(i);
	}
	io_Mesh->vertices.resize(nv);
	
	int ne = 0;
	for(unsigned int i=0; i<io_Mesh->halfEdges.size(); i++)
	{
		if(!io_Mesh->halfEdges[i]->valid) continue;
		io_Mesh->halfEdges[ne++] = io_Mesh->halfEdges[i];
	}
	io_Mesh->halfEdges.resize(ne);
	
	int nf = 0;
	for(unsigned int i=0; i<io_Mesh->faces.size(); i++)
	{
		if(!io_Mesh->faces[i]->valid) continue;
		io_Mesh->faces[nf++] = io_Mesh->faces[i];
	}
	io_Mesh->faces.resize(nf);
	
	resetElementIDs(io_Mesh);
}

SVertex* addVertex(SMesh* io_Mesh)
{
	if(io_Mesh == NULL) return NULL;

	SVertex* the_Vertex = new SVertex;
	the_Vertex->e = NULL;
	the_Vertex->boundary = true;
    the_Vertex->valid = true;
    the_Vertex->id = 0;
	io_Mesh->vertices.push_back(the_Vertex);

	return the_Vertex;
}

//should be counter clockwise
SFace* addFace(SMesh* io_Mesh, SVertex* a, SVertex* b, SVertex* c)
{
	//create half edges;
	SHalfEdge* e1 = new SHalfEdge;
	SHalfEdge* e2 = new SHalfEdge;
	SHalfEdge* e3 = new SHalfEdge;
	SFace* f = new SFace;

	e1->target = b;
	e2->target = c;
	e3->target = a;
    
    e1->valid = true;
    e2->valid = true;
    e3->valid = true;

	SVertexPair the_P1; the_P1.a = a; the_P1.b = b;
	SVertexPair the_P2; the_P2.a = b; the_P2.b = c;
	SVertexPair the_P3; the_P3.a = c; the_P3.b = a;

	io_Mesh->halfedgeMap.insert(pair<SVertexPair, SHalfEdge*>(the_P1, e1));
	io_Mesh->halfedgeMap.insert(pair<SVertexPair, SHalfEdge*>(the_P2, e2));
	io_Mesh->halfedgeMap.insert(pair<SVertexPair, SHalfEdge*>(the_P3, e3));

	e1->next = e2; e1->prev = e3;
	e2->next = e3; e2->prev = e1;
	e3->next = e1; e3->prev = e2;

	e1->f = f; e2->f = f; e3->f = f;
	f->e = e1;
	f->id = 0;
    f->valid = true;

	if(a->e == NULL) a->e = e1;
	if(b->e == NULL) b->e = e2;
	if(c->e == NULL) c->e = e3;

	e1->opposite = findHalfEdge(io_Mesh, b, a);
	if(e1->opposite != NULL) e1->opposite->opposite = e1;
	e2->opposite = findHalfEdge(io_Mesh, c, b);
	if(e2->opposite != NULL) e2->opposite->opposite = e2;
	e3->opposite = findHalfEdge(io_Mesh, a, c);
	if(e3->opposite != NULL) e3->opposite->opposite = e3;

	io_Mesh->halfEdges.push_back(e1);
	io_Mesh->halfEdges.push_back(e2);
	io_Mesh->halfEdges.push_back(e3);

	io_Mesh->faces.push_back(f);

	return f;
}

void addVerticesFacesComplete(SMesh* io_Mesh)
{
	vector<SVertex*>::iterator pv = io_Mesh->vertices.begin();
	for(; pv!=io_Mesh->vertices.end(); pv++)
	{
        if(!(*pv)->valid) continue;        
		(*pv)->boundary = true;
		resetHalfEdgeofVertex(*pv);
	}
}

bool testEdgeFlipRetainManifold(const SHalfEdge* in_Edge)
{
	const SVertex* vl = in_Edge->next->target;
	const SVertex* vr = in_Edge->opposite->next->target;
			
	const SHalfEdge* startEdge = vl->e;
	const SHalfEdge* e = startEdge;
	while(1)
	{
		if(e->target == vr) return false;
					
		const SHalfEdge* prev = e->prev;
		e = prev->opposite;
		if(e == NULL) break;
		if(e == startEdge) break;
	}
			
	startEdge = vr->e;
	e = startEdge;
	while(1)
	{
		if(e->target == vl) return false;
					
		const SHalfEdge* prev = e->prev;
		e = prev->opposite;
		if(e == NULL) break;
		if(e == startEdge) break;
	}
	
	return true;
}

void edgeFlip(SHalfEdge* io_Edge)
{
	if((io_Edge == NULL) || (io_Edge->opposite == NULL)) return;

	// mesh after swap should still be a manifold

	SHalfEdge* el_1 = io_Edge->next; SHalfEdge* el_2 = io_Edge->prev;
	SHalfEdge* er_1 = io_Edge->opposite->next; SHalfEdge* er_2 = io_Edge->opposite->prev;

	SVertex* vl = io_Edge->next->target;
	SVertex* vr = io_Edge->opposite->next->target;

	SVertex* vto = io_Edge->target;
	SVertex* vfrom = io_Edge->opposite->target;

	io_Edge->target = vl;
	io_Edge->opposite->target = vr;

	io_Edge->next = el_2;
	io_Edge->prev = er_1;

	io_Edge->opposite->next = er_2;
	io_Edge->opposite->prev = el_1;

	io_Edge->f->e = io_Edge;
	io_Edge->opposite->f->e = io_Edge->opposite;

	el_1->next = io_Edge->opposite;
	el_1->prev = er_2;
	el_1->f = io_Edge->opposite->f;
	el_2->next = er_1;
	el_2->prev = io_Edge;
	el_2->f = io_Edge->f;

	er_1->next = io_Edge;
	er_1->prev = el_2;
	er_1->f = io_Edge->f;
	er_2->next = el_1;
	er_2->prev = io_Edge->opposite;
	er_2->f = io_Edge->opposite->f;

	vto->e = el_1;
	vfrom->e = er_1;
	vl->boundary = true;
	resetHalfEdgeofVertex(vl);
	vr->boundary = true;
	resetHalfEdgeofVertex(vr);
	vto->boundary = true;
	resetHalfEdgeofVertex(vto);
	vfrom->boundary = true;
	resetHalfEdgeofVertex(vfrom);
}

bool testEdgeCollapseRetainManifold(const SHalfEdge* in_Edge)
{
	const SVertex* v1 = in_Edge->prev->target;
	const SVertex* v2 = in_Edge->target;
			
	set<const SVertex*> v1_neighbors;
	set<const SVertex*> v2_neighbors;
			
	const SHalfEdge* startEdge = v1->e;
	const SHalfEdge* e = startEdge;
	while(1)
	{
		v1_neighbors.insert(e->target);
		const SHalfEdge* prev = e->prev;
		e = prev->opposite;
		if(e == NULL) 
		{
			v1_neighbors.insert(prev->prev->target);
			break;
		}
		if(e == startEdge) break;
	}
			
	startEdge = v2->e;
	e = startEdge;
	while(1)
	{
		v2_neighbors.insert(e->target);
		const SHalfEdge* prev = e->prev;
		e = prev->opposite;
		if(e == NULL)
		{
			v2_neighbors.insert(prev->prev->target);
			break;
		}
		if(e == startEdge) break;
	}
			
	const SVertex* third_v1 = in_Edge->next->target;
	const SVertex* third_v2 = NULL;
	if(in_Edge->opposite) third_v2 = in_Edge->opposite->next->target;
			
	//check common adjacent vertices
	set<const SVertex*>::iterator p = v1_neighbors.begin();
	for(; p!=v1_neighbors.end(); p++)
	{
		set<const SVertex*>::iterator q = v2_neighbors.find(*p);
		if(q != v2_neighbors.end())
		{
			if((*q != third_v1) && (*q != third_v2)) return false;
		}
	}
			
	return true;
}

void replaceVertex(SVertex* source, SVertex* dest, const SHalfEdge* deletingEdge)
{
	SHalfEdge* startEdge = source->e;
	SHalfEdge* e = startEdge;

	//backward search
	while(1)
	{
		SHalfEdge* prev = e->prev;
		if(prev != deletingEdge) prev->target = dest;
		e = prev->opposite;
		if(e == NULL) break;
		if(e == startEdge) break;
	}
}

void bondSideEdges(SHalfEdge* e)
{
	if(e == NULL) return;

	if(e->next->opposite)
		e->next->opposite->opposite = e->prev->opposite;

	if(e->prev->opposite)
		e->prev->opposite->opposite = e->next->opposite;
}

void edgeCollapse(SHalfEdge* io_Edge, SVertex* io_VertexToRemove, SMesh* io_Mesh)
{
    if((io_Edge->target != io_VertexToRemove) && (io_Edge->prev->target != io_VertexToRemove))
    {
        printf("Edge collapse: the vertex is not on the edge...\n");
        return;
    }
    
    // mesh after collapse should still be a manifold
    
    SVertex* v1 = io_VertexToRemove;
	SVertex* v2;
    if(io_Edge->target != io_VertexToRemove) v2 = io_Edge->target;
    else v2 = io_Edge->prev->target;
    
	replaceVertex(v1, v2, io_Edge);

	bondSideEdges(io_Edge);
	bondSideEdges(io_Edge->opposite);

	if(io_Edge->next->opposite == NULL)
		v2->e = io_Edge->prev->opposite;
	else
		v2->e = io_Edge->next->opposite->next;

	//reset edges of the third vertices;

	v2->boundary = true;
	resetHalfEdgeofVertex(v2);
	
	if(io_Edge->next->target->e == io_Edge->prev)
	{
		if(io_Edge->next->opposite == NULL)
			io_Edge->next->target->e = io_Edge->prev->opposite->next;
		else
			io_Edge->next->target->e = io_Edge->next->opposite;
	}
	io_Edge->next->target->boundary = true;
	resetHalfEdgeofVertex(io_Edge->next->target);
	
	if(io_Edge->opposite)
	{
		if(io_Edge->opposite->next->target->e == io_Edge->opposite->prev)
		{
			if(io_Edge->opposite->next->opposite == NULL)
				io_Edge->opposite->next->target->e = io_Edge->opposite->prev->opposite->next;
			else
				io_Edge->opposite->next->target->e = io_Edge->opposite->next->opposite;
		}
		io_Edge->opposite->next->target->boundary = true;
		resetHalfEdgeofVertex(io_Edge->opposite->next->target);
	}

    io_Edge->f->valid = false;
    io_Edge->valid = false;
    io_Edge->next->valid = false;
    io_Edge->prev->valid = false;
	
	if(io_Edge->opposite != NULL)
	{
		io_Edge->opposite->f->valid = false;
        io_Edge->opposite->valid = false;
        io_Edge->opposite->next->valid = false;
        io_Edge->opposite->prev->valid = false;
	}

    v1->valid = false;
/*
    SVertexPair the_Pair;
    the_Pair.a = io_Edge->target; the_Pair.b = io_Edge->next->target;
    map<SVertexPair, SHalfEdge*>::iterator p = io_Mesh->halfedgeMap.find(the_Pair);
	if(p != io_Mesh->halfedgeMap.end()) io_Mesh->halfedgeMap.erase(p);
    
    the_Pair.a = io_Edge->next->target; the_Pair.b = io_Edge->prev->target;
    p = io_Mesh->halfedgeMap.find(the_Pair);
	if(p != io_Mesh->halfedgeMap.end()) io_Mesh->halfedgeMap.erase(p);
    
    the_Pair.a = io_Edge->prev->target; the_Pair.b = io_Edge->target;
    p = io_Mesh->halfedgeMap.find(the_Pair);
	if(p != io_Mesh->halfedgeMap.end()) io_Mesh->halfedgeMap.erase(p);
    
    if(io_Edge->opposite != NULL)
	{
        the_Pair.a = io_Edge->opposite->target; the_Pair.b = io_Edge->opposite->next->target;
        p = io_Mesh->halfedgeMap.find(the_Pair);
        if(p != io_Mesh->halfedgeMap.end()) io_Mesh->halfedgeMap.erase(p);
        
        the_Pair.a = io_Edge->opposite->next->target; the_Pair.b = io_Edge->opposite->prev->target;
        p = io_Mesh->halfedgeMap.find(the_Pair);
        if(p != io_Mesh->halfedgeMap.end()) io_Mesh->halfedgeMap.erase(p);
        
        the_Pair.a = io_Edge->opposite->prev->target; the_Pair.b = io_Edge->opposite->target;
        p = io_Mesh->halfedgeMap.find(the_Pair);
        if(p != io_Mesh->halfedgeMap.end()) io_Mesh->halfedgeMap.erase(p);
    }
    //*/
}

int edgeSplit(SMesh* io_Mesh, SHalfEdge* io_Edge)
{
	SVertex* new_v = new SVertex;
	new_v->id = io_Mesh->vertices.size();
    new_v->valid = true;

	io_Edge->f->e = io_Edge->prev;
	SFace* new_f = new SFace;
	new_f->id = io_Mesh->faces.size();
	new_f->e = io_Edge->next;
    new_f->valid = true;
	io_Edge->next->f = new_f;
    
    io_Edge->target->e = io_Edge->next;
	
	SHalfEdge* new_e = new SHalfEdge;
    new_e->valid = true;

	new_e->opposite = NULL;
	new_e->target = io_Edge->target;
	io_Edge->target = new_v;

	new_v->e = new_e;

	SHalfEdge* center_e_old = new SHalfEdge;
	SHalfEdge* center_e_new = new SHalfEdge;
	
    center_e_old->valid = true;
    center_e_new->valid = true;

	center_e_old->opposite = center_e_new;
	center_e_new->opposite = center_e_old;
	
	//printf("b\n");

	center_e_old->target = io_Edge->next->target;
	center_e_new->target = new_v;

	center_e_old->f = io_Edge->f;
	center_e_new->f = new_f;

	center_e_old->prev = io_Edge;
	center_e_old->next = io_Edge->prev;

	center_e_new->prev = io_Edge->next;
	center_e_new->next = new_e;

	io_Edge->next->next = center_e_new;
	io_Edge->prev->prev = center_e_old;

	new_e->prev = center_e_new;
	new_e->next = io_Edge->next;
	new_e->f = new_f;

	io_Edge->next->target->boundary = true;
	resetHalfEdgeofVertex(io_Edge->next->target);

	io_Edge->next->prev = new_e;
	io_Edge->next = center_e_old;
	
	io_Mesh->vertices.push_back(new_v);
	io_Mesh->faces.push_back(new_f);
	io_Mesh->halfEdges.push_back(new_e);
    io_Mesh->halfEdges.push_back(center_e_old);
    io_Mesh->halfEdges.push_back(center_e_new);
    
	if(io_Edge->opposite)
	{
		io_Edge->opposite->f->e = io_Edge->opposite->next;
		SFace* new_fo = new SFace;
		new_fo->id = io_Mesh->faces.size();
		
		new_fo->e = io_Edge->opposite->prev;
        new_fo->valid = true;
		io_Edge->opposite->prev->f = new_fo;

		SHalfEdge* new_eo = new SHalfEdge;
        new_eo->valid = true;
		new_eo->target = new_v;
		
		new_e->opposite = new_eo;
		new_eo->opposite = new_e;
		new_eo->f = new_fo;

		SHalfEdge* center_eo_old = new SHalfEdge;
		SHalfEdge* center_eo_new = new SHalfEdge;

        center_eo_old->valid = true;
        center_eo_new->valid = true;
        
		center_eo_old->opposite = center_eo_new;
		center_eo_new->opposite = center_eo_old;

		center_eo_old->target = new_v;
		center_eo_new->target = io_Edge->opposite->next->target;
		
		center_eo_old->f = io_Edge->opposite->f;
		center_eo_new->f = new_fo;

		center_eo_old->prev = io_Edge->opposite->next;
		center_eo_old->next = io_Edge->opposite;

		center_eo_new->prev = new_eo;
		center_eo_new->next = io_Edge->opposite->prev;

		new_eo->prev = io_Edge->opposite->prev;
		new_eo->next = center_eo_new;

		io_Edge->opposite->prev->prev = center_eo_new;
		io_Edge->opposite->next->next = center_eo_old;
		
		io_Edge->opposite->next->target->boundary = true;
		resetHalfEdgeofVertex(io_Edge->opposite->next->target);
		
		io_Edge->opposite->prev->next = new_eo;
		io_Edge->opposite->prev = center_eo_old;
		
		io_Mesh->faces.push_back(new_fo);
		io_Mesh->halfEdges.push_back(new_eo);
		io_Mesh->halfEdges.push_back(center_eo_old);
    	io_Mesh->halfEdges.push_back(center_eo_new);    	
	}

	io_Edge->prev->target->boundary = true;
	resetHalfEdgeofVertex(io_Edge->prev->target);
	new_e->target->boundary = true;
	resetHalfEdgeofVertex(new_e->target);
	new_v->boundary = true;
	resetHalfEdgeofVertex(new_v);
	
	return new_v->id;
}

//snap io_Edge->v to io_Edge
int snapToEdge(SMesh* io_Mesh, SHalfEdge* io_Edge)
{
	SVertex* v = io_Edge->next->target;
	int newVid = edgeSplit(io_Mesh, io_Edge);
	edgeCollapse(io_Edge->next, v, io_Mesh);

	return newVid;
}

int getValence(const SVertex* in_v)
{
	if(in_v->e == NULL) return 0;
	SHalfEdge* startEdge = in_v->e;
	SHalfEdge* e = startEdge;
	//backward search
	int count = 1;
	while(1)
	{
		SHalfEdge* prev = e->prev;
		count++;
		e = prev->opposite;
		if(e == NULL) return count;
		if(e == startEdge) return count-1;
	}
}

void markEdges(SMesh* io_Mesh)
{
	vector<SHalfEdge*>::iterator pe = io_Mesh->halfEdges.begin();
	for(; pe!=io_Mesh->halfEdges.end(); pe++)
    {
        //if(!(*pe)->valid) continue;
		(*pe)->mark = false;
    }

	pe = io_Mesh->halfEdges.begin();
	for(; pe!=io_Mesh->halfEdges.end(); pe++)
	{
        if(!(*pe)->valid) continue;
		if(((*pe)->opposite == NULL) || ((*pe)->opposite->mark == false))
			(*pe)->mark = true;
	}
}

void resetElementIDs(SMesh* io_Mesh)
{
	int id = 0;
	vector<SVertex*>::iterator pv = io_Mesh->vertices.begin();
	for(; pv!=io_Mesh->vertices.end(); pv++)
    {
        if(!(*pv)->valid) continue;
		(*pv)->id = id++;
    }

	int hid = 0; id = 0;
	markEdges(io_Mesh);
	vector<SHalfEdge*>::iterator pe = io_Mesh->halfEdges.begin();
	for(; pe!=io_Mesh->halfEdges.end(); pe++)
	{
        if(!(*pe)->valid) continue;
		(*pe)->hid = hid++;
		if((*pe)->mark)
		{
			(*pe)->id = id++;
			if((*pe)->opposite) (*pe)->opposite->id = (*pe)->id;
		}
	}
	io_Mesh->nEdges = id;

	id=0;
	vector<SFace*>::iterator pf = io_Mesh->faces.begin();
	for(; pf!=io_Mesh->faces.end(); pf++)
    {
        if(!(*pf)->valid) continue;
		(*pf)->id = id++;
    }
}

void resetElementIDs(SMesh* io_Mesh, int* io_OldToNewVIDMap, int in_nOldToNewVIDMap)
{
	for(int i=0; i<in_nOldToNewVIDMap; i++)
		io_OldToNewVIDMap[i] = -1;
	
	int id = 0;
	vector<SVertex*>::iterator pv = io_Mesh->vertices.begin();
	for(; pv!=io_Mesh->vertices.end(); pv++)
	{
        if(!(*pv)->valid) continue;
		int newid = id;
		io_OldToNewVIDMap[(*pv)->id] = id;
		(*pv)->id = newid;
		id++;
	}	

	int hid = 0; id = 0;
	markEdges(io_Mesh);
	vector<SHalfEdge*>::iterator pe = io_Mesh->halfEdges.begin();
	for(; pe!=io_Mesh->halfEdges.end(); pe++)
	{
        if(!(*pe)->valid) continue;
		(*pe)->hid = hid++;
		if((*pe)->mark)
		{
			(*pe)->id = id++;
			if((*pe)->opposite) (*pe)->opposite->id = (*pe)->id;
		}
	}
	io_Mesh->nEdges = id;

	id=0;
	vector<SFace*>::iterator pf = io_Mesh->faces.begin();
	for(; pf!=io_Mesh->faces.end(); pf++)
    {
        if(!(*pf)->valid) continue;
		(*pf)->id = id++;
    }
}

void loadMesh(char* fn, SMesh* io_Mesh)
{
	clearMesh(io_Mesh);

#ifdef _MSC_VER
	FILE* f;
	fopen_s(&f, fn, "rb");
#else
	FILE* f = fopen(fn, "rb");
#endif

	vector<SVertex*> the_VertPtrVector;

	int32_t nVertices;
	fread(&nVertices, sizeof(int32_t), 1, f);
	for(int i=0; i<nVertices; i++) the_VertPtrVector.push_back(addVertex(io_Mesh));

	int32_t nFaces;
	fread(&nFaces, sizeof(int32_t), 1, f);
	for(int i=0; i<nFaces; i++)
	{
		int32_t id1, id2, id3;
		fread(&id1, sizeof(int32_t), 1, f);
		fread(&id2, sizeof(int32_t), 1, f);
		fread(&id3, sizeof(int32_t), 1, f);

		SVertex* v1 = the_VertPtrVector[id1];
		SVertex* v2 = the_VertPtrVector[id2];
		SVertex* v3 = the_VertPtrVector[id3];

		addFace(io_Mesh, v1, v2, v3);
	}
	
	fclose(f);

	addVerticesFacesComplete(io_Mesh);
	resetElementIDs(io_Mesh);
}

void loadMeshInvert(char* fn, SMesh* io_Mesh)
{
	clearMesh(io_Mesh);

#ifdef _MSC_VER
	FILE* f;
	fopen_s(&f, fn, "rb");
#else
	FILE* f = fopen(fn, "rb");
#endif

	vector<SVertex*> the_VertPtrVector;

	int32_t nVertices;
	fread(&nVertices, sizeof(int32_t), 1, f);
	for(int i=0; i<nVertices; i++) the_VertPtrVector.push_back(addVertex(io_Mesh));

	map<SVertexPair, SHalfEdge*> the_VertEdgeMap;

	int32_t nFaces;
	fread(&nFaces, sizeof(int32_t), 1, f);
	for(int i=0; i<nFaces; i++)
	{
		int32_t id1, id2, id3;
		fread(&id1, sizeof(int32_t), 1, f);
		fread(&id2, sizeof(int32_t), 1, f);
		fread(&id3, sizeof(int32_t), 1, f);

		SVertex* v1 = the_VertPtrVector[id1];
		SVertex* v2 = the_VertPtrVector[id3];
		SVertex* v3 = the_VertPtrVector[id2];
		
		addFace(io_Mesh, v1, v2, v3);
	}
	
	fclose(f);

	addVerticesFacesComplete(io_Mesh);
	resetElementIDs(io_Mesh);
}

void saveMesh(char* fn, const SMesh* in_Mesh)
{
#ifdef _MSC_VER
	FILE* f;
	fopen_s(&f, fn, "wb");
#else
	FILE* f = fopen(fn, "wb");
#endif
    
    //need to apply mesh compaction!!

	int32_t nVertices = in_Mesh->vertices.size();
	fwrite(&nVertices, sizeof(int32_t), 1, f);

	int32_t nFaces = in_Mesh->faces.size();
	fwrite(&nFaces, sizeof(int32_t), 1, f);
	for(int i=0; i<nFaces; i++)
	{
		int32_t id1 = in_Mesh->faces[i]->e->target->id;
		int32_t id2 = in_Mesh->faces[i]->e->next->target->id;
		int32_t id3 = in_Mesh->faces[i]->e->next->next->target->id;
		fwrite(&id1, sizeof(int32_t), 1, f);
		fwrite(&id2, sizeof(int32_t), 1, f);
		fwrite(&id3, sizeof(int32_t), 1, f);
	}
	
	fclose(f);
}

bool meshValidityTest(const SMesh* in_Mesh)
{
	printf("#1: v->e validity\n");
	vector<SVertex*>::const_iterator pv = in_Mesh->vertices.begin();
	for(; pv!=in_Mesh->vertices.end(); pv++)
	{
		if(!(*pv)->valid) continue;
		if(!(*pv)->e->valid)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#2: f->e validity\n");
	vector<SFace*>::const_iterator pf = in_Mesh->faces.begin();
	for(; pf!=in_Mesh->faces.end(); pf++)
	{
		if(!(*pf)->valid) continue;
		if(!(*pf)->e->valid)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#3: e->* validity\n");
	vector<SHalfEdge*>::const_iterator pe = in_Mesh->halfEdges.begin();
	for(; pe!=in_Mesh->halfEdges.end(); pe++)
	{
		if(!(*pe)->valid) continue;
		if(!(*pe)->target->valid || !(*pe)->f->valid || !(*pe)->prev->valid || !(*pe)->next->valid || ((*pe)->opposite && !(*pe)->opposite->valid))
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#4: v->e->prev->target == v\n");
	pv = in_Mesh->vertices.begin();
	for(; pv!=in_Mesh->vertices.end(); pv++)
	{
		if(!(*pv)->valid) continue;
		if((*pv)->e->prev->target != *pv)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#5: e->opposite->opposite == e\n");
	pe = in_Mesh->halfEdges.begin();
	for(; pe!=in_Mesh->halfEdges.end(); pe++)
	{
		if(!(*pe)->valid) continue;
		if(!(*pe)->opposite) continue;
		if((*pe)->opposite->opposite != *pe)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#6: e->target == e->opposite->prev->target\n");
	pe = in_Mesh->halfEdges.begin();
	for(; pe!=in_Mesh->halfEdges.end(); pe++)
	{
		if(!(*pe)->valid) continue;
		if(!(*pe)->opposite) continue;
		if((*pe)->target != (*pe)->opposite->prev->target)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#7: e->prev->target == e->opposite->target\n");
	pe = in_Mesh->halfEdges.begin();
	for(; pe!=in_Mesh->halfEdges.end(); pe++)
	{
		if(!(*pe)->valid) continue;
		if(!(*pe)->opposite) continue;
		if((*pe)->prev->target != (*pe)->opposite->target)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#8: f->e->next->next->next == f->e\n");
	pf = in_Mesh->faces.begin();
	for(; pf!=in_Mesh->faces.end(); pf++)
	{
		if(!(*pf)->valid) continue;
		if((*pf)->e->next->next->next != (*pf)->e)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#9: e in e->f->edges\n");
	pe = in_Mesh->halfEdges.begin();
	for(; pe!=in_Mesh->halfEdges.end(); pe++)
	{
		if(!(*pe)->valid) continue;
		if(((*pe)!=(*pe)->f->e) && ((*pe)!=(*pe)->f->e->next) && ((*pe)!=(*pe)->f->e->next->next))
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	printf("#10: vertex manifold\n");
	vector<SHalfEdge*>* the_EdgeLists = new vector<SHalfEdge*>[in_Mesh->vertices.size()];
	pe = in_Mesh->halfEdges.begin();
	for(; pe!=in_Mesh->halfEdges.end(); pe++)
	{
		if(!(*pe)->valid) continue;
		the_EdgeLists[(*pe)->prev->target->id].push_back(*pe);
		the_EdgeLists[(*pe)->target->id].push_back(*pe);
	}
	pv = in_Mesh->vertices.begin();
	for(; pv!=in_Mesh->vertices.end(); pv++)
	{
		if(!(*pv)->valid) continue;
		
		int noOpposite = 0;
		vector<SHalfEdge*>::const_iterator q = the_EdgeLists[(*pv)->id].begin();
		for(; q!=the_EdgeLists[(*pv)->id].end(); q++) if(!(*q)->opposite) noOpposite++;
		
		if((noOpposite != 0) && (noOpposite != 2))
		{
			printf("failed...\n");
			return false;
		}
	}
	delete[] the_EdgeLists;
	printf("passed!\n");
	
	printf("#11: va-->vb duplication\n");
	set<SVertexPair> the_VertexPairs;
	pe = in_Mesh->halfEdges.begin();
	for(; pe!=in_Mesh->halfEdges.end(); pe++)
	{
		if(!(*pe)->valid) continue;
		
		SVertexPair p;
		p.a = (*pe)->prev->target; p.b = (*pe)->target;
		pair<set<SVertexPair>::iterator, bool> ret = the_VertexPairs.insert(p);
		if(ret.second == false)
		{
			printf("failed...\n");
			return false;
		}
	}
	printf("passed!\n");
	
	return true;
}

