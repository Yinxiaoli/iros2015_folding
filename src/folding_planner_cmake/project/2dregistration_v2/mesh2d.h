#ifndef __MESH_H__
#define __MESH_H__

#include <map>
#include <vector>
using namespace std;
#include <stdint.h>

struct SVertex;
struct SHalfEdge;
struct SFace;

struct SVertex
{
	SHalfEdge *e;
	bool boundary;
	int32_t id;
    bool valid;
    mutable bool mark;
};

struct SHalfEdge
{
	SHalfEdge *opposite;
	SVertex *target;
	SHalfEdge *prev, *next;
	SFace* f;

	int32_t hid; //not saved
	int32_t id;
    bool valid;
    mutable bool mark;
};

struct SFace
{
	SHalfEdge *e;
	int32_t id;
    bool valid;
    mutable bool mark;
};

struct SVertexPair
{
	SVertex* a;
	SVertex* b;
};

struct SHalfEdgeVertexPair
{
	SHalfEdge* e;
	SVertex* v;
};

struct SMesh
{
	vector<SVertex*> vertices;
	vector<SHalfEdge*> halfEdges;
	vector<SFace*> faces;
	map<SVertexPair, SHalfEdge*> halfedgeMap; // Do we need this?
	int32_t nEdges;
};

inline bool operator<(const SVertexPair& in_lhs, const SVertexPair& in_rhs)
{
	return (in_lhs.a < in_rhs.a) || ((in_lhs.a == in_rhs.a) && (in_lhs.b < in_rhs.b));
}

inline bool operator==(const SVertexPair& in_lhs, const SVertexPair& in_rhs)
{
	return (in_lhs.a == in_rhs.a) && (in_lhs.b == in_rhs.b);
}

inline bool operator<(const SHalfEdgeVertexPair& in_lhs, const SHalfEdgeVertexPair& in_rhs)
{
	return (in_lhs.e < in_rhs.e) || ((in_lhs.e == in_rhs.e) && (in_lhs.v < in_rhs.v));
}

inline bool operator==(const SHalfEdgeVertexPair& in_lhs, const SHalfEdgeVertexPair& in_rhs)
{
	return (in_lhs.e == in_rhs.e) && (in_lhs.v == in_rhs.v);
}

SVertex* addVertex(SMesh* io_Mesh);
SFace* addFace(SMesh* io_Mesh, SVertex* a, SVertex* b, SVertex* c);
void addVerticesFacesComplete(SMesh* io_Mesh);

void resetHalfEdgeofVertex(SVertex* a);
void clearMesh(SMesh* io_Mesh);

void removeInvalidElements(SMesh* io_Mesh, vector<int>& io_VidRef);

int getValence(const SVertex* in_v);
void markEdges(SMesh* io_Mesh);
void resetElementIDs(SMesh* io_Mesh);
void resetElementIDs(SMesh* io_Mesh, int* io_OldToNewVIDMap, int in_nOldToNewVIDMap);
void edgeFlip(SHalfEdge* io_Edge);
void edgeCollapse(SHalfEdge* io_Edge, SVertex* io_VertexToRemove, SMesh* io_Mesh);
int edgeSplit(SMesh* io_Mesh, SHalfEdge* io_Edge);
int snapToEdge(SMesh* io_Mesh, SHalfEdge* io_Edge);

bool testEdgeFlipRetainManifold(const SHalfEdge* in_Edge);
bool testEdgeCollapseRetainManifold(const SHalfEdge* in_Edge);

void loadMesh(char* fn, SMesh* io_Mesh);
void loadMeshInvert(char* fn, SMesh* io_Mesh);
void saveMesh(char* fn, const SMesh* in_Mesh);

bool meshValidityTest(const SMesh* in_Mesh);

#endif
