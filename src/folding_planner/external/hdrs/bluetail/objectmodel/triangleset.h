#ifndef __TRIANGLE_SET_H__
#define __TRIANGLE_SET_H__

#include <blackbill/mathbase/mathbase.h>
using namespace _BlackBill::_MathBase;

#include "../material/material.h"
using namespace _BlueTail::_Material;

#include <vector>
using namespace std;

namespace _BlueTail{
	namespace _ObjectModel{

		template<typename T> struct STriangleSet
		{
			unsigned int nVertices;//loaded, and will be reset
			T* v[3];//loaded, and will be reset
			T* vn[3];
			unsigned int nTriangles;//loaded
			unsigned int* t[3];//loaded
			T* tn[3];
			T* tbn[3];
			T* tcn[3];
			T* tu[3];
			T* tv[3];
			unsigned int* shaderID;

			unsigned int nMaterials;//loaded
			SMaterial<T>* materials;//loaded
		};

		template<typename T> void initTriangleSet(STriangleSet<T>* io_DataSet)
		{
			io_DataSet->nVertices = 0;
			io_DataSet->v[0] = NULL; io_DataSet->v[1] = NULL; io_DataSet->v[2] = NULL;
			io_DataSet->nTriangles = 0;
			io_DataSet->t[0] = NULL; io_DataSet->t[1] = NULL; io_DataSet->t[2] = NULL;
			io_DataSet->tn[0] = NULL; io_DataSet->tn[1] = NULL; io_DataSet->tn[2] = NULL;
			io_DataSet->tbn[0] = NULL; io_DataSet->tbn[1] = NULL; io_DataSet->tbn[2] = NULL;
			io_DataSet->tcn[0] = NULL; io_DataSet->tcn[1] = NULL; io_DataSet->tcn[2] = NULL;
			io_DataSet->tu[0] = NULL; io_DataSet->tu[1] = NULL; io_DataSet->tu[2] = NULL;
			io_DataSet->tv[0] = NULL; io_DataSet->tv[1] = NULL; io_DataSet->tv[2] = NULL;
			io_DataSet->shaderID = NULL;
			io_DataSet->nMaterials = 0;
			io_DataSet->materials = NULL;
		}

		template<typename T> void finalizeTriangleSet(STriangleSet<T>* io_DataSet)
		{
			if(io_DataSet->v[0] != NULL) free(io_DataSet->v[0]); if(io_DataSet->v[1] != NULL) free(io_DataSet->v[1]);
			if(io_DataSet->v[2] != NULL) free(io_DataSet->v[2]);
			if(io_DataSet->t[0] != NULL) free(io_DataSet->t[0]); if(io_DataSet->t[1] != NULL) free(io_DataSet->t[1]);
			if(io_DataSet->t[2] != NULL) free(io_DataSet->t[2]);
			if(io_DataSet->tn[0] != NULL) free(io_DataSet->tn[0]); if(io_DataSet->tn[1] != NULL) free(io_DataSet->tn[1]);
			if(io_DataSet->tn[2] != NULL) free(io_DataSet->tn[2]);
			if(io_DataSet->tbn[0] != NULL) free(io_DataSet->tbn[0]); if(io_DataSet->tbn[1] != NULL) free(io_DataSet->tbn[1]);
			if(io_DataSet->tbn[2] != NULL) free(io_DataSet->tbn[2]);
			if(io_DataSet->tcn[0] != NULL) free(io_DataSet->tcn[0]); if(io_DataSet->tcn[1] != NULL) free(io_DataSet->tcn[1]);
			if(io_DataSet->tcn[2] != NULL) free(io_DataSet->tcn[2]);
			if(io_DataSet->tu[0] != NULL) free(io_DataSet->tu[0]); if(io_DataSet->tu[1] != NULL) free(io_DataSet->tu[1]);
			if(io_DataSet->tu[2] != NULL) free(io_DataSet->tu[2]);
			if(io_DataSet->tv[0] != NULL) free(io_DataSet->tv[0]); if(io_DataSet->tv[1] != NULL) free(io_DataSet->tv[1]);
			if(io_DataSet->tv[2] != NULL) free(io_DataSet->tv[2]);
			if(io_DataSet->shaderID != NULL) free(io_DataSet->shaderID);
			if(io_DataSet->materials != NULL) free(io_DataSet->materials);
			initTriangleSet(io_DataSet);
		}

		template<typename T> void prepareTriangleSet(STriangleSet<T>* io_DataSet)
		{
			vector<int>** the_VertexConnectivity = (vector<int>**)malloc(sizeof(vector<int>*)*io_DataSet->nVertices);
			for(int i=0; i<io_DataSet->nVertices; i++) the_VertexConnectivity[i] = NULL;

			for(unsigned int i=0; i<io_DataSet->nTriangles; i++)
			{
				T x1 = io_DataSet->v[0][io_DataSet->t[0][i]]; T y1 = io_DataSet->v[1][io_DataSet->t[0][i]]; T z1 = io_DataSet->v[2][io_DataSet->t[0][i]];
				T x2 = io_DataSet->v[0][io_DataSet->t[1][i]]; T y2 = io_DataSet->v[1][io_DataSet->t[1][i]]; T z2 = io_DataSet->v[2][io_DataSet->t[1][i]];
				T x3 = io_DataSet->v[0][io_DataSet->t[2][i]]; T y3 = io_DataSet->v[1][io_DataSet->t[2][i]]; T z3 = io_DataSet->v[2][io_DataSet->t[2][i]];

				//triangle normal
				const T ba_ca_x = cross4(y2-y1, z3-z1, z2-z1, y3-y1); const T ba_ca_y = cross4(z2-z1, x3-x1, x2-x1, z3-z1); const T ba_ca_z = cross4(x2-x1, y3-y1, y2-y1, x3-x1);
				const T ac_bc_x = cross4(y1-y3, z2-z3, z1-z3, y2-y3); const T ac_bc_y = cross4(z1-z3, x2-x3, x1-x3, z2-z3); const T ac_bc_z = cross4(x1-x3, y2-y3, y1-y3, x2-x3);
				const T cb_ab_x = cross4(y3-y2, z1-z2, z3-z2, y1-y2); const T cb_ab_y = cross4(z3-z2, x1-x2, x3-x2, z1-z2); const T cb_ab_z = cross4(x3-x2, y1-y2, y3-y2, x1-x2);
				const T _x = ba_ca_x + ac_bc_x + cb_ab_x; const T _y = ba_ca_y + ac_bc_y + cb_ab_y; const T _z = ba_ca_z + ac_bc_z + cb_ab_z;
				const T _len = T(1.0)/length(_x, _y, _z);

				io_DataSet->tn[0][i] = _x * _len; io_DataSet->tn[1][i] = _y * _len; io_DataSet->tn[2][i] = _z * _len;

				const T len12 = length(x2-x1, y2-y1, z2-z1); const T len23 = length(x3-x2, y3-y2, z3-z2); const T len31 = length(x1-x3, y1-y3, z1-z3);
				const T _bnx = ((len12 > len23) && (len12 > len31)) ? x2-x1 : (((len31 > len12) && (len31 > len23)) ? x1-x3 : x3-x2);
				const T _bny = ((len12 > len23) && (len12 > len31)) ? y2-y1 : (((len31 > len12) && (len31 > len23)) ? y1-y3 : y3-y2);
				const T _bnz = ((len12 > len23) && (len12 > len31)) ? z2-z1 : (((len31 > len12) && (len31 > len23)) ? z1-z3 : z3-z2);
				const T inv_len_bn = T(1.0)/length(_bnx, _bny, _bnz);
				io_DataSet->tbn[0][i] = _bnx * inv_len_bn; io_DataSet->tbn[1][i] = _bny * inv_len_bn; io_DataSet->tbn[2][i] = _bnz * inv_len_bn;

				const T _cnx = cross4(io_DataSet->tn[1][i], io_DataSet->tbn[2][i], io_DataSet->tn[2][i], io_DataSet->tbn[1][i]);
				const T _cny = cross4(io_DataSet->tn[2][i], io_DataSet->tbn[0][i], io_DataSet->tn[0][i], io_DataSet->tbn[2][i]);
				const T _cnz = cross4(io_DataSet->tn[0][i], io_DataSet->tbn[1][i], io_DataSet->tn[1][i], io_DataSet->tbn[0][i]);
				const T inv_len_cn = T(1.0)/length(_cnx, _cny, _cnz);
				io_DataSet->tcn[0][i] = _cnx * inv_len_cn; io_DataSet->tcn[1][i] = _cny * inv_len_cn; io_DataSet->tcn[2][i] = _cnz * inv_len_cn;

				if(the_VertexConnectivity[io_DataSet->t[0][i]] == NULL) the_VertexConnectivity[io_DataSet->t[0][i]] = new vector<int>();
				if(the_VertexConnectivity[io_DataSet->t[1][i]] == NULL) the_VertexConnectivity[io_DataSet->t[1][i]] = new vector<int>();
				if(the_VertexConnectivity[io_DataSet->t[2][i]] == NULL) the_VertexConnectivity[io_DataSet->t[2][i]] = new vector<int>();

				the_VertexConnectivity[io_DataSet->t[0][i]]->push_back(i);
				the_VertexConnectivity[io_DataSet->t[1][i]]->push_back(i);
				the_VertexConnectivity[io_DataSet->t[2][i]]->push_back(i);
			}

			int* t[3];
			for(int k=0; k<3; k++)
			{
				t[k] = (int*)malloc(sizeof(int)*io_DataSet->nTriangles);
				for(int j=0; j<io_DataSet->nTriangles; j++) t[k][j] = -1;
			}

			T* v[3]; T* vn[3];
			for(int k=0; k<3; k++)
			{
				v[k] = (T*)malloc(sizeof(T)*io_DataSet->nVertices);
				vn[k] = (T*)malloc(sizeof(T)*io_DataSet->nVertices);
				memset(v[k], 0, sizeof(T)*io_DataSet->nVertices);
				memset(vn[k], 0, sizeof(T)*io_DataSet->nVertices);
			}

			int nVerts = 0;
			int nMaxVerts = io_DataSet->nVertices;
			const unsigned long N_ADDVERT = 1024 * 16;

			for(unsigned int i=0; i<io_DataSet->nTriangles; i++)
			{
				for(int j=0; j<3; j++)
				{
					int found_vertex_id = -1;
					T normal[3] = {io_DataSet->tn[0][i], io_DataSet->tn[1][i], io_DataSet->tn[2][i]};

					unsigned int _vi = io_DataSet->t[j][i];
					const T u_vi = io_DataSet->tu[j][i];
					const T v_vi = io_DataSet->tv[j][i];

					for(unsigned int k=0; k<the_VertexConnectivity[_vi]->size(); k++)
					{
						int _id = the_VertexConnectivity[_vi]->at(k);
						if(_id == i)
							continue;
						T neighbor_normal[3] = {io_DataSet->tn[0][_id], io_DataSet->tn[1][_id], io_DataSet->tn[2][_id]};
						T _dot = max<T>(-1.0, min<T>(1.0, dot(normal, neighbor_normal)));
						T theta = acos(_dot);
						T thr = io_DataSet->materials[io_DataSet->shaderID[i]].smoothing_threshold;
						if(theta <= thr)
						{
							if(io_DataSet->shaderID[i] == io_DataSet->shaderID[_id])
							{	
								for(int l=0; l<3; l++)
								{
									if((_vi == io_DataSet->t[l][_id]) && (t[l][_id] >= 0))
									{
										//uv
										const T u_t = io_DataSet->tu[l][_id];
										const T v_t = io_DataSet->tv[l][_id];

										if((u_t == u_vi) && (v_t == v_vi)) found_vertex_id = t[l][_id];
									}
								}
							}

							normal[0] += neighbor_normal[0];
							normal[1] += neighbor_normal[1];
							normal[2] += neighbor_normal[2];
						}
					}

					if(length(normal) < 1.0e-9)
						normal[0] = normal[1] = normal[2] = 0.0f;
					else
					{
						const T _len = 1.0f / length(normal);
						normal[0] *= _len; normal[1] *= _len; normal[2] *= _len;
					}

					if(found_vertex_id >= 0) t[j][i] = found_vertex_id;
					else
					{
						if(nVerts + 1 >= nMaxVerts)
						{
							nMaxVerts += N_ADDVERT;
							for(int k=0; k<3; k++)
							{
								v[k] = (T*)realloc(v[k], sizeof(T)*nMaxVerts);
								vn[k] = (T*)realloc(vn[k], sizeof(T)*nMaxVerts);
							}
						}
						t[j][i] = nVerts;

						v[0][nVerts] = io_DataSet->v[0][_vi];
						v[1][nVerts] = io_DataSet->v[1][_vi];
						v[2][nVerts] = io_DataSet->v[2][_vi];

						vn[0][nVerts] = normal[0]; vn[1][nVerts] = normal[1]; vn[2][nVerts] = normal[2];
						nVerts++;
					}
				}
			}

			free(io_DataSet->v[0]); free(io_DataSet->v[1]); free(io_DataSet->v[2]);
			io_DataSet->v[0] = v[0]; io_DataSet->v[1] = v[1]; io_DataSet->v[2] = v[2];
			io_DataSet->vn[0] = vn[0]; io_DataSet->vn[1] = vn[1]; io_DataSet->vn[2] = vn[2];
			io_DataSet->nVertices = nVerts;

			for(int i=0; i<io_DataSet->nTriangles; i++)
			{
				io_DataSet->t[0][i] = t[0][i]; io_DataSet->t[1][i] = t[1][i]; io_DataSet->t[2][i] = t[2][i];
			}

			free(t[0]); free(t[1]); free(t[2]);

			printf("Prepared: %d vertices, %d tris\n", io_DataSet->nVertices, io_DataSet->nTriangles);
		}

		template<typename T> void rePreparePrimitives(STriangleSet<T>* io_DataSet, const unsigned int in_StartPrimID, const unsigned int in_nPrims)
		{
			for(int i=in_StartPrimID; i<in_StartPrimID+in_nPrims; i++)
			{
				T x1 = v[0][t[0][i]]; T y1 = v[1][t[0][i]]; T z1 = v[2][t[0][i]];
				T x2 = v[0][t[1][i]]; T y2 = v[1][t[1][i]]; T z2 = v[2][t[1][i]];
				T x3 = v[0][t[2][i]]; T y3 = v[1][t[2][i]]; T z3 = v[2][t[2][i]];

				//triangle normal
				const T ba_ca_x = cross4(y2-y1, z3-z1, z2-z1, y3-y1); const T ba_ca_y = cross4(z2-z1, x3-x1, x2-x1, z3-z1); const T ba_ca_z = cross4(x2-x1, y3-y1, y2-y1, x3-x1);
				const T ac_bc_x = cross4(y1-y3, z2-z3, z1-z3, y2-y3); const T ac_bc_y = cross4(z1-z3, x2-x3, x1-x3, z2-z3); const T ac_bc_z = cross4(x1-x3, y2-y3, y1-y3, x2-x3);
				const T cb_ab_x = cross4(y3-y2, z1-z2, z3-z2, y1-y2); const T cb_ab_y = cross4(z3-z2, x1-x2, x3-x2, z1-z2); const T cb_ab_z = cross4(x3-x2, y1-y2, y3-y2, x1-x2);
				const T _x = ba_ca_x + ac_bc_x + cb_ab_x; const T _y = ba_ca_y + ac_bc_y + cb_ab_y; const T _z = ba_ca_z + ac_bc_z + cb_ab_z;
				const T _len = (1.0)/length(_x, _y, _z);

				io_DataSet->tn[0][i] = _x * _len; io_DataSet->tn[1][i] = _y * _len; io_DataSet->tn[2][i] = _z * _len;

				const T len12 = length(x2-x1, y2-y1, z2-z1); const T len23 = length(x3-x2, y3-y2, z3-z2); const T len31 = length(x1-x3, y1-y3, z1-z3);
				const T _bnx = ((len12 > len23) && (len12 > len31)) ? x2-x1 : (((len31 > len12) && (len31 > len23)) ? x1-x3 : x3-x2);
				const T _bny = ((len12 > len23) && (len12 > len31)) ? y2-y1 : (((len31 > len12) && (len31 > len23)) ? y1-y3 : y3-y2);
				const T _bnz = ((len12 > len23) && (len12 > len31)) ? z2-z1 : (((len31 > len12) && (len31 > len23)) ? z1-z3 : z3-z2);
				const T inv_len_bn = T(1.0)/length(_bnx, _bny, _bnz);
				io_DataSet->tbn[0][i] = _bnx * inv_len_bn; io_DataSet->tbn[1][i] = _bny * inv_len_bn; io_DataSet->tbn[2][i] = _bnz * inv_len_bn;

				const T _cnx = cross4(io_DataSet->tn[1][i], io_DataSet->tbn[2][i], io_DataSet->tn[2][i], io_DataSet->tbn[1][i]);
				const T _cny = cross4(io_DataSet->tn[2][i], io_DataSet->tbn[0][i], io_DataSet->tn[0][i], io_DataSet->tbn[2][i]);
				const T _cnz = cross4(io_DataSet->tn[0][i], io_DataSet->tbn[1][i], io_DataSet->tn[1][i], io_DataSet->tbn[0][i]);
				const T inv_len_cn = T(1.0)/length(_cnx, _cny, _cnz);
				io_DataSet->tcn[0][i] = _cnx * inv_len_cn; io_DataSet->tcn[1][i] = _cny * inv_len_cn; io_DataSet->tcn[2][i] = _cnz * inv_len_cn;
			}
		}
		
		template<typename T> void mergeTriangleSet(STriangleSet<T>* io_DataSet, const STriangleSet<T>* in_DataSet)
		{
			const unsigned int nTrianglesOld = io_DataSet->n_TotTriangles; const unsigned int nVerticesOld = io_DataSet->n_TotVertices;
			const unsigned int nMaterialsOld = io_DataSet->n_TotMaterials;
			const unsigned int nTrianglesNew = io_DataSet->n_TotTriangles + in_DataSet->n_TotTriangles; const unsigned int nVerticesNew = io_DataSet->n_TotVertices + in_DataSet->n_TotVertices;
			const unsigned int nMaterialsNew = io_DataSet->n_TotMaterials + in_DataSet->n_TotMaterials;

			for(int i=0; i<3; i++)
			{
				io_DataSet->v[i] = (T*)realloc(io_DataSet->v[i], sizeof(T)*nVerticesNew);
				memcpy(&(io_DataSet->v[i][nVerticesOld]), in_DataSet->v[i], sizeof(T)*in_DataSet->nVertices);

				io_DataSet->tn[i] = (T*)realloc(io_DataSet->tn[i], sizeof(T)*nTrianglesNew);
				memcpy(&(io_DataSet->tn[i][nTrianglesOld]), in_DataSet->tn[i], sizeof(T)*in_DataSet->nTriangles);
				io_DataSet->tbn[i] = (T*)realloc(io_DataSet->tbn[i], sizeof(T)*nTrianglesNew);
				memcpy(&(io_DataSet->tbn[i][nTrianglesOld]), in_DataSet->tbn[i], sizeof(T)*in_DataSet->nTriangles);
				io_DataSet->tcn[i] = (T*)realloc(io_DataSet->tcn[i], sizeof(T)*nTrianglesNew);
				memcpy(&(io_DataSet->tcn[i][nTrianglesOld]), in_DataSet->tcn[i], sizeof(T)*in_DataSet->nTriangles);
				io_DataSet->tu[i] = (T*)realloc(io_DataSet->tu[i], sizeof(T)*nTrianglesNew);
				memcpy(&(io_DataSet->tu[i][nTrianglesOld]), in_DataSet->tu[i], sizeof(T)*in_DataSet->nTriangles);
				io_DataSet->tv[i] = (T*)realloc(io_DataSet->tv[i], sizeof(T)*nTrianglesNew);
				memcpy(&(io_DataSet->tv[i][nTrianglesOld]), in_DataSet->tv[i], sizeof(T)*in_DataSet->nTriangles);
				io_DataSet->t[i] = (unsigned int*)realloc(io_DataSet->t[i], sizeof(unsigned int)*nTrianglesNew);
				memcpy(&(io_DataSet->t[i][nTrianglesOld]), in_DataSet->t[i], sizeof(unsigned int)*in_DataSet->nTriangles);

				for(int k=0; k<in_DataSet->nTriangles; k++) t[i][k+nTrianglesOld] += nVerticesOld;
			}

			io_DataSet->shaderID = (unsigned int*)realloc(io_DataSet->shaderID, sizeof(unsigned int)*nTrianglesNew);
			memcpy(&(io_DataSet->shaderID[nTrianglesOld]), in_DataSet->shaderID, sizeof(unsigned int)*in_DataSet->nTriangles);
			
			for(int k=0; k<in_DataSet->nTriangles; k++) io_DataSet->shaderID[k+nTrianglesOld] += nMaterialsOld;

			io_DataSet->materials = (SMaterial*)realloc(io_DataSet->materials, sizeof(SMaterial*)*nMaterialsNew);
			memcpy(&(io_DataSet->materials[nMaterialsOld]), in_DataSet->materials, sizeof(SMaterial)*in_DataSet->nMaterials);

			io_DataSet->nMaterials = nMaterialsNew;
			io_DataSet->nTriangles = nTrianglesNew;
			io_DataSet->nVertices = nVerticesNew;
		}
		
		void saveIntoDisk();
		void restoreFromDisk();
	};
};

#endif
