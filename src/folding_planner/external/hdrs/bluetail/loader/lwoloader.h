#ifndef __LWO_LOADER_H__
#define __LWO_LOADER_H__

#include "../objectmodel/triangleset.h"
using namespace _BlueTail::_ObjectModel;

#include <LWObj2/obj2.h>
#include <LWObj2/lwob.h>
#include <LWObj2/lwo2.h>
#pragma comment(lib, "obj2.lib")

#include "../image/image.h"
#include "../image/imageiohdr.h"
using namespace _BlueTail::_Image;

#include <iostream>
using namespace std;

#include "../material/material.h"
using namespace _BlueTail::_Material;

#include <blackbill/filepath/filepath.h>
using namespace _BlackBill::_FilePath;

//#include <antique/progressnotify/progressnotify.h>
//using namespace _Antique::_ProgressNotify;

namespace _BlueTail{
	namespace _Loader{

		//bool loadLwo(const char* in_currentdir, const char* in_filename, SSceneDescriptor* io_Scene, CProgressNotify* io_Notifier);
		
		inline const char* getTexName(const char* in_char)
		{
			const char* ptr = strrchr(in_char, '\\');
			if(ptr == NULL)
				return in_char;
			else
				return ptr + 1;
		}

		inline OBJ2_ID openLwo(const char* in_filename)
		{
			OBJ2_option opt;

			//opt.ope_code = OPT_OPE_SORT | OPT_OPE_TRIPLE | OPT_OPE_UVMAP | OPT_OPE_NORMAL;
			opt.ope_code = OPT_OPE_SORT | OPT_OPE_TRIPLE | OPT_OPE_UVMAP;
			opt.format_flags = OBJ_FLAG_NONE;
			opt.normal_flags = OPT_NORM_COMPRESS;
			opt.uvmap_flags = OPT_UV_COMPRESS;// | OPT_UV_IDENTIFY;
			opt.uvmap_channel = MAT_CHAN_ALL;
			opt.uvmap_nlayrs = 9;
			opt.triple_flags = OPT_TRI_NONE;

			return OBJ2_open(const_cast<char*>(in_filename), &opt);
		}

		template<typename T>
		int loadVertices(OBJ2_ID in_obj, T* v[3], int in_Offset)
		{
			int nPoint = (*in_obj->pointCount)(in_obj);
			PointData* p_info;
			for(int i=0; i<nPoint; i++)
			{
				p_info = (*in_obj->pointInfo)(in_obj, i);
				v[0][i+in_Offset] = T((p_info->position)[0]);
				v[1][i+in_Offset] = T((p_info->position)[1]);
				v[2][i+in_Offset] = -T((p_info->position)[2]);
			}
			return nPoint;
		}

		template<typename T>
		void loadSurface(const char* in_currentdir, OBJ2_ID in_obj, int in_i, SMaterial<T>* out_Material)
		{
			SurfaceData* s_info = (*in_obj->surfInfo)(in_obj, in_i);
			MaterialData* mat = (*in_obj->getMaterial)(in_obj, in_i);//s_info->mat;
			T cr = (mat->colr)[0];
			T cg = (mat->colr)[1];
			T cb = (mat->colr)[2];
			T diffuse = mat->difi;
			T smoothing = mat->sman;
			T reflection = mat->refl;
			T transparency = mat->tran;
			T refraction_index = mat->rind;
			T specularity = mat->spci;
			T luminosity = mat->lumi;

			T glossiness = 0.1f;
			unsigned int the_side = 0;

			if(mat->type == MAT_TYPE_LWOB)
			{
				LWOB_surf* extra_data = LWOB_unpackSurf(in_obj, s_info->size, s_info->data, NULL);
				if(extra_data->glos != NULL)
					glossiness = extra_data->glos->glossiness;
				if(extra_data->flag != NULL)
					the_side = (extra_data->flag->flags) & LWOB_SF_DOBLE_SIDE;
			}
			else
			{
				LWO2_surf* extra_data = LWO2_unpackSurf(in_obj, s_info->size, s_info->data, NULL);
				if(extra_data->glos != NULL)
					glossiness = extra_data->glos->glossiness;
				if(extra_data->side != NULL)
					the_side = (extra_data->side->sideness) & 0x02;
			}

			out_Material->color[0] = cr;
			out_Material->color[1] = cg;
			out_Material->color[2] = cb;

			out_Material->diffuse = diffuse;
			out_Material->reflection = reflection;
			out_Material->transparency = transparency;
			out_Material->specularity = specularity;
			out_Material->glossiness = glossiness;
			out_Material->luminosity = luminosity;
			out_Material->refraction_index = refraction_index;
			out_Material->smoothing_threshold = smoothing;

			out_Material->material_name = new char[strlen(mat->surf)+1];

#ifdef _WIN32
			strcpy_s(out_Material->material_name, strlen(mat->surf)+1, mat->surf);
#else
			strcpy(out_Material->material_name, mat->surf);
#endif

			if(the_side == 0)
				out_Material->type = MT_SINGLESIDED;
			else
				out_Material->type = MT_DOUBLESIDED;

			//load textures
			int nTex = mat->num_layrs;
			TextureLayer* layer = mat->tlayrs;
			//int width, height;

			initImage(0, 0, out_Material->color_tex);
			initImage(0, 0, out_Material->normal_tex);
			initImage(0, 0, out_Material->diff_tex);
			initImage(0, 0, out_Material->transparent_tex);

			for(int i=0; i<nTex; i++)
			{
				if(layer->chan == MAT_CHAN_COLR)
				{
					const char* texname = getTexName(layer->timg);

					char* the_texname = new char[strlen(in_currentdir) + strlen(texname) + 2];
					concatFilePath(the_texname, in_currentdir, texname);

					cout << "Loading Color Texture... " << the_texname << endl;	
					if(isBRFail(loadImageHdr(the_texname, out_Material->color_tex)))
					{
						cout << "Unable to read." << endl;
						finalizeImage(out_Material->color_tex);
					}
					else cout << "Success." << endl;

					delete[] the_texname;
				}
				else if(layer->chan == MAT_CHAN_BUMP)
				{
					const char* texname = getTexName(layer->timg);

					char* the_texname = new char[strlen(in_currentdir) + strlen(texname) + 2];
					concatFilePath(the_texname, in_currentdir, texname);

					cout << "Loading Normal Texture... " << the_texname << endl;
					if(isBRFail(loadImageHdr(the_texname, out_Material->normal_tex)))
					{
						cout << "Unable to read." << endl;
						finalizeImage(out_Material->normal_tex);
					}
					else cout << "Success." << endl;

					delete[] the_texname;
				}
				else if(layer->chan == MAT_CHAN_DIFF)
				{
					const char* texname = getTexName(layer->timg);

					char* the_texname = new char[strlen(in_currentdir) + strlen(texname) + 2];
					concatFilePath(the_texname, in_currentdir, texname);

					cout << "Loading Diffuse Texture... " << the_texname << endl;
					if(isBRFail(loadImageHdr(the_texname, out_Material->diff_tex)))
					{
						cout << "Unable to read." << endl;
						finalizeImage(out_Material->diff_tex);
					}
					else cout << "Success." << endl;

					delete[] the_texname;
				}
				else if(layer->chan == MAT_CHAN_TRAN)
				{
					const char* texname = getTexName(layer->timg);

					char* the_texname = new char[strlen(in_currentdir) + strlen(texname) + 2];
					concatFilePath(the_texname, in_currentdir, texname);

					cout << "Loading Transparent Texture... " << the_texname << endl;
					if(isBRFail(loadImageHdr(the_texname, out_Material->transparent_tex)))
					{
						cout << "Unable to read." << endl;
						finalizeImage(out_Material->transparent_tex);
					}
					else cout << "Success." << endl;

					delete[] the_texname;
				}

				layer = layer->next;
			}
		}

		template<typename T>
		bool loadLwo(const char* in_currentdir, const char* in_filename, STriangleSet<T>* io_Objects)
		{
			/*
			if(io_Notifier != NULL)
			{
				io_Notifier->Position = io_Notifier->MinValue;
			}
			//*/

			unsigned int the_Triangle_offset = io_Objects->nTriangles;
			unsigned int the_Vertex_offset = io_Objects->nVertices;
			unsigned int the_Material_offset = io_Objects->nMaterials;

			char* the_FileName = new char[strlen(in_currentdir) + strlen(in_filename) + 2];
			concatFilePath(the_FileName, in_currentdir, in_filename);

			OBJ2_ID obj = openLwo(the_FileName);

			delete[] the_FileName;

			if(NULL == obj)
			{
				cout << "Can not open lwo file <" << in_filename << ">" << endl;
				return false;
			}

			//load vertices
			int nPoint = (*obj->pointCount)(obj);
			for(int i=0; i<3; i++)
				io_Objects->v[i] = (T*)realloc(io_Objects->v[i], sizeof(T)*(the_Vertex_offset+nPoint));
			int nPoints = loadVertices(obj, io_Objects->v, the_Vertex_offset);
			io_Objects->nVertices = the_Vertex_offset+nPoint;

			/*
			if(io_Notifier != NULL)
			{
				io_Notifier->Position = (io_Notifier->MaxValue - io_Notifier->MinValue) * 0.3 + io_Notifier->MinValue;
			}
			//*/

			//load surfaces
			int nSurf = (*obj->surfCount)(obj);
			io_Objects->materials = (SMaterial<T>*)realloc(io_Objects->materials, sizeof(SMaterial<T>)*(the_Material_offset+nSurf));
			io_Objects->nMaterials = the_Material_offset+nSurf;

			SMaterial<T>* p_Mat = &(io_Objects->materials[the_Material_offset]);
			for(int i=0; i<nSurf; i++)
			{
				loadSurface(in_currentdir, obj, i, p_Mat);
				p_Mat++;
			}

			/*
			if(io_Notifier != NULL)
			{
				io_Notifier->Position = (io_Notifier->MaxValue - io_Notifier->MinValue) * 0.4 + io_Notifier->MinValue;
			}
			//*/

			//load polygons
			int nPolys = (*obj->polyCount)(obj);
			for(int i=0; i<3; i++)
			{
				io_Objects->t[i] = (unsigned int*)realloc(io_Objects->t[i], sizeof(unsigned int)*(the_Triangle_offset+nPolys));
				io_Objects->tn[i] = (T*)realloc(io_Objects->tn[i], sizeof(T)*(the_Triangle_offset+nPolys));
				io_Objects->tbn[i] = (T*)realloc(io_Objects->tbn[i], sizeof(T)*(the_Triangle_offset+nPolys));
				io_Objects->tcn[i] = (T*)realloc(io_Objects->tcn[i], sizeof(T)*(the_Triangle_offset+nPolys));
				io_Objects->tu[i] = (T*)realloc(io_Objects->tu[i], sizeof(T)*(the_Triangle_offset+nPolys));
				io_Objects->tv[i] = (T*)realloc(io_Objects->tv[i], sizeof(T)*(the_Triangle_offset+nPolys));
//				io_Objects->tbvm[i] = (T*)realloc(io_Objects->tbvm[i], sizeof(T)*(the_Triangle_offset+nPolys));
//				io_Objects->tbvM[i] = (T*)realloc(io_Objects->tbvM[i], sizeof(T)*(the_Triangle_offset+nPolys));
			}
			io_Objects->shaderID = (unsigned int*)realloc(io_Objects->shaderID, sizeof(unsigned int)*(the_Triangle_offset+nPolys));

			io_Objects->nTriangles = the_Triangle_offset+nPolys;

			PolygonData* pl_info;
			T uv[3][3] = {{T(0), T(0), T(0)}, {T(0), T(0), T(0)}, {T(0), T(0), T(0)}};
			T uv2[3][3] = {{T(0), T(0), T(0)}, {T(0), T(0), T(0)}, {T(0), T(0), T(0)}};

			for(int i=0; i<nPolys; i++)
			{
				pl_info = (*obj->polyInfo)(obj, i);
				int nPoints = pl_info->numPnts;
				if(nPoints != 3)
				{
					//CShowMessage::showMessage("this polygon has more than 3 vertices.");
					continue;
				}

				int surf_no = pl_info->surf_no;
				int v_id[3];

				for(int j=0; j<3; j++)
				{
					int p_no = pl_info->points[j];
					v_id[j] = p_no + the_Vertex_offset;
				}

				PolygonMap* polymap = (*obj->getPolymap)(obj, i, MAT_CHAN_BUMP, 0);
				if(polymap != NULL)
				{
					for(int j=0; j<3; j++)
					{
						MapData* map = (*obj->mapInfo)(obj, polymap->maps[j]);
						uv[0][j] = T(map->u);
						uv[1][j] = T(1.0f-map->v);
						uv[2][j] = T(map->w);
					}
				}
				else
				{
					PolygonMap* polymap = (*obj->getPolymap)(obj, i, MAT_CHAN_COLR, 0);
					if(polymap != NULL)
					{
						for(int j=0; j<3; j++)
						{
							MapData* map = (*obj->mapInfo)(obj, polymap->maps[j]);
							uv[0][j] = T(map->u);
							uv[1][j] = T(1.0f-map->v);
							uv[2][j] = T(map->w);
						}
					}
				}

				polymap = (*obj->getPolymap)(obj, i, MAT_CHAN_DIFF, 0);
				if(polymap != NULL)
				{
					for(int j=0; j<3; j++)
					{
						MapData* map = (*obj->mapInfo)(obj, polymap->maps[j]);
						uv2[0][j] = T(map->u);
						uv2[1][j] = T(1.0f-map->v);
						uv2[2][j] = T(map->w);
					}
				}

				io_Objects->t[0][i+the_Triangle_offset] = v_id[0];
				io_Objects->t[1][i+the_Triangle_offset] = v_id[2];
				io_Objects->t[2][i+the_Triangle_offset] = v_id[1];

				//printf("%d, %d, %d\n", p_TriVerts->vertId[0], p_TriVerts->vertId[1], p_TriVerts->vertId[2]);
				
				io_Objects->tu[0][i+the_Triangle_offset] = uv[0][0]; io_Objects->tv[0][i+the_Triangle_offset] = uv[1][0];
				io_Objects->tu[1][i+the_Triangle_offset] = uv[0][1]; io_Objects->tv[1][i+the_Triangle_offset] = uv[1][1];
				io_Objects->tu[2][i+the_Triangle_offset] = uv[0][2]; io_Objects->tv[2][i+the_Triangle_offset] = uv[1][2];

				/*
				p_TriShades->normalsAndUvsAndBiNormals[21]  = uv2[0].e[0]; p_TriShades->normalsAndUvsAndBiNormals[22] = uv2[0].e[1];// p_TriShades->normalsAndUvsAndBiNormal[11] = uv[0].e[2];
				p_TriShades->normalsAndUvsAndBiNormals[23] = uv2[2].e[0]; p_TriShades->normalsAndUvsAndBiNormals[24] = uv2[2].e[1];// p_TriShades->normalsAndUvsAndBiNormal[14] = uv[2].e[2];
				p_TriShades->normalsAndUvsAndBiNormals[25] = uv2[1].e[0]; p_TriShades->normalsAndUvsAndBiNormals[26] = uv2[1].e[1];// p_TriShades->normalsAndUvsAndBiNormal[17] = uv[1].e[2];
				//*/

				io_Objects->shaderID[i+the_Triangle_offset] = surf_no + the_Material_offset;
			}

			OBJ2_close(&obj);
			printf("Loaded. (%d Surfs, %d Polys, %d Points)\n", nSurf, nPolys, nPoints);

			/*
			if(io_Notifier != NULL)
			{
				io_Notifier->Position = (io_Notifier->MaxValue - io_Notifier->MinValue) * 0.9 + io_Notifier->MinValue;
			}
			//*/

			return true;
		}
	};
};

#endif
