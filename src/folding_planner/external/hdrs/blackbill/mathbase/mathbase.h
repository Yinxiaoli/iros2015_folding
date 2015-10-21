#ifndef __MATH_BASE_H__
#define __MATH_BASE_H__

#include <cmath>
using namespace std;

namespace _BlackBill
{
	namespace _MathBase
	{
		template<typename T> T BBPI() { return T(3.14159265358979323846264338327950288); }

		template<typename T>
		inline T cross4(const T in_a, const T in_b, const T in_c, const T in_d)
		{
			return in_a * in_b - in_c * in_d;
		}

		template<typename T>
		inline T dot(const T in_lx, const T in_ly, const T in_lz, const T in_rx, const T in_ry, const T in_rz)
		{
			return in_lx * in_rx + in_ly * in_ry + in_lz * in_rz;
		}

		template<typename T>
		inline T dot(const T in_l[3], const T in_r[3])
		{
			return in_l[0] * in_r[0] + in_l[1] * in_r[1] + in_l[2] * in_r[2];
		}

		template<typename T>
		inline T length(const T in_lx, const T in_ly, const T in_lz)
		{
			return sqrt(in_lx * in_lx + in_ly * in_ly + in_lz * in_lz);
		}

		template<typename T>
		inline T length(const T in_l[3])
		{
			return sqrt(in_l[0] * in_l[0] + in_l[1] * in_l[1] + in_l[2] * in_l[2]);
		}

		template<typename T>
		inline T length2(const T in_lx, const T in_ly, const T in_lz)
		{
			return in_lx * in_lx + in_ly * in_ly + in_lz * in_lz;
		}

		template<typename T>
		inline T length2(const T in_l[3])
		{
			return in_l[0] * in_l[0] + in_l[1] * in_l[1] + in_l[2] * in_l[2];
		}

		template<typename T>
		inline void rotateVector(T* out_x, T* out_y, T* out_z, const T in_px, const T in_py, const T in_pz, const T in_axis_x, const T in_axis_y, const T in_axis_z, const T in_angle_rad)
		{
			const T inv_len_axis = T(1.0) / length<T>(in_axis_x, in_axis_y, in_axis_z);
			const T axis_x = in_axis_x * inv_len_axis;
			const T axis_y = in_axis_y * inv_len_axis;
			const T axis_z = in_axis_z * inv_len_axis;

			const T cos_a = cos(in_angle_rad*T(0.5));
			const T sin_a = sin(in_angle_rad*T(0.5));
			const T m_sin_a = -sin_a;

			//(r * p * q).v
			//p = (0, px, py, pz)
			//q = (cos_a, axis_x * sin_a, axis_y * sin_a, axis_z * sin_a)
			//r = (cos_a, axis_x * m_sin_a, axis_y * m_sin_a, axis_z * m_sin_a)

			const T r_p_t = -dot<T>(axis_x * m_sin_a, axis_y * m_sin_a, axis_z * m_sin_a, in_px, in_py, in_pz);
			const T r_p_x = in_px * cos_a + cross4<T>(in_py, axis_z * m_sin_a, in_pz, axis_y * m_sin_a);
			const T r_p_y = in_py * cos_a + cross4<T>(in_pz, axis_x * m_sin_a, in_px, axis_z * m_sin_a);
			const T r_p_z = in_pz * cos_a + cross4<T>(in_px, axis_y * m_sin_a, in_py, axis_x * m_sin_a);

			*out_x = axis_x * sin_a * r_p_t + r_p_x * cos_a + cross4<T>(axis_y * sin_a, r_p_z, axis_z * sin_a, r_p_y);
			*out_y = axis_y * sin_a * r_p_t + r_p_y * cos_a + cross4<T>(axis_z * sin_a, r_p_x, axis_x * sin_a, r_p_z);
			*out_z = axis_z * sin_a * r_p_t + r_p_z * cos_a + cross4<T>(axis_x * sin_a, r_p_y, axis_y * sin_a, r_p_x);
		}

		template<typename T>
		inline void genBinormal(const T in_Normal[3], T out_BiNormal[3])
		{
			T _binormal[3];
			if((in_Normal[0] <= in_Normal[1]) && (in_Normal[0] <= in_Normal[2]))
			{
				_binormal[0] = 1.0; _binormal[1] = 0.0; _binormal[2] = 0.0;
			}
			else if((in_Normal[1] <= in_Normal[0]) && (in_Normal[1] <= in_Normal[2]))
			{
				_binormal[0] = 0.0; _binormal[1] = 1.0; _binormal[2] = 0.0;
			}
			else
			{
				_binormal[0] = 0.0; _binormal[1] = 0.0; _binormal[2] = 1.0;
			}

			T c = dot(in_Normal, _binormal);
			_binormal[0] -= c * in_Normal[0]; _binormal[1] -= c * in_Normal[1]; _binormal[2] -= c * in_Normal[2];

			T inv_len = 1.0 / length(_binormal);
			out_BiNormal[0] = _binormal[0] * inv_len; out_BiNormal[1] = _binormal[1] * inv_len; out_BiNormal[2] = _binormal[2] * inv_len;
		}

		template<typename T>
		inline void genBinormal(const T in_Normal[3], T out_BiNormal1[3], T out_BiNormal2[3])
		{
			T _binormal[3];
			if((in_Normal[0] <= in_Normal[1]) && (in_Normal[0] <= in_Normal[2]))
			{
				_binormal[0] = 1.0; _binormal[1] = 0.0; _binormal[2] = 0.0;
			}
			else if((in_Normal[1] <= in_Normal[0]) && (in_Normal[1] <= in_Normal[2]))
			{
				_binormal[0] = 0.0; _binormal[1] = 1.0; _binormal[2] = 0.0;
			}
			else
			{
				_binormal[0] = 0.0; _binormal[1] = 0.0; _binormal[2] = 1.0;
			}

			T c = dot(in_Normal, _binormal);
			_binormal[0] -= c * in_Normal[0]; _binormal[1] -= c * in_Normal[1]; _binormal[2] -= c * in_Normal[2];

			T inv_len = 1.0 / length(_binormal);
			out_BiNormal1[0] = _binormal[0] * inv_len; out_BiNormal1[1] = _binormal[1] * inv_len; out_BiNormal1[2] = _binormal[2] * inv_len;

			out_BiNormal2[0] = cross4(out_BiNormal1[1], in_Normal[2], out_BiNormal1[2], in_Normal[1]);
			out_BiNormal2[1] = cross4(out_BiNormal1[2], in_Normal[0], out_BiNormal1[0], in_Normal[2]);
			out_BiNormal2[2] = cross4(out_BiNormal1[0], in_Normal[1], out_BiNormal1[1], in_Normal[0]);
		}
	};
};

#endif
