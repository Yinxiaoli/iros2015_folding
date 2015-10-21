#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <blackbill/mathbase/mathbase.h>
using namespace _BlackBill::_MathBase;
#include "../ray/ray.h"
using namespace _BlueTail::_Ray;

#include <stdint.h>

namespace _BlueTail{
	namespace _Camera{

		template<typename T>
		struct SInitialCameraSettings
		{
			T eyePoint[3];
			T lookAtLocation[3];
			T upVector[3]; //not necessarily perpendicular to the internal look-at vector.
			T irisF;
			T focalLength;
		};

		template<typename T>
		struct SCameraInternal
		{
			T zVector[3];

			T focalLength;
			T focusLength;
			T t_i;

			int32_t width;
			int32_t height;
			
			T irisF;
		};

		template<typename T>
		struct SCamera
		{
			T eyePoint[3];
			T xVector[3];
			T yVector[3];
			T planeCenter[3];

			T screenWidth;
			T screenHeight;

			T invWidth;
			T invHeight;

			//Following two data fields are not needed if depth of field is not in consideration
			T lensRadius;
			T focusPI;

			SCameraInternal<T> internalData; //This data is not used (even read) during rendering and is modified only when the camera condition is changed.
		};

		template<typename T>
		void setPlane(SCamera<T>& io_Camera)
		{
			io_Camera.planeCenter[0] = io_Camera.internalData.t_i * io_Camera.internalData.zVector[0];
			io_Camera.planeCenter[1] = io_Camera.internalData.t_i * io_Camera.internalData.zVector[1];
			io_Camera.planeCenter[2] = io_Camera.internalData.t_i * io_Camera.internalData.zVector[2];
		}

		template<typename T>
		void recalcLensData(SCamera<T>& io_Camera)
		{
			io_Camera.lensRadius = io_Camera.internalData.focalLength / (T(2.0) * io_Camera.internalData.irisF);
			io_Camera.internalData.t_i = io_Camera.internalData.focalLength;// * io_Camera.internalData.focusLength / (io_Camera.internalData.focusLength - io_Camera.internalData.focalLength);
			io_Camera.focusPI = io_Camera.internalData.focusLength / io_Camera.internalData.t_i;
			setPlane<T>(io_Camera);
		}

		template<typename T>
		void setEyePoint(const T in_EyePoint[3], SCamera<T>& io_Camera)
		{
			io_Camera.eyePoint[0] = in_EyePoint[0];
			io_Camera.eyePoint[1] = in_EyePoint[1];
			io_Camera.eyePoint[2] = in_EyePoint[2];
		}

		template<typename T>
		void setFocalLength(const T in_FocalLength, SCamera<T>& io_Camera)
		{
			io_Camera.internalData.focalLength = in_FocalLength;
			recalcLensData<T>(io_Camera);
		}

		template<typename T>
		void setIrisF(const T in_IrisF, SCamera<T>& io_Camera)
		{
			io_Camera.internalData.irisF = in_IrisF;
			recalcLensData<T>(io_Camera);
		}

		template<typename T>
		void lookAt(const T in_LookAt[3], const T in_Up[3], SCamera<T>& io_Camera)
		{
			const T v_x = in_LookAt[0] - io_Camera.eyePoint[0];
			const T v_y = in_LookAt[1] - io_Camera.eyePoint[1];
			const T v_z = in_LookAt[2] - io_Camera.eyePoint[2];
			const T len = length<T>(v_x, v_y, v_z);

			io_Camera.internalData.focusLength = len;

			io_Camera.internalData.zVector[0] = -v_x / len;
			io_Camera.internalData.zVector[1] = -v_y / len;
			io_Camera.internalData.zVector[2] = -v_z / len;
			
			//const T dot_up_z = dot<T>(in_Up[0], in_Up[1], in_Up[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2]);
			const T dot_up_z = dot<T>(in_Up, io_Camera.internalData.zVector);
			const T _yx = in_Up[0] - dot_up_z * io_Camera.internalData.zVector[0];
			const T _yy = in_Up[1] - dot_up_z * io_Camera.internalData.zVector[1];
			const T _yz = in_Up[2] - dot_up_z * io_Camera.internalData.zVector[2];
			const T inv_len_y = T(1.0) / length<T>(_yx, _yy, _yz);
			
			io_Camera.yVector[0] = _yx*inv_len_y;
			io_Camera.yVector[1] = _yy*inv_len_y;
			io_Camera.yVector[2] = _yz*inv_len_y;

			io_Camera.xVector[0] = cross4<T>(io_Camera.yVector[1], io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[1], io_Camera.yVector[2]);
			io_Camera.xVector[1] = cross4<T>(io_Camera.yVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[2], io_Camera.yVector[0]);
			io_Camera.xVector[2] = cross4<T>(io_Camera.yVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[0], io_Camera.yVector[1]);

			recalcLensData<T>(io_Camera);
		}

		template<typename T>
		void setViewportSize(const int32_t in_Width, const int32_t in_Height, SCamera<T>& io_Camera)
		{
			io_Camera.internalData.width = in_Width;
			io_Camera.invWidth = T(1.0) / in_Width;
			io_Camera.internalData.height = in_Height;
			io_Camera.invHeight = T(1.0) / in_Height;

			//sw <= 0.036, sh <= 0.024‚É‚·‚éB
			if(in_Width >= T(1.5)*in_Height)
			{
				io_Camera.screenWidth = T(0.036);
				io_Camera.screenHeight = in_Height * T(0.036) * io_Camera.invWidth;
			}
			else
			{
				io_Camera.screenWidth = in_Width * T(0.024) * io_Camera.invHeight;
				io_Camera.screenHeight = T(0.024);
			}
		}

		template<typename T>
		void prepareCameraWithCameraSettings(const int32_t in_Width, const int32_t in_Height, const SInitialCameraSettings<T>& in_Settings, SCamera<T>& io_Camera)
		{
			io_Camera.internalData.focalLength = 100.0;
			io_Camera.internalData.focusLength = 1.0;
			io_Camera.internalData.irisF = 16.0;

			setEyePoint<T>(in_Settings.eyePoint, io_Camera);
			lookAt<T>(in_Settings.lookAtLocation, in_Settings.upVector, io_Camera);
			setIrisF(in_Settings.irisF, io_Camera);
			setFocalLength(in_Settings.focalLength, io_Camera);
			setViewportSize(in_Width, in_Height, io_Camera);
		}

		template<typename T>
		void screenView(const SCamera<T>& in_Camera, const int32_t in_x, const int32_t in_y, SRay<T>& out_Ray, const T in_RandomSample[2])
		{
			const T s = - ((in_x + in_RandomSample[0]) * in_Camera.invWidth - T(0.5)) * in_Camera.screenWidth;
			const T t = ((in_y + in_RandomSample[1]) * in_Camera.invHeight - T(0.5)) * in_Camera.screenHeight;

			const T dx = -(in_Camera.xVector[0] * s + in_Camera.yVector[0] * t + in_Camera.planeCenter[0]);
			const T dy = -(in_Camera.xVector[1] * s + in_Camera.yVector[1] * t + in_Camera.planeCenter[1]);
			const T dz = -(in_Camera.xVector[2] * s + in_Camera.yVector[2] * t + in_Camera.planeCenter[2]);

			const T inv_len = T(1.0) / length(dx, dy, dz);

			out_Ray.o[0] = in_Camera.eyePoint[0];
			out_Ray.o[1] = in_Camera.eyePoint[1];
			out_Ray.o[2] = in_Camera.eyePoint[2];
			out_Ray.d[0] = dx * inv_len;
			out_Ray.d[1] = dy * inv_len;
			out_Ray.d[2] = dz * inv_len;
			out_Ray.tMin = T(0.0);
			out_Ray.tMax = _BT_RAY_FAR<T>();
		}

		template<typename T>
		void screenViewFishEye(const SCamera<T>& in_Camera, const int32_t in_x, const int32_t in_y, SRay<T>& out_Ray, const T in_RandomSample[2])
		{
			out_Ray.o[0] = in_Camera.eyePoint[0];
			out_Ray.o[1] = in_Camera.eyePoint[1];
			out_Ray.o[2] = in_Camera.eyePoint[2];

			const T x = T(in_x) * in_Camera.invWidth * T(2.0) - T(1.0);
			const T y = T(in_y) * in_Camera.invWidth * T(2.0) - T(1.0);

			const T r = sqrt(x*x + y*y);
			const T phi = atan2(y, x);

			if(r >= T(1.0))
			{
				out_Ray.d[0] = T(0.0);
				out_Ray.d[1] = T(-1.0);
				out_Ray.d[2] = T(0.0);
			}
			else
			{
				const T theta = r * T(0.5) * BBPI<T>();

				out_Ray.d[0] = sin(theta)*cos(phi);
				out_Ray.d[1] = cos(theta);
				out_Ray.d[2] = sin(theta)*sin(phi);
			}

			out_Ray.tMin = T(0.0);
			out_Ray.tMax = _BT_RAY_FAR<T>();
		}

		template<typename T>
		void screenViewDF(const SCamera<T>& in_Camera, const int32_t in_x, const int32_t in_y, SRay<T>& out_Ray, const T in_RandomSample[2])
		{
			const T s = - ((in_x + in_RandomSample[0]) * in_Camera.invWidth - T(0.5)) * in_Camera.screenWidth;
			const T t = ((in_y + in_RandomSample[1]) * in_Camera.invHeight - T(0.5)) * in_Camera.screenHeight;

			const T _dx = -(in_Camera.xVector[0] * s + in_Camera.yVector[0] * t + in_Camera.planeCenter[0]);
			const T _dy = -(in_Camera.xVector[1] * s + in_Camera.yVector[1] * t + in_Camera.planeCenter[1]);
			const T _dz = -(in_Camera.xVector[2] * s + in_Camera.yVector[2] * t + in_Camera.planeCenter[2]);

			const T dx_lens = in_Camera.lensRadius * in_RandomSample[0];
			const T dy_lens = in_Camera.lensRadius * in_RandomSample[1];

			const T dx = _dx * in_Camera.focusPI - (dx_lens * in_Camera.xVector[0] + dy_lens * in_Camera.yVector[0]);
			const T dy = _dy * in_Camera.focusPI - (dx_lens * in_Camera.xVector[1] + dy_lens * in_Camera.yVector[1]);
			const T dz = _dz * in_Camera.focusPI - (dx_lens * in_Camera.xVector[2] + dy_lens * in_Camera.yVector[2]);

			out_Ray.o[0] = in_Camera.eyePoint[0] + (dx_lens * in_Camera.xVector[0] + dy_lens * in_Camera.yVector[0]);
			out_Ray.o[1] = in_Camera.eyePoint[1] + (dx_lens * in_Camera.xVector[1] + dy_lens * in_Camera.yVector[1]);
			out_Ray.o[2] = in_Camera.eyePoint[2] + (dx_lens * in_Camera.xVector[2] + dy_lens * in_Camera.yVector[2]);

			const T inv_len = T(1.0) / length<T>(dx, dy, dz);
			out_Ray.d[0] = dx * inv_len;
			out_Ray.d[1] = dy * inv_len;
			out_Ray.d[2] = dz * inv_len;

			out_Ray.tMin = T(0.0);
			out_Ray.tMax = _BT_RAY_FAR<T>();
		}

		template<typename T>
		void getLookAt(T out_V[3], const SCamera<T>& in_Camera)
		{
			out_V[0] = in_Camera.eyePoint[0] - in_Camera.internalData.zVector[0] * in_Camera.internalData.focusLength;
			out_V[1] = in_Camera.eyePoint[1] - in_Camera.internalData.zVector[1] * in_Camera.internalData.focusLength;
			out_V[2] = in_Camera.eyePoint[2] - in_Camera.internalData.zVector[2] * in_Camera.internalData.focusLength;
		}

		template<typename T>
		void moveUpCameraCoord(const T in_Dist, SCamera<T>& io_Camera)
		{
			io_Camera->eyePoint[0] += in_Dist * io_Camera.yVector[0];
			io_Camera->eyePoint[1] += in_Dist * io_Camera.yVector[1];
			io_Camera->eyePoint[2] += in_Dist * io_Camera.yVector[2];
		}

		template<typename T>
		void moveUpGlobalCoord(const T in_Dist, SCamera<T>& io_Camera)
		{
			io_Camera->eyePoint[1] += in_Dist;
		}

		template<typename T>
		void moveFrontFixUp(const T in_Dist, SCamera<T>& io_Camera)
		{
			const T _x = io_Camera.internalData.zVector[0];
			const T _y = T(0.0);
			const T _z = io_Camera.internalData.zVector[2];

			const T _len2 = length2<T>(_x, _y, _z);
			if(_len2 < T(0.0000001))
				return;

			const T inv_len = T(1.0) / sqrt(_len2);
			const T dx = -_x * inv_len;
			const T dy = -_y * inv_len;
			const T dz = -_z * inv_len;

			io_Camera.eyePoint[0] += dx * in_Dist;
			io_Camera.eyePoint[1] += dy * in_Dist;
			io_Camera.eyePoint[2] += dz * in_Dist;
		}

		template<typename T>
		void moveFrontFixUpFixLookAt(const T in_Dist, SCamera<T>& io_Camera)
		{
			const T _x = io_Camera.internalData.zVector[0];
			const T _y = T(0.0);
			const T _z = io_Camera.internalData.zVector[2];

			const T _len2 = length2<T>(_x, _y, _z);
			if(_len2 < T(0.0000001))
				return;

			const T inv_len = T(1.0) / sqrt(_len2);
			const T dx = -_x * inv_len;
			const T dy = -_y * inv_len;
			const T dz = -_z * inv_len;

			const T newFocusLength = max(io_Camera.internalData.focalLength, io_Camera.internalData.focusLength-in_Dist);
			const T the_Dist = io_Camera.internalData.focusLength - newFocusLength;

			io_Camera.eyePoint[0] += dx * the_Dist;
			io_Camera.eyePoint[1] += dy * the_Dist;
			io_Camera.eyePoint[2] += dz * the_Dist;

			io_Camera.internalData.focusLength = newFocusLength;
		}

		template<typename T>
		void rotateFixEyePointCameraCoord(T in_HorizontalAngle, T in_VerticalAngle, T in_RoundAngle, SCamera<T>& io_Camera)
		{
			//rotate around y-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1],io_Camera.xVector[2], io_Camera.yVector[0], io_Camera.yVector[1],io_Camera.yVector[2], in_HorizontalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], in_HorizontalAngle);

			//rotate around x-axis
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);

			//rotate around z-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);

			setPlane<T>(io_Camera);
		}

		template<typename T>
		void rotateFixEyePointCameraCoordFixUp(T in_HorizontalAngle, T in_VerticalAngle, T in_RoundAngle, SCamera<T>& io_Camera)
		{
			//rotate around y-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);

			//rotate around x-axis
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);

			//rotate around z-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);

			setPlane<T>(io_Camera);
		}

		template<typename T>
		void rotateFixEyePointGlobalCoord(T in_HorizontalAngle, T in_VerticalAngle, T in_RoundAngle, SCamera<T>& io_Camera)
		{
			//rotate around y-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);

			//rotate around x-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 1.0f, 0.0f, 0.0f, in_VerticalAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 1.0f, 0.0f, 0.0f, in_VerticalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 1.0f, 0.0f, 0.0f, in_VerticalAngle);

			//rotate around z-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 0.0f, 0.0f, 1.0f, -in_RoundAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 0.0f, 0.0f, 1.0f, -in_RoundAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 0.0f, 0.0f, 1.0f, -in_RoundAngle);

			setPlane<T>(io_Camera);
		}

		template<typename T>
		void rotateFixLookAtCameraCoord(T in_HorizontalAngle, T in_VerticalAngle, T in_RoundAngle, SCamera<T>& io_Camera)
		{
			const T fixed_look_at_x = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0] + io_Camera.eyePoint[0];
			const T fixed_look_at_y = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1] + io_Camera.eyePoint[1];
			const T fixed_look_at_z = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2] + io_Camera.eyePoint[2];

			T arm_x = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0];
			T arm_y = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1];
			T arm_z = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2];

			//rotate around y-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], in_HorizontalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], in_HorizontalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], in_HorizontalAngle);

			//rotate around x-axis
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);

			//rotate around z-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);

			io_Camera.eyePoint[0] = fixed_look_at_x + arm_x;
			io_Camera.eyePoint[1] = fixed_look_at_y + arm_y;
			io_Camera.eyePoint[2] = fixed_look_at_z + arm_z;

			setPlane<T>(io_Camera);
		}

		template<typename T>
		void rotateFixLookAtCameraCoordFixUp(T in_HorizontalAngle, T in_VerticalAngle, T in_RoundAngle, SCamera<T>& io_Camera)
		{
			const T fixed_look_at_x = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0] + io_Camera.eyePoint[0];
			const T fixed_look_at_y = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1] + io_Camera.eyePoint[1];
			const T fixed_look_at_z = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2] + io_Camera.eyePoint[2];

			T arm_x = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0];
			T arm_y = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1];
			T arm_z = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2];

			//rotate around y-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, 0.0f, 1.0f, 0.0f, in_HorizontalAngle);

			//rotate around x-axis
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);

			//rotate around z-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);

			io_Camera.eyePoint[0] = fixed_look_at_x + arm_x;
			io_Camera.eyePoint[1] = fixed_look_at_y + arm_y;
			io_Camera.eyePoint[2] = fixed_look_at_z + arm_z;

			setPlane<T>(io_Camera);
		}

		template<typename T>
		void rotateFixLookAtCameraCoordFixUp(T in_HorizontalAngle, T in_VerticalAngle, T in_RoundAngle, SCamera<T>& io_Camera, T in_Up[3])
		{
			const T fixed_look_at_x = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0] + io_Camera.eyePoint[0];
			const T fixed_look_at_y = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1] + io_Camera.eyePoint[1];
			const T fixed_look_at_z = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2] + io_Camera.eyePoint[2];

			T arm_x = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0];
			T arm_y = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1];
			T arm_z = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2];

			//rotate around y-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_Up[0], in_Up[1], in_Up[2], in_HorizontalAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], in_Up[0], in_Up[1], in_Up[2], in_HorizontalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], in_Up[0], in_Up[1], in_Up[2], in_HorizontalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, in_Up[0], in_Up[1], in_Up[2], in_HorizontalAngle);

			//rotate around x-axis
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], in_VerticalAngle);

			//rotate around z-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], -in_RoundAngle);

			io_Camera.eyePoint[0] = fixed_look_at_x + arm_x;
			io_Camera.eyePoint[1] = fixed_look_at_y + arm_y;
			io_Camera.eyePoint[2] = fixed_look_at_z + arm_z;

			setPlane<T>(io_Camera);
		}

		template<typename T>
		void rotateFixLookAtGlobalCoord(T in_HorizontalAngle, T in_VerticalAngle, T in_RoundAngle, SCamera<T>& io_Camera)
		{
			const T fixed_look_at_x = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0] + io_Camera.eyePoint[0];
			const T fixed_look_at_y = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1] + io_Camera.eyePoint[1];
			const T fixed_look_at_z = -io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2] + io_Camera.eyePoint[2];

			T arm_x = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[0];
			T arm_y = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[1];
			T arm_z = io_Camera.internalData.focusLength * io_Camera.internalData.zVector[2];

			//rotate around y-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 0.0f, 1.0f, 0.0f, in_HorizontalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, 0.0f, 1.0f, 0.0f, in_HorizontalAngle);

			//rotate around x-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 1.0f, 0.0f, 0.0f, in_VerticalAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 1.0f, 0.0f, 0.0f, in_VerticalAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 1.0f, 0.0f, 0.0f, in_VerticalAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, 1.0f, 0.0f, 0.0f, in_VerticalAngle);

			//rotate around z-axis
			rotateVector<T>(&io_Camera.xVector[0], &io_Camera.xVector[1], &io_Camera.xVector[2], io_Camera.xVector[0], io_Camera.xVector[1], io_Camera.xVector[2], 0.0f, 0.0f, 1.0f, -in_RoundAngle);
			rotateVector<T>(&io_Camera.yVector[0], &io_Camera.yVector[1], &io_Camera.yVector[2], io_Camera.yVector[0], io_Camera.yVector[1], io_Camera.yVector[2], 0.0f, 0.0f, 1.0f, -in_RoundAngle);
			rotateVector<T>(&io_Camera.internalData.zVector[0], &io_Camera.internalData.zVector[1], &io_Camera.internalData.zVector[2], io_Camera.internalData.zVector[0], io_Camera.internalData.zVector[1], io_Camera.internalData.zVector[2], 0.0f, 0.0f, 1.0f, -in_RoundAngle);
			rotateVector<T>(&arm_x, &arm_y, &arm_z, arm_x, arm_y, arm_z, 0.0f, 0.0f, 1.0f, -in_RoundAngle);

			io_Camera.eyePoint[0] = fixed_look_at_x + arm_x;
			io_Camera.eyePoint[1] = fixed_look_at_y + arm_y;
			io_Camera.eyePoint[2] = fixed_look_at_z + arm_z;

			setPlane<T>(io_Camera);
		}

		
		//void lookAtBVCenter(const SMinMaxBV& in_BV, SCameraInternal* io_internal, SCamera* io_camera);
	};
};

#endif

