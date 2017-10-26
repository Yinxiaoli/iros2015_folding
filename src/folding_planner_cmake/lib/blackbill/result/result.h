#ifndef __RESULT_H__
#define __RESULT_H__

namespace _BlackBill
{
	namespace _Result
	{
		enum BBRESULT
		{
			BR_SUCCESS = 0,
			BR_FAIL = 1
		};

		inline bool isBRSuccess(BBRESULT in_Res)
		{
			return in_Res == BR_SUCCESS;
		}

		inline bool isBRFail(BBRESULT in_Res)
		{
			return in_Res != BR_SUCCESS;
		}
	};
};

#endif
