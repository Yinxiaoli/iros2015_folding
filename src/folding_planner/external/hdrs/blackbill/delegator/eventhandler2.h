#ifndef __EVENT__
#define __EVENT__

//2003/09/10
//modified: 2012/05/27

#include "delegator.h"
#include "delegatorchain.h"

//return type is void for events

template <typename _A1 = NullType, typename _A2 = NullType, typename _A3 = NullType, typename _A4 = NullType, typename _A5 = NullType> class event;

template <> class event<>
{
public:
	event<>& operator += (const delegator<void>& in_Data)
	{
		del_chain += in_Data;
		return *this;
	}

	void operator()()
	{
		if(!del_chain.isNull())
			del_chain();
	}

protected:
	delegator_chain<void> del_chain;
};


template <typename _A1> class event<_A1>
{
public:
	event<_A1>& operator += (const delegator<void, _A1>& in_Data)
	{
		del_chain += in_Data;
		return *this;
	}

	void operator()(_A1 _arg1)
	{
		if(!del_chain.isNull())
			del_chain(_arg1);
	}

protected:
	delegator_chain<void, _A1> del_chain;
};


template <typename _A1, typename _A2> class event<_A1, _A2>
{
public:
	event<_A1, _A2>& operator += (const delegator<void, _A1, _A2>& in_Data)
	{
		del_chain += in_Data;
		return *this;
	}

	void operator()(_A1 _arg1, _A2 _arg2)
	{
		if(!del_chain.isNull())
			del_chain(_arg1, _arg2);
	}

protected:
	delegator_chain<void, _A1, _A2> del_chain;
};


template <typename _A1, typename _A2, typename _A3> class event<_A1, _A2, _A3>
{
public:
	event<_A1, _A2, _A3>& operator += (const delegator<void, _A1, _A2, _A3>& in_Data)
	{
		del_chain += in_Data;
		return *this;
	}

	void operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3)
	{
		if(!del_chain.isNull())
			del_chain(_arg1, _arg2, _arg3);
	}

protected:
	delegator_chain<void, _A1, _A2, _A3> del_chain;
};


template <typename _A1, typename _A2, typename _A3, typename _A4> class event<_A1, _A2, _A3, _A4>
{
public:
	event<_A1, _A2, _A3, _A4>& operator += (const delegator<void, _A1, _A2, _A3, _A4>& in_Data)
	{
		del_chain += in_Data;
		return *this;
	}

	void operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3)
	{
		if(!del_chain.isNull())
			del_chain(_arg1, _arg2, _arg3, _arg4);
	}

protected:
	delegator_chain<void, _A1, _A2, _A3, _A4> del_chain;
};


#endif