#ifndef __DELEGATOR_CHAIN__
#define __DELEGATOR_CHAIN__

#include "delegator.h"
#include <vector>
using namespace std;

template <typename _R, typename _A1 = NullType, typename _A2 = NullType, typename _A3 = NullType, typename _A4 = NullType, typename _A5 = NullType> class delegator_chain;

template <typename _R> class delegator_chain<_R>
{
public:
	delegator_chain() {}

	virtual ~delegator_chain() {}

	delegator_chain<_R>& operator+=(delegator<_R>& in_delegator)
	{
		m_List.push_back(in_delegator);
		return *this;
	}

	template <typename _Tfunc> const delegator_chain<_R>& operator+=(_Tfunc _funcobj)
	{
		delegator<_R> _a = delegator<_R>(_funcobj);
		m_List.push_back(_a);
		return *this;
	}

	//run in reverse order //2012/05/27
	_R operator()()
	{
		delegator<_R> the_delegator;
		
		for(int i=int(m_List.size()-1); i>=0; i--)
		{
			the_delegator = m_List[i];
			if(i != 0)
				the_delegator();
		}

		return the_delegator();
	}

	long isNull()
	{
		return m_List.empty();
	}

private:
	vector<delegator<_R> > m_List;
};


template <typename _R, typename _A1> class delegator_chain<_R, _A1>
{
public:
	delegator_chain() {}

	virtual ~delegator_chain() {}

	delegator_chain<_R, _A1>& operator+=(delegator<_R, _A1>& in_delegator)
	{
		m_List.push_back(in_delegator);
		return *this;
	}

	template <typename _Tfunc> delegator_chain<_R, _A1>& operator+=(_Tfunc _funcobj)
	{
		delegator<_R, _A1> _a = delegator<_R, _A1>(_funcobj);
		m_List.push_back(_a);
		return *this;
	}

	//run in reverse order //2012/05/27
	_R operator()(_A1 _arg1)
	{
		delegator<_R, _A1> the_delegator;
		for(int i=m_List.size(); i>=0; i--)
		{
			the_delegator = m_List[i];
			if(i != 0)
				the_delegator(_arg1);
		}
		return the_delegator(_arg1);
	}

	long isNull()
	{
		return m_List.empty();
	}

private:
	vector<delegator<_R, _A1> > m_List;
};


template <typename _R, typename _A1, typename _A2> class delegator_chain<_R, _A1, _A2>
{
public:
	delegator_chain() {}

	virtual ~delegator_chain() {}

	delegator_chain<_R, _A1, _A2>& operator+=(delegator<_R, _A1, _A2>& in_delegator)
	{
		m_List.push_back(in_delegator);
		return *this;
	}

	template <typename _Tfunc> delegator_chain<_R, _A1, _A2>& operator+=(_Tfunc _funcobj)
	{
		delegator<_R, _A1, _A2> _a = delegator<_R, _A1, _A2>(_funcobj);
		m_List.push_back(_a);
		return *this;
	}

	//run in reverse order //2012/05/27
	_R operator()(_A1 _arg1, _A2 _arg2)
	{
		delegator<_R, _A1, _A2> the_delegator;
		for(unsigned int i=0; i<m_List.size(); i++)
		{
			the_delegator = m_List[i];
			if(i != m_List.size()-1)
				the_delegator(_arg1, _arg2);
		}
		return the_delegator(_arg1, _arg2);
	}

	long isNull()
	{
		return m_List.empty();
	}

private:
	vector<delegator<_R, _A1, _A2> > m_List;
};


template <typename _R, typename _A1, typename _A2, typename _A3> class delegator_chain<_R, _A1, _A2, _A3>
{
public:
	delegator_chain() {}

	virtual ~delegator_chain() {}

	delegator_chain<_R, _A1, _A2, _A3>& operator+=(delegator<_R, _A1, _A2, _A3>& in_delegator)
	{
		m_List.add(in_delegator);
		return *this;
	}

	template <typename _Tfunc> delegator_chain<_R, _A1, _A2, _A3>& operator+=(_Tfunc _funcobj)
	{
		delegator<_R, _A1, _A2, _A3> _a = delegator<_R, _A1, _A2, _A3>(_funcobj);
		m_List.push_back(_a);
		return *this;
	}

	//run in reverse order //2012/05/27
	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3)
	{
		delegator<_R, _A1, _A2, _A3> the_delegator;
		for(unsigned int i=0; i<m_List.size(); i++)
		{
			the_delegator = m_List[i];
			if(i != m_List.size()-1)
				the_delegator(_arg1, _arg2, _arg3);
		}
		return the_delegator(_arg1, _arg2, _arg3);
	}

	long isNull()
	{
		return m_List.empty();
	}

private:
	vector<delegator<_R, _A1, _A2, _A3> > m_List;
};


template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4> class delegator_chain<_R, _A1, _A2, _A3, _A4>
{
public:
	delegator_chain() {}

	virtual ~delegator_chain() {}

	delegator_chain<_R, _A1, _A2, _A3, _A4>& operator+=(delegator<_R, _A1, _A2, _A3, _A4>& in_delegator)
	{
		m_List.add(in_delegator);
		return *this;
	}

	template <typename _Tfunc> delegator_chain<_R, _A1, _A2, _A3, _A4>& operator+=(_Tfunc _funcobj)
	{
		delegator<_R, _A1, _A2, _A3, _A4> _a = delegator<_R, _A1, _A2, _A3, _A4>(_funcobj);
		m_List.push_back(_a);
		return *this;
	}

	//run in reverse order //2012/05/27
	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4)
	{
		delegator<_R, _A1, _A2, _A3, _A4> the_delegator;
		for(unsigned int i=0; i<m_List.size(); i++)
		{
			the_delegator = m_List[i];
			if(i != m_List.size()-1)
				the_delegator(_arg1, _arg2, _arg3, _arg4);
		}
		return the_delegator(_arg1, _arg2, _arg3, _arg4);
	}

	long isNull()
	{
		return m_List.empty();
	}

private:
	vector<delegator<_R, _A1, _A2, _A3, _A4> > m_List;
};


#endif