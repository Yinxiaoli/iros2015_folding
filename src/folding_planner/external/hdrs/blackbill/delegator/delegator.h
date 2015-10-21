#ifndef __DELEGATOR__
#define __DELEGATOR__

#ifdef _MSC_VER
#pragma pack(push, 8)
#endif

/* copied from boost/.../is_same.hpp */

/*
template <typename T1> struct _is_same
{
	template<typename T2>    struct part     { enum { value = false }; };
	template<>               struct part<T1> { enum { value = true }; };
};

template <typename T1, typename T2> struct is_same
{
	enum { value = _is_same<T1>::template part<T2>::value };
};

*/

/* copy end */

union any_pointer
{
	void* ptr_obj;
	const void* ptr_obj_const;
	void (*ptr_func)();
};

struct NullType {};

/*
template <typename T> struct is_NullType
{
	enum { value = is_same<T, NullType>::value };
};
*/

//t@NVIuWFNg`
template <typename _R> struct _funcobj_0arg
{
	typedef _R return_type;
};

template <typename _R, typename _A1> struct _funcobj_1arg
{
	typedef _R return_type;
	typedef _A1 arg1_type;
};

template <typename _R, typename _A1, typename _A2> struct _funcobj_2args
{
	typedef _R return_type;
	typedef _A1 arg1_type;
	typedef _A2 arg2_type;
};

template <typename _R, typename _A1, typename _A2, typename _A3> struct _funcobj_3args
{
	typedef _R return_type;
	typedef _A1 arg1_type;
	typedef _A2 arg2_type;
	typedef _A3 arg3_type;
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4> struct _funcobj_4args
{
	typedef _R return_type;
	typedef _A1 arg1_type;
	typedef _A2 arg2_type;
	typedef _A3 arg3_type;
	typedef _A4 arg4_type;
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> struct _funcobj_5args
{
	typedef _R return_type;
	typedef _A1 arg1_type;
	typedef _A2 arg2_type;
	typedef _A3 arg3_type;
	typedef _A4 arg4_type;
	typedef _A5 arg5_type;
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5, typename _A6> struct _funcobj_6args
{
	typedef _R return_type;
	typedef _A1 arg1_type;
	typedef _A2 arg2_type;
	typedef _A3 arg3_type;
	typedef _A4 arg4_type;
	typedef _A5 arg5_type;
	typedef _A6 arg6_type;
};


template <typename _R> class _delegator_0arg : public _funcobj_0arg<_R>
{
public:
	explicit _delegator_0arg(_R (*_func)()): m_Func(_func){}
	virtual ~_delegator_0arg(){}
	_R operator()() const { return (m_Func()); }

protected:
	_R (*m_Func)();
};

template <typename _R, typename _A1> class _delegator_1arg : public _funcobj_1arg<_R, _A1>
{
public:
	explicit _delegator_1arg(_R (*_func)(_A1)): m_Func(_func){}
	virtual ~_delegator_1arg(){}
	_R operator()(_A1 _arg1) const { return (m_Func(_arg1)); }

protected:
	_R (*m_Func)(_A1);
};

template <typename _R, typename _A1, typename _A2> class _delegator_2args : public _funcobj_2args<_R, _A1, _A2>
{
public:
	explicit _delegator_2args(_R (*_func)(_A1, _A2)): m_Func(_func){}
	virtual ~_delegator_2args(){}
	_R operator()(_A1 _arg1, _A2 _arg2) const { return (m_Func(_arg1, _arg2)); }

protected:
	_R (*m_Func)(_A1, _A2);
};

template <typename _R, typename _A1, typename _A2, typename _A3> class _delegator_3args : public _funcobj_3args<_R, _A1, _A2, _A3>
{
public:
	explicit _delegator_3args(_R (*_func)(_A1, _A2, _A3)): m_Func(_func){}
	virtual ~_delegator_3args(){}
	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3) const { return (m_Func(_arg1, _arg2, _arg3)); }

protected:
	_R (*m_Func)(_A1, _A2, _A3);
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4> class _delegator_4args : public _funcobj_4args<_R, _A1, _A2, _A3, _A4>
{
public:
	explicit _delegator_4args(_R (*_func)(_A1, _A2, _A3, _A4)): m_Func(_func){}
	virtual ~_delegator_4args(){}
	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4) const { return (m_Func(_arg1, _arg2, _arg3, _arg4)); }

protected:
	_R (*m_Func)(_A1, _A2, _A3, _A4);
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> class _delegator_5args : public _funcobj_5args<_R, _A1, _A2, _A3, _A4, _A5>
{
public:
	explicit _delegator_5args(_R (*_func)(_A1, _A2, _A3, _A4, _A5)): m_Func(_func){}
	virtual ~_delegator_5args(){}
	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4, _A5 _arg5) const { return (m_Func(_arg1, _arg2, _arg3, _arg4, _arg5)); }

protected:
	_R (*m_Func)(_A1, _A2, _A3, _A4, _A5);
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5, typename _A6> class _delegator_6args : public _funcobj_6args<_R, _A1, _A2, _A3, _A4, _A5, _A6>
{
public:
	explicit _delegator_6args(_R (*_func)(_A1, _A2, _A3, _A4, _A5, _A6)): m_Func(_func){}
	virtual ~_delegator_6args(){}
	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4, _A5 _arg5, _A6 _arg6) const { return (m_Func(_arg1, _arg2, _arg3, _arg4, _arg5, _arg6)); }

protected:
	_R (*m_Func)(_A1, _A2, _A3, _A4, _A5, _A6);
};

template <typename _R, typename _Tc> class _delegator_mem_0arg : public _funcobj_1arg<_R, _Tc*>
{
public:
	explicit _delegator_mem_0arg(_R (_Tc::*_func)()): m_Func(_func){}
	virtual ~_delegator_mem_0arg(){}
	_R operator()(_Tc* _class) const { return ((_class->*m_Func)()); }

protected:
	_R (_Tc::*m_Func)();
};

template <typename _R, typename _Tc, typename _A1> class _delegator_mem_1arg : public _funcobj_2args<_R, _Tc*, _A1>
{
public:
	explicit _delegator_mem_1arg(_R (_Tc::*_func)(_A1)): m_Func(_func){}
	virtual ~_delegator_mem_1arg(){}
	_R operator() (_Tc* _class, _A1 _arg1) const { return ((_class->*m_Func)(_arg1)); }

protected:
	_R (_Tc::*m_Func)(_A1);
};

template <typename _R, typename _Tc, typename _A1, typename _A2> class _delegator_mem_2args : public _funcobj_3args<_R, _Tc*, _A1, _A2>
{
public:
	explicit _delegator_mem_2args(_R (_Tc::*_func)(_A1, _A2)): m_Func(_func){}
	virtual ~_delegator_mem_2args(){}
	_R operator() (_Tc* _class, _A1 _arg1, _A2 _arg2) const { return ((_class->*m_Func)(_arg1, _arg2)); }

protected:
	_R (_Tc::*m_Func)(_A1, _A2);
};

template <typename _R, typename _Tc, typename _A1, typename _A2, typename _A3> class _delegator_mem_3args : public _funcobj_4args<_R, _Tc*, _A1, _A2, _A3>
{
public:
	explicit _delegator_mem_3args(_R (_Tc::*_func)(_A1, _A2, _A3)): m_Func(_func){}
	virtual ~_delegator_mem_3args(){}
	_R operator() (_Tc* _class, _A1 _arg1, _A2 _arg2, _A3 _arg3) const { return ((_class->*m_Func)(_arg1, _arg2, _arg3)); }

protected:
	_R (_Tc::*m_Func)(_A1, _A2, _A3);
};

template <typename _R, typename _Tc, typename _A1, typename _A2, typename _A3, typename _A4> class _delegator_mem_4args : public _funcobj_5args<_R, _Tc*, _A1, _A2, _A3, _A4>
{
public:
	explicit _delegator_mem_4args(_R (_Tc::*_func)(_A1, _A2, _A3, _A4)): m_Func(_func){}
	virtual ~_delegator_mem_4args(){}
	_R operator() (_Tc* _class, _A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4) const { return ((_class->*m_Func)(_arg1, _arg2, _arg3, _arg4)); }

protected:
	_R (_Tc::*m_Func)(_A1, _A2, _A3, _A4);
};

template <typename _R, typename _Tc, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> class _delegator_mem_5args : public _funcobj_6args<_R, _Tc*, _A1, _A2, _A3, _A4, _A5>
{
public:
	explicit _delegator_mem_5args(_R (_Tc::*_func)(_A1, _A2, _A3, _A4, _A5)): m_Func(_func){}
	virtual ~_delegator_mem_5args(){}
	_R operator() (_Tc* _class, _A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4, _A5 _arg5) const { return ((_class->*m_Func)(_arg1, _arg2, _arg3, _arg4, _arg5)); }

protected:
	_R (_Tc::*m_Func)(_A1, _A2, _A3, _A4, _A5);
};

//_AoXCX^XoChgpC
template <typename _Tfunc> class _delegator_binder1st_1arg : public _funcobj_0arg<typename _Tfunc::return_type>
{
public:
	_delegator_binder1st_1arg(const _Tfunc& _func, const typename _Tfunc::arg1_type _arg1): m_Func(_func), m_Arg1(_arg1){}
	virtual ~_delegator_binder1st_1arg(){}
	typename _Tfunc::return_type operator()() const { return (m_Func(m_Arg1)); }

protected:
	_Tfunc m_Func;
	typename _Tfunc::arg1_type m_Arg1;
};

template <typename _Tfunc> class _delegator_binder1st_2args : public _funcobj_1arg<typename _Tfunc::return_type, typename _Tfunc::arg2_type>
{
public:
	_delegator_binder1st_2args(const _Tfunc& _func, const typename _Tfunc::arg1_type _arg1): m_Func(_func), m_Arg1(_arg1){}
	virtual ~_delegator_binder1st_2args(){}
	typename _Tfunc::return_type operator()(typename _Tfunc::arg2_type _arg2) const { return (m_Func(m_Arg1, _arg2)); 
	}
	//typename _Tfunc::return_type operator()(const typename _Tfunc::arg2_type _arg2) const { return (m_Func(m_Arg1, _arg2)); }

protected:
	_Tfunc m_Func;
	typename _Tfunc::arg1_type m_Arg1;
};

template <typename _Tfunc> class _delegator_binder1st_3args : public _funcobj_2args<typename _Tfunc::return_type, typename
_Tfunc::arg2_type, typename _Tfunc::arg3_type>
{
public:
	_delegator_binder1st_3args(const _Tfunc& _func, const typename _Tfunc::arg1_type _arg1): m_Func(_func), m_Arg1(_arg1){}
	virtual ~_delegator_binder1st_3args(){}
	typename _Tfunc::return_type operator()(typename _Tfunc::arg2_type _arg2, typename _Tfunc::arg3_type _arg3) const { return (m_Func(m_Arg1, _arg2, _arg3)); }
	//typename _Tfunc::return_type operator()(typename _Tfunc::arg2_type& _arg2, const typename _Tfunc::arg3_type& _arg3) const { return (m_Func(m_Arg1, _arg2, _arg3)); }
	//typename _Tfunc::return_type operator()(typename _Tfunc::arg2_type& _arg2, const typename _Tfunc::arg3_type& _arg3) const return (m_Func(m_Arg1, _arg2, _arg3)); }
	//typename _Tfunc::return_type operator()(const typename _Tfunc::arg2_type _arg2, const typename _Tfunc::arg3_type _arg3) const { return (m_Func(m_Arg1, _arg2, _arg3)); }

protected:
	_Tfunc m_Func;
	typename _Tfunc::arg1_type m_Arg1;
};

template <typename _Tfunc> class _delegator_binder1st_4args : public _funcobj_3args<typename _Tfunc::return_type, typename _Tfunc::arg2_type, typename _Tfunc::arg3_type, typename _Tfunc::arg4_type>
{
public:
	_delegator_binder1st_4args(const _Tfunc& _func, const typename _Tfunc::arg1_type _arg1): m_Func(_func), m_Arg1(_arg1){}
	virtual ~_delegator_binder1st_4args(){}
	typename _Tfunc::return_type operator()(typename _Tfunc::arg2_type _arg2, typename _Tfunc::arg3_type _arg3, typename _Tfunc::arg4_type _arg4) const { return (m_Func(m_Arg1, _arg2, _arg3, _arg4)); }
	//typename _Tfunc::return_type operator()(const typename _Tfunc::arg2_type _arg2, const typename _Tfunc::arg3_type _arg3, const typename _Tfunc::arg4_type _arg4) const { return (m_Func(m_Arg1, _arg2, _arg3, _arg4)); }

protected:
	_Tfunc m_Func;
	typename _Tfunc::arg1_type m_Arg1;
};

template <typename _Tfunc> class _delegator_binder1st_5args : public _funcobj_4args<typename _Tfunc::return_type, typename _Tfunc::arg2_type, typename _Tfunc::arg3_type, typename _Tfunc::arg4_type, typename _Tfunc::arg5_type>
{
public:
	_delegator_binder1st_5args(const _Tfunc& _func, const typename _Tfunc::arg1_type _arg1): m_Func(_func), m_Arg1(_arg1){}
	virtual ~_delegator_binder1st_5args(){}
	typename _Tfunc::return_type operator()(typename _Tfunc::arg2_type _arg2, typename _Tfunc::arg3_type _arg3, typename _Tfunc::arg4_type _arg4, typename _Tfunc::arg5_type _arg5) const { return (m_Func(m_Arg1, _arg2, _arg3, _arg4, _arg5)); }
	//typename _Tfunc::return_type operator()(const typename _Tfunc::arg2_type _arg2, const typename _Tfunc::arg3_type _arg3, const typename _Tfunc::arg4_type _arg4) const { return (m_Func(m_Arg1, _arg2, _arg3, _arg4)); }

protected:
	_Tfunc m_Func;
	typename _Tfunc::arg1_type m_Arg1;
};

template <typename _Tfunc> class _delegator_binder1st_6args : public _funcobj_5args<typename _Tfunc::return_type, typename _Tfunc::arg2_type, typename _Tfunc::arg3_type, typename _Tfunc::arg4_type, typename _Tfunc::arg5_type, typename _Tfunc::arg6_type>
{
public:
	_delegator_binder1st_6args(const _Tfunc& _func, const typename _Tfunc::arg1_type _arg1): m_Func(_func), m_Arg1(_arg1){}
	virtual ~_delegator_binder1st_6args(){}
	typename _Tfunc::return_type operator()(typename _Tfunc::arg2_type _arg2, typename _Tfunc::arg3_type _arg3, typename _Tfunc::arg4_type _arg4, typename _Tfunc::arg5_type _arg5, typename _Tfunc::arg6_type _arg6) const { return (m_Func(m_Arg1, _arg2, _arg3, _arg4, _arg5, _arg6)); }
	//typename _Tfunc::return_type operator()(const typename _Tfunc::arg2_type _arg2, const typename _Tfunc::arg3_type _arg3, const typename _Tfunc::arg4_type _arg4) const { return (m_Func(m_Arg1, _arg2, _arg3, _arg4)); }

protected:
	_Tfunc m_Func;
	typename _Tfunc::arg1_type m_Arg1;
};

template <typename _R> inline _delegator_0arg<_R> _delegator_create(_R (*_func)())
{
	return (_delegator_0arg<_R>(_func));
}

template <typename _R, typename _A1> inline _delegator_1arg<_R, _A1> _delegator_create(_R (*_func)(_A1))
{
	return (_delegator_1arg<_R, _A1>(_func));
}

template <typename _R, typename _A1, typename _A2> inline _delegator_2args<_R, _A1, _A2> _delegator_create(_R (*_func)(_A1, _A2))
{
	return (_delegator_2args<_R, _A1, _A2>(_func));
}

template <typename _R, typename _A1, typename _A2, typename _A3> inline _delegator_3args<_R, _A1, _A2, _A3> _delegator_create(_R (*_func)(_A1, _A2, _A3))
{
	return (_delegator_3args<_R, _A1, _A2, _A3>(_func));
}

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4> inline _delegator_4args<_R, _A1, _A2, _A3, _A4> _delegator_create(_R (*_func)(_A1, _A2, _A3, _A4))
{
	return (_delegator_4args<_R, _A1, _A2, _A3, _A4>(_func));
}


template <typename _R, typename _Tc> inline _delegator_mem_0arg<_R, _Tc> mem_func(_R (_Tc::*_func)())
{
	return (_delegator_mem_0arg<_R, _Tc>(_func));
}

template <typename _R, typename _Tc, typename _A1> inline _delegator_mem_1arg<_R, _Tc, _A1> mem_func(_R (_Tc::*_func)(_A1))
{
	return (_delegator_mem_1arg<_R, _Tc, _A1>(_func));
}

template <typename _R, typename _Tc, typename _A1, typename _A2> inline _delegator_mem_2args<_R, _Tc, _A1, _A2> mem_func(_R (_Tc::*_func)(_A1, _A2))
{
	return (_delegator_mem_2args<_R, _Tc, _A1, _A2>(_func));
}

template <typename _R, typename _Tc, typename _A1, typename _A2, typename _A3> inline _delegator_mem_3args<_R, _Tc, _A1, _A2, _A3> mem_func(_R (_Tc::*_func)(_A1, _A2, _A3))
{
	return (_delegator_mem_3args<_R, _Tc, _A1, _A2, _A3>(_func));
}

template <typename _R, typename _Tc, typename _A1, typename _A2, typename _A3, typename _A4> inline _delegator_mem_4args<_R, _Tc, _A1, _A2, _A3, _A4> mem_func(_R (_Tc::*_func)(_A1, _A2, _A3, _A4))
{
	return (_delegator_mem_4args<_R, _Tc, _A1, _A2, _A3, _A4>(_func));
}

template <typename _R, typename _Tc, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> inline _delegator_mem_5args<_R, _Tc, _A1, _A2, _A3, _A4, _A5> mem_func(_R (_Tc::*_func)(_A1, _A2, _A3, _A4, _A5))
{
	return (_delegator_mem_5args<_R, _Tc, _A1, _A2, _A3, _A4, _A5>(_func));
}


template <typename _R, typename _A> inline _delegator_binder1st_1arg<_delegator_1arg<_R, _A> > bind1st(const _delegator_1arg<_R, _A>& _func, const _A& _arg1)
{
	return (_delegator_binder1st_1arg<_delegator_1arg<_R, _A> >(_func, _arg1));
}

template <typename _R, typename _A> inline _delegator_binder1st_1arg<_delegator_mem_0arg<_R, _A> > bind1st(const _delegator_mem_0arg<_R, _A>& _func, _A* _arg1)
{
	return (_delegator_binder1st_1arg<_delegator_mem_0arg<_R, _A> >(_func, _arg1));
}


template <typename _Tfunc, typename _A> inline _delegator_binder1st_1arg<_delegator_binder1st_2args<_Tfunc> > bind1st(const _delegator_binder1st_2args<_Tfunc>& _func, const _A& _arg1)
{
	return (_delegator_binder1st_1arg<_delegator_binder1st_2args<_Tfunc> >(_func, _arg1));
}



template <typename _R, typename _A1, typename _A2> inline _delegator_binder1st_2args<_delegator_2args<_R, _A1, _A2> > bind1st(const _delegator_2args<_R, _A1, _A2>& _func, const _A1& _arg1)
{
	return (_delegator_binder1st_2args<_delegator_2args<_R, _A1, _A2> >(_func, _arg1));
}

template <typename _R, typename _A1, typename _A2> inline _delegator_binder1st_2args<_delegator_mem_1arg<_R, _A1, _A2> > bind1st(const _delegator_mem_1arg<_R, _A1, _A2>& _func, _A1* _arg1)
{
	return (_delegator_binder1st_2args<_delegator_mem_1arg<_R, _A1, _A2> >(_func, _arg1));
}

template <typename _Tfunc, typename _A> inline _delegator_binder1st_2args<_delegator_binder1st_3args<_Tfunc> > bind1st(const _delegator_binder1st_3args<_Tfunc>& _func, const _A& _arg1)
{
	return (_delegator_binder1st_2args<_delegator_binder1st_3args<_Tfunc> >(_func, _arg1));
}


template <typename _R, typename _A1, typename _A2, typename _A3> inline _delegator_binder1st_3args<_delegator_3args<_R, _A1, _A2, _A3> > bind1st(const _delegator_3args<_R, _A1, _A2, _A3>& _func, const _A1& _arg1)
{
	return (_delegator_binder1st_3args<_delegator_3args<_R, _A1, _A2, _A3> >(_func, _arg1));
}

template <typename _R, typename _A1, typename _A2, typename _A3> inline _delegator_binder1st_3args<_delegator_mem_2args<_R, _A1, _A2, _A3> > bind1st(const _delegator_mem_2args<_R, _A1, _A2, _A3>& _func, _A1* _arg1)
{
	return (_delegator_binder1st_3args<_delegator_mem_2args<_R, _A1, _A2, _A3> >(_func, _arg1));
}

template <typename _Tfunc, typename _A> inline _delegator_binder1st_3args<_delegator_binder1st_4args<_Tfunc> > bind1st(const _delegator_binder1st_4args<_Tfunc>& _func, const _A& _arg1)
{
	return (_delegator_binder1st_3args<_delegator_binder1st_4args<_Tfunc> >(_func, _arg1));
}


template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4> inline _delegator_binder1st_4args<_delegator_4args<_R, _A1, _A2, _A3, _A4> > bind1st(const _delegator_4args<_R, _A1, _A2, _A3, _A4>& _func, const _A1& _arg1)
{
	return (_delegator_binder1st_4args<_delegator_4args<_R, _A1, _A2, _A3, _A4> >(_func, _arg1));
}

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4> inline _delegator_binder1st_4args<_delegator_mem_3args<_R, _A1, _A2, _A3, _A4> > bind1st(const _delegator_mem_3args<_R, _A1, _A2, _A3, _A4>& _func, _A1* _arg1)
{
	return (_delegator_binder1st_4args<_delegator_mem_3args<_R, _A1, _A2, _A3, _A4> >(_func, _arg1));
}

template <typename _Tfunc, typename _A> inline _delegator_binder1st_4args<_delegator_binder1st_5args<_Tfunc> > bind1st(const _delegator_binder1st_5args<_Tfunc>& _func, const _A& _arg1)
{
	return (_delegator_binder1st_4args<_delegator_binder1st_5args<_Tfunc> >(_func, _arg1));
}


template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> inline _delegator_binder1st_5args<_delegator_5args<_R, _A1, _A2, _A3, _A4, _A5> > bind1st(const _delegator_5args<_R, _A1, _A2, _A3, _A4, _A5>& _func, const _A1& _arg1)
{
	return (_delegator_binder1st_5args<_delegator_5args<_R, _A1, _A2, _A3, _A4, _A5> >(_func, _arg1));
}

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> inline _delegator_binder1st_5args<_delegator_mem_4args<_R, _A1, _A2, _A3, _A4, _A5> > bind1st(const _delegator_mem_4args<_R, _A1, _A2, _A3, _A4, _A5>& _func, _A1* _arg1)
{
	return (_delegator_binder1st_5args<_delegator_mem_4args<_R, _A1, _A2, _A3, _A4, _A5> >(_func, _arg1));
}

template <typename _Tfunc, typename _A> inline _delegator_binder1st_5args<_delegator_binder1st_6args<_Tfunc> > bind1st(const _delegator_binder1st_6args<_Tfunc>& _func, const _A& _arg1)
{
	return (_delegator_binder1st_5args<_delegator_binder1st_6args<_Tfunc> >(_func, _arg1));
}


template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5, typename _A6> inline _delegator_binder1st_6args<_delegator_6args<_R, _A1, _A2, _A3, _A4, _A5, _A6> > bind1st(const _delegator_6args<_R, _A1, _A2, _A3, _A4, _A5, _A6>& _func, const _A1& _arg1)
{
	return (_delegator_binder1st_6args<_delegator_6args<_R, _A1, _A2, _A3, _A4, _A5, _A6> >(_func, _arg1));
}

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5, typename _A6> inline _delegator_binder1st_6args<_delegator_mem_5args<_R, _A1, _A2, _A3, _A4, _A5, _A6> > bind1st(const _delegator_mem_5args<_R, _A1, _A2, _A3, _A4, _A5, _A6>& _func, _A1* _arg1)
{
	return (_delegator_binder1st_6args<_delegator_mem_5args<_R, _A1, _A2, _A3, _A4, _A5, _A6> >(_func, _arg1));
}
//delegator

template <typename _Tfuncobj> struct worker
{
	static void erase(any_pointer _obj)
	{
		delete (reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj));
	}

	static void copy(const any_pointer& _obj, any_pointer* _dest)
	{
		_dest->ptr_obj = new _Tfuncobj(*(reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj)));
	}
};

template <typename _R, typename _A1 = NullType, typename _A2 = NullType, typename _A3 = NullType, typename _A4 = NullType, typename _A5 = NullType, typename _A6 = NullType> class delegator;

template <typename _Tfuncobj> struct invoker_0
{
	static typename _Tfuncobj::return_type invoke(any_pointer _obj)
	{
		return (*(reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj)))();
	}
};

template <typename _R> class delegator<_R>
{
public:
	delegator(){ functor.ptr_obj = 0; }

	delegator(const delegator<_R>& in_copy): invoke(in_copy.invoke), copy(in_copy.copy), erase(in_copy.erase)
	{
		copy(in_copy.functor, &functor);
	}

	template <typename _Tfunc> delegator(_Tfunc _funcobj)
	{
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_0<_Tfunc>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;
	}

	virtual ~delegator()
	{
		if(0 != functor.ptr_obj)
			erase(functor);
	}

	bool is_Null() const
	{
		return 0 == functor.ptr_obj;
	}

	delegator<_R>& operator = (const delegator<_R>& in_copy)
	{
		if(this == &in_copy)
			return *this;

		if(functor.ptr_obj)
			erase(functor);

		invoke = in_copy.invoke;
		erase = in_copy.erase;
		copy = in_copy.copy;

		copy(in_copy.functor, &functor);
		return *this;
	}

	template <typename _Tfunc> delegator<_R>& operator = (_Tfunc _funcobj)
	{
		if(functor.ptr_obj)
			erase(functor);//delete functor.ptr_obj;
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_0<_Tfunc>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;

		return *this;
	}

	_R operator()()
	{
		return ((*invoke)(functor));
	}
protected:
	_R (*invoke)(any_pointer);
	void (*erase)(any_pointer);
	void (*copy)(const any_pointer&, any_pointer*);
	any_pointer functor;
};


template <typename _Tfuncobj, typename _A1> struct invoker_1
{
	static typename _Tfuncobj::return_type invoke(any_pointer _obj, _A1 _arg1)
	{
		return (*(reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj)))(_arg1);
	}
};

template <typename _R, typename _A1> class delegator<_R, _A1>
{
public:
	delegator(){ functor.ptr_obj = 0; }

	delegator(const delegator<_R, _A1>& in_copy): invoke(in_copy.invoke), copy(in_copy.copy), erase(in_copy.erase)
	{
		copy(in_copy.functor, &functor);
	}

	template <typename _Tfunc> delegator(_Tfunc _funcobj)
	{
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_1<_Tfunc, _A1>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;
	}

	virtual ~delegator()
	{
		if(0 != functor.ptr_obj)
			erase(functor);//delete functor.ptr_obj;
	}

	bool is_Null() const
	{
		return 0 == functor.ptr_obj;
	}

	delegator<_R, _A1>& operator = (const delegator<_R, _A1>& in_copy)
	{
		if(this == &in_copy)
			return *this;

		if(functor.ptr_obj)
			erase(functor);//delete functor.ptr_obj;

		invoke = in_copy.invoke;
		erase = in_copy.erase;
		copy = in_copy.copy;

		copy(in_copy.functor, &functor);
		return *this;
	}

	template <typename _Tfunc> delegator<_R, _A1>& operator = (_Tfunc _funcobj)
	{
		if(functor.ptr_obj)
			erase(functor);//delete functor.ptr_obj;
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_1<_Tfunc, _A1>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;

		return *this;
	}

	_R operator()(_A1 _arg1)
	{
		return ((*invoke)(functor, _arg1));
	}

protected:
	_R (*invoke)(any_pointer, _A1);
	void (*erase)(any_pointer);
	void (*copy)(const any_pointer&, any_pointer*);
	any_pointer functor;
};


template <typename _Tfuncobj, typename _A1, typename _A2> struct invoker_2
{
	static typename _Tfuncobj::return_type invoke(any_pointer _obj, _A1 _arg1, _A2 _arg2)
	{
		return (*(reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj)))(_arg1, _arg2);
	}
};

template <typename _R, typename _A1, typename _A2> class delegator<_R, _A1, _A2>
{
public:
	delegator(){ functor.ptr_obj = 0; }

	delegator(const delegator<_R, _A1, _A2>& in_copy): invoke(in_copy.invoke), copy(in_copy.copy), erase(in_copy.erase)
	{
		copy(in_copy.functor, &functor);
	}

	virtual ~delegator()
	{
		if(0 != functor.ptr_obj)
			erase(functor);
	}

	bool is_Null() const
	{
		return 0 == functor.ptr_obj;
	}

	template <typename _Tfunc> delegator(_Tfunc _funcobj)
	{
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_2<_Tfunc, _A1, _A2>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;
	}

	template <typename _Tfunc> delegator<_R, _A1, _A2>& operator = (_Tfunc _funcobj)
	{
		if(functor.ptr_obj)
			erase(functor);
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_2<_Tfunc, _A1, _A2>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;

		return *this;
	}

	delegator<_R, _A1, _A2>& operator = (const delegator<_R, _A1, _A2>& in_copy)
	{
		if(this == &in_copy)
			return *this;

		if(functor.ptr_obj)
			erase(functor);

		invoke = in_copy.invoke;
		erase = in_copy.erase;
		copy = in_copy.copy;

		copy(in_copy.functor, &functor);
		return *this;
	}

	_R operator()(_A1 _arg1, _A2 _arg2)
	{
		return ((*invoke)(functor, _arg1, _arg2));
	}

protected:
	_R (*invoke)(any_pointer, _A1, _A2);
	void (*erase)(any_pointer);
	void (*copy)(const any_pointer&, any_pointer*);
	any_pointer functor;
};


template <typename _Tfuncobj, typename _A1, typename _A2, typename _A3> struct invoker_3
{
	static typename _Tfuncobj::return_type invoke(any_pointer _obj, _A1 _arg1, _A2 _arg2, _A3 _arg3)
	{
		return (*(reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj)))(_arg1, _arg2, _arg3);
	}
};

template <typename _R, typename _A1, typename _A2, typename _A3> class delegator<_R, _A1, _A2, _A3>
{
public:
	delegator(){ functor.ptr_obj = 0; }

	delegator(const delegator<_R, _A1, _A2, _A3>& in_copy): invoke(in_copy.invoke), copy(in_copy.copy), erase(in_copy.erase)
	{
		copy(in_copy.functor, &functor);
	}

	virtual ~delegator()
	{
		if(0 != functor.ptr_obj)
			erase(functor);
	}

	bool is_Null() const
	{
		return 0 == functor.ptr_obj;
	}

	template <typename _Tfunc> delegator(_Tfunc _funcobj)
	{
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_3<_Tfunc, _A1, _A2, _A3>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;
	}

	delegator<_R, _A1, _A2, _A3>& operator = (const delegator<_R, _A1, _A2, _A3>& in_copy)
	{
		if(this == &in_copy)
			return *this;

		if(functor.ptr_obj)
			erase(functor);

		invoke = in_copy.invoke;
		erase = in_copy.erase;
		copy = in_copy.copy;

		copy(in_copy.functor, &functor);
		return *this;
	}

	template <typename _Tfunc> delegator<_R, _A1, _A2, _A3>& operator = (_Tfunc _funcobj)
	{
		if(functor.ptr_obj)
			erase(functor);
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_3<_Tfunc, _A1, _A2, _A3>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;

		return *this;
	}

	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3)
	{
		return ((*invoke)(functor, _arg1, _arg2, _arg3));
	}

protected:
	_R (*invoke)(any_pointer, _A1, _A2, _A3);
	void (*erase)(any_pointer);
	void (*copy)(const any_pointer&, any_pointer*);
	any_pointer functor;
};

template <typename _Tfuncobj, typename _A1, typename _A2, typename _A3, typename _A4> struct invoker_4
{
	static typename _Tfuncobj::return_type invoke(any_pointer _obj, _A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4)
	{
		return (*(reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj)))(_arg1, _arg2, _arg3, _arg4);
	}
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4> class delegator<_R, _A1, _A2, _A3, _A4>
{
public:
	delegator(){ functor.ptr_obj = 0; }

	delegator(const delegator<_R, _A1, _A2, _A3, _A4>& in_copy): invoke(in_copy.invoke), copy(in_copy.copy), erase(in_copy.erase)
	{
		copy(in_copy.functor, &functor);
	}

	virtual ~delegator()
	{
		if(0 != functor.ptr_obj)
			erase(functor);
	}

	bool is_Null() const
	{
		return 0 == functor.ptr_obj;
	}

	template <typename _Tfunc> delegator(_Tfunc _funcobj)
	{
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_4<_Tfunc, _A1, _A2, _A3, _A4>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;
	}

	delegator<_R, _A1, _A2, _A3, _A4>& operator = (const delegator<_R, _A1, _A2, _A3, _A4>& in_copy)
	{
		if(this == &in_copy)
			return *this;

		if(functor.ptr_obj)
			erase(functor);

		invoke = in_copy.invoke;
		erase = in_copy.erase;
		copy = in_copy.copy;

		copy(in_copy.functor, &functor);
		return *this;
	}

	template <typename _Tfunc> delegator<_R, _A1, _A2, _A3, _A4>& operator = (_Tfunc _funcobj)
	{
		if(functor.ptr_obj)
			erase(functor);
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_4<_Tfunc, _A1, _A2, _A3, _A4>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;

		return *this;
	}

	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4)
	{
		return ((*invoke)(functor, _arg1, _arg2, _arg3, _arg4));
	}

protected:
	_R (*invoke)(any_pointer, _A1, _A2, _A3, _A4);
	void (*erase)(any_pointer);
	void (*copy)(const any_pointer&, any_pointer*);
	any_pointer functor;
};

template <typename _Tfuncobj, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> struct invoker_5
{
	static typename _Tfuncobj::return_type invoke(any_pointer _obj, _A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4, _A5 _arg5)
	{
		return (*(reinterpret_cast<_Tfuncobj*>(_obj.ptr_obj)))(_arg1, _arg2, _arg3, _arg4, _arg5);
	}
};

template <typename _R, typename _A1, typename _A2, typename _A3, typename _A4, typename _A5> class delegator<_R, _A1, _A2, _A3, _A4, _A5>
{
public:
	delegator(){ functor.ptr_obj = 0; }

	delegator(const delegator<_R, _A1, _A2, _A3, _A4, _A5>& in_copy): invoke(in_copy.invoke), copy(in_copy.copy), erase(in_copy.erase)
	{
		copy(in_copy.functor, &functor);
	}

	virtual ~delegator()
	{
		if(0 != functor.ptr_obj)
			erase(functor);
	}

	bool is_Null() const
	{
		return 0 == functor.ptr_obj;
	}

	template <typename _Tfunc> delegator(_Tfunc _funcobj)
	{
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_5<_Tfunc, _A1, _A2, _A3, _A4, _A5>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;
	}

	delegator<_R, _A1, _A2, _A3, _A4, _A5>& operator = (const delegator<_R, _A1, _A2, _A3, _A4, _A5>& in_copy)
	{
		if(this == &in_copy)
			return *this;

		if(functor.ptr_obj)
			erase(functor);

		invoke = in_copy.invoke;
		erase = in_copy.erase;
		copy = in_copy.copy;

		copy(in_copy.functor, &functor);
		return *this;
	}

	template <typename _Tfunc> delegator<_R, _A1, _A2, _A3, _A4, _A5>& operator = (_Tfunc _funcobj)
	{
		if(functor.ptr_obj)
			erase(functor);
		functor.ptr_obj = new _Tfunc(_funcobj);
		invoke = &invoker_5<_Tfunc, _A1, _A2, _A3, _A4, _A5>::invoke;
		erase = &worker<_Tfunc>::erase;
		copy = &worker<_Tfunc>::copy;

		return *this;
	}

	_R operator()(_A1 _arg1, _A2 _arg2, _A3 _arg3, _A4 _arg4, _A5 _arg5)
	{
		return ((*invoke)(functor, _arg1, _arg2, _arg3, _arg4, _arg5));
	}

protected:
	_R (*invoke)(any_pointer, _A1, _A2, _A3, _A4, _A5);
	void (*erase)(any_pointer);
	void (*copy)(const any_pointer&, any_pointer*);
	any_pointer functor;
};

#ifdef _MSC_VER
#pragma pack(pop)
#endif

#endif
