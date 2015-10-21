#ifndef __PROPERTY__
#define __PROPERTY__

//2003/09/09

#include "delegator.h"

template <typename TData> class property
{
public:
	mutable delegator<TData> read;
	mutable delegator<void, const TData&> write;

	property<TData>& operator = (const TData& in_Data)
	{
		write(in_Data);
		return *this;
	}

	operator TData() const
	{
		return (TData) (read());
	}
};


template <typename T> class propertyobj
{
public:
	delegator<T*> read;
	delegator<void, const T&> write;

	propertyobj<T>& operator = (const T& in_Data)
	{
		write(in_Data);
		return *this;
	}

	operator T*()
	{
		return read();
	}

	T* operator ->()
	{
		return read();
	}
};



#endif