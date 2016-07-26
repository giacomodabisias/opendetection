#pragma once
#ifdef WITH_STD_SHARED_PTR
	#include <memory>
	using std::shared_ptr;
	using std::make_shared;
	using std::static_pointer_cast;
	using std::dynamic_pointer_cast;
#else
	#include <boost/shared_ptr.hpp>
	#include <boost/make_shared.hpp>
	#include <boost/pointer_cast.hpp>
	using boost::shared_ptr;
	using boost::make_shared;
	using boost::static_pointer_cast;
	using boost::dynamic_pointer_cast;
#endif