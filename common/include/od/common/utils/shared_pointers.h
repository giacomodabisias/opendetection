#ifdef WITH_BOOST_SHARED_PTR
	#define shared_ptr boost:shared_ptr
	#define make_shared boost::make_shared
	#include <boost/shared_ptr.hpp>
	#include <boost/make_shared.hpp>
#else
	#define shared_ptr std::shared_ptr
	#define make_shared std::make_shared
	#include <memory>
#endif