#ifndef UTILITIES_H
#define UTILITIES_H

#include"timer.h"
#include"../containers/string.h"

namespace mech {

	struct ScopeProfiler {

		Timer timer;
		String name;

		ScopeProfiler(const char* n) : name(n) {}
		~ScopeProfiler();
	};

#if mech_ENABLE_PROFILER
#define PROFILE(name) ScopeProfiler profiler(name)
#else
#define PROFILE(name) void()
#endif

	void log(const char* str);

#if mech_ENABLE_LOGGER
#define LOG(str) log(str)
#else
#define LOG(str) void()
#endif


}

#endif
