#pragma once

//// namespace
#define PHYS_NAMESPACE_BEGIN namespace physeng {
#define PHYS_NAMESPACE_END }
#define USE_PHYS_NAMESPACE using namespace physeng;

//// alignment
#define ATTRIBUTE_ALIGNED16(a) a alignas(16)
#define ATTRIBUTE_ALIGNED64(a) a alignas(64)
#define ATTRIBUTE_ALIGNED128(a) a alignas(128)

//// filename macro
#ifdef __APPLE__
#define __FILENAME__ (strrchr(__FILE__, '/') + 1)
#else
#define __FILENAME__ (strrchr(__FILE__, '\\') + 1)
#endif

//// inline
#ifdef _WIN32
    #define FORCE_INLINE __forceinline
#else
    #define FORCE_INLINE __attribute__ ((always_inline)) inline
#endif

//// setter & getter
#define DEFINE_MEMBER_SET_GET(T, name, Name)\
    protected:\
    T name;\
    public:\
    void set##Name(const T& t){ name = t; }\
    const T& get##Name() const { return name; }

#define DEFINE_MEMBER_PTR_SET_GET(T, name, Name)\
    protected:\
    T* name;\
    public:\
    void set##Name(T* t){ delete name; name = t; }\
    T* get##Name() const { return name; }


#define DEFINE_OBJARRAY_SET_GET(T, name, Name)\
    protected:\
    ObjectArray<T> name;\
    public:\
    void set##Name(ObjectArray<T>& arr){ name = std::move(arr); }\
    ObjectArray<T>& get##Name(){ return name; }\

//// setter & getter of arrays
#define DEFINE_ARRAY_MEMBER_SET_GET(T, cnt, name, Name)\
    protected:\
    T name[cnt];\
    public:\
    void set##Name(int idx, const T& t){ name[idx] = t; }\
    const T& get##Name(int idx) const { return name[idx]; }

#define DEFINE_DYNAMIC_ARRAY_MEMBER_SET_GET(T, name, Name)\
    protected:\
    ObjectArray<T> name;\
    public:\
    void set##Name(int idx, const T& t){ name[idx] = t; }\
    const T& get##Name(int idx) const { return name[idx]; }



#define DEFINE_MEMBER_GET(T, name, Name)\
    protected:\
    T name;\
    public:\
    const T& get##Name() const { return name; }
    

#define DEFINE_MEMBER_PTR_GET(T, name, Name)\
    protected:\
    T* name;\
    public:\
    T* get##Name() const { return name; }


//// assert
#include <assert.h>
#define ASSERT assert

#define CUSTOM_ASSERT(cond, str)\
    if(cond){\
        std::cout << "\033[35m[" << __TIME__ << "][" << __FILENAME__ << ":" << __LINE__ << "][Assert]\033[0m " << str << std::endl;\
        exit(-1);\
    }

#define UNIMPL_ASSERT()\
    CUSTOM_ASSERT(true, "unimplemented")

#if defined(DEBUG) || defined (_DEBUG)
    #define DEBUG_ASSERT assert
#else
    #define DEBUG_ASSERT void  // suppress compiler unused-value warning
#endif