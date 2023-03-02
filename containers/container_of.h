#ifndef _CONTAINER_OF_H
#define _CONTAINER_OF_H
#if __linux__   //  or #if __GNUC__
    #if __LP64__
        #define ENVIRONMENT64
    #else
        #define ENVIRONMENT32
    #endif
#else
    #if _WIN32
        #define ENVIRONMENT32
    #else
        #define ENVIRONMENT64
    #endif
#endif // __linux__

#ifdef ENVIRONMENT64
    #define PTR_OFFSET (unsigned long long int)
#else
    #define PTR_OFFSET (unsigned long int)
#endif // ENVIRONMENT64

#define adv_offsetof(TYPE, MEMBER) (PTR_OFFSET &((TYPE *)0)->MEMBER)

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
               (type *)( (char *)__mptr - adv_offsetof(type,member) );})

#define list_entry(ptr,type,member)     \
    container_of(ptr, type, member)

#endif
