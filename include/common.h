/**
 * Copyright (C) 2014 Daniel Turecek
 *
 * @file      common.h
 * @author    Daniel Turecek <daniel@turecek.de>
 * @date      2014-07-16
 *
 * Common file defining basic types, constatns and
 * usefull macros. Used almost in every source file
 * in the software.
 *
 */
#ifndef COMMON_H
#define COMMON_H
#define NO_DBGTRACE

// basic data types
typedef int BOOL;
typedef unsigned char byte;
typedef char i8;
typedef unsigned char u8;
typedef short i16;
typedef unsigned short u16;
typedef int i32;
typedef unsigned int u32;
typedef long long i64;
typedef unsigned long long u64;

#ifndef FALSE
#define FALSE  0
#endif
#ifndef TRUE
#define TRUE   1
#endif

#ifdef _MSC_VER
#define NOMINMAX
#endif

#include <cstdlib>
typedef size_t sizet;

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
typedef u32 THREADID;
#define PATH_SEPAR          '\\'
#define PATH_SEPAR_STR      "\\"
#define PATH_SEPAR_STRW      L"\\"
#else
#include <stdint.h>
typedef long THREADID;
typedef void* HMODULE;
typedef void* HINSTANCE;
#define MAXDWORD            0xffffffff
#define PATH_SEPAR          '/'
#define PATH_SEPAR_STR      "/"
#endif

#define MAX_U32             0xffffffff

typedef enum _DataType
{
    DT_CHAR     = 0,  // signed char
    DT_BYTE     = 1,  // unsigned char
    DT_I16      = 2,  // signed short
    DT_U16      = 3,  // unsigned short
    DT_I32      = 4,  // int
    DT_U32      = 5,  // unsigned int
    DT_I64      = 6,  // long long
    DT_U64      = 7,  // unsigned long long
    DT_FLOAT    = 8,  // float
    DT_DOUBLE   = 9,  // double
    DT_BOOL     = 10, // BOOL (int)
    DT_STRING   = 11, // const char *
    DT_LAST     = 12, // end of table
} DataType;

typedef unsigned DEVID;
typedef unsigned DATAID;
typedef unsigned EVENTID;
typedef unsigned FRAMEID;
typedef unsigned MENUITEMID;
typedef u64 THREADPARAM;

#define PXUNUSED(x) (void)x;

#ifndef WIN32
    #define EXPORTFUNC extern "C" __attribute__ ((visibility("default")))
#else
    #define EXPORTFUNC extern "C" __declspec(dllexport)
#endif

#ifdef WIN32
    #define _STR(x) #x
    #define STR(x) _STR(x)
    #define TODO(x) __pragma(message("TODO: " _STR(x) " :: " __FILE__ "@" STR(__LINE__)))
#else
    #define DOPRAGMA(x) _Pragma(#x)
    #define TODO(x) DOPRAGMA(GCC warning #x)
#endif

#endif /* end of include guard: COMMON_H */
