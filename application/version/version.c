/*
 * version.c
 *
 *  Created on: Sep 25, 2017
 *      Author: pawel
 */

#include "version.h"
#include "version_gen.h"
#include <string.h>

#define xstr(a) str(a)
#define str(a) #a

#ifndef VER_PROG_NAME
    static const char program_name[] = "Unknown";
#else
    static const char program_name[] = xstr(VER_PROG_NAME);
#endif

#ifndef VER_TIME
    static const char compilation_time[] = __TIME__;
#else
    static const char compilation_time[] = xstr(VER_TIME);
#endif

#ifndef VER_DATE
    static const char compilation_date[] = __DATE__;
#else
    static const char compilation_date[] = xstr(VER_DATE);
#endif

#ifndef VER_GIT_REV
    static const char sw_revision[] = "Unknown";
#else
    static const char sw_revision[] = xstr(VER_GIT_REV);
#endif

static size_t get(char *buf, char const *from, size_t buflen)
{
    if (buflen != 0) {
        strncpy(buf, from, buflen);
    }

    return strlen(from);
}

size_t VER_get_program_name(char *buf, size_t buflen)
{
    return get(buf, program_name, buflen);
}

size_t VER_get_sw_revision(char *buf, size_t buflen)
{
    return get(buf, sw_revision, buflen);
}

size_t VER_get_compilation_time(char *buf, size_t buflen)
{
    return get(buf, compilation_time, buflen);
}

size_t VER_get_compilation_date(char *buf, size_t buflen)
{
    return get(buf, compilation_date, buflen);
}
