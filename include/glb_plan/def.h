#ifndef DEF_H
#define DEF_H

#ifndef NULL
    #define NULL    0
#endif

#ifndef BOOL
    #define BOOL    int
    #define TRUE    1
    #define FALSE   0
#endif

#ifndef int8_t
//    typedef char            int8_t;
    typedef short           int16_t;
    typedef int             int32_t;
    typedef unsigned char   uint8_t, BYTE;
    typedef unsigned short  uint16_t, WORD;
    typedef unsigned int    uint32_t, DWORD;
#endif

#endif // DEF_H
