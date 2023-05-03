// ISO C headers.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>

#include "msb.h"

int int2barr(char *barr, int offset, void *val, unsigned int l, unsigned int h)
{
    unsigned int j;

    if (h < l)
    {
        return 0;
    }

    for (j=l;j<h;j++)
        if (BARR_TEST(val,j))
            BARR_SET(barr,(offset+j-l));
        else 
            BARR_CLEAR(barr,(offset+j-l));
    return h-l;
}

int barr2int(char *barr, int offset, void *val, unsigned int l, unsigned int h)
{
    unsigned int j;

    if (h < l)
    {
        return 0;
    }

    for (j=0;j<h-l;j++)
        if (BARR_TEST(barr,offset+j+l))
            BARR_SET(val,j);
        else
            BARR_CLEAR(val,j);
    return h-l;
}


int swap(int32_t from, size_t len)
{
    int32_t to = 0, i;
    char *barr_from = (char*)&from;
    char *barr_to = (char*)&to;
    for (i = 0; i < len; i++) {
        barr_to[i] = barr_from[len - i - 1];
    }
    return to;
}

void log_barr(char *barr, int len)
{
    int i;

    if (barr!=NULL)
        for (i=0;i<len;i++)
            printf("%d",BARR_TEST(barr,i)&&1);
    printf("\n");
}
