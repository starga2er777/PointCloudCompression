#include "f2c.h"

/* compare two strings */
int s_cmp(char *a0, char *b0)
{
    int la = (int)strlen(a0);
    int lb = (int)strlen(b0);
    unsigned char *a, *aend, *b, *bend;
    a = (unsigned char *)a0;
    b = (unsigned char *)b0;
    aend = a + la;
    bend = b + lb;

    if(la <= lb)
    	{
    	while(a < aend)
    		if(*a != *b)
    			return( *a - *b );
    		else
    			{ ++a; ++b; }

    	while(b < bend)
    		if(*b != ' ')
    			return( ' ' - *b );
    		else	++b;
    	}
    else
    	{
    	while(b < bend)
    		if(*a == *b)
    			{ ++a; ++b; }
    		else
    			return( *a - *b );
    	while(a < aend)
    		if(*a != ' ')
    			return(*a - ' ');
    		else	++a;
    	}
    return(0);
}
