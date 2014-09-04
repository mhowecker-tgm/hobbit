#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
   setuid( 0 );

   int i=0;
   for (i=0; i<argc; i++)
   {
     if (strcmp(argv[i],"updateGlobal")==0) { return system("apt-get update && apt-get upgrade -y"); } else
     if (strcmp(argv[i],"shutdownPi")==0)   { return system("nc6 192.168.2.199 8086"); } else 
     if (strcmp(argv[i],"shutdown")==0)     { return system("init 0"); } else
     if (strcmp(argv[i],"restart")==0)      { return system("init 6"); } 
   }

   return 0;
}
