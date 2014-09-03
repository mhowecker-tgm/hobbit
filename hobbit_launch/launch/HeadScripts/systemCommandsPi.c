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
     if (strcmp(argv[i],"update")==0)      { system("dpkg --configure -a && apt-get update && apt-get upgrade -y && rpi-update"); } else
     if (strcmp(argv[i],"chronyStop")==0)  { system("/etc/init.d/chrony stop"); } else
     if (strcmp(argv[i],"ntpdate")==0)     { system("ntpdate 192.168.2.122"); } else
     if (strcmp(argv[i],"chronyStart")==0) { system("/etc/init.d/chrony start"); }
   }

   return 0;
}
