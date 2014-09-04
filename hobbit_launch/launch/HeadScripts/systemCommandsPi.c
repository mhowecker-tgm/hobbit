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
     if (strcmp(argv[i],"update")==0)             { return system("dpkg --configure -a && apt-get update && apt-get upgrade -y && rpi-update"); } else
     if (strcmp(argv[i],"restart")==0)            { return system("init 6"); } else
     if (strcmp(argv[i],"shutdown")==0)           { return system("init 0"); } else
     if (strcmp(argv[i],"shutdownOtherPi")==0)    { return system("echo notyet"); } else
     if (strcmp(argv[i],"chronyStop")==0)         { return system("/etc/init.d/chrony stop"); } else
     if (strcmp(argv[i],"ntpdate")==0)            { return system("ntpdate 192.168.2.122"); } else
     if (strcmp(argv[i],"chronyStart")==0)        { return system("/etc/init.d/chrony start"); }
   }

   return 1;
}
