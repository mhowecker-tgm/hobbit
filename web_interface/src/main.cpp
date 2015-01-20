/*
AmmarServer , main executable

URLs: http://ammar.gr
Written by Ammar Qammaz a.k.a. AmmarkoV 2012

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "AmmServerlib.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <stdexcept>
#include <image_transport/image_transport.h>


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>

#include <std_srvs/Empty.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>


#define NODE_NAME "web_interface"

ros::NodeHandle * nhPtr=0;


#define MAX_BINDING_PORT 65534

#define ENABLE_PASSWORD_PROTECTION 0
#define ENABLE_CHAT_BOX 0


#define DEFAULT_BINDING_PORT 8080  // <--- Change this to 80 if you want to bind to the default http port..!
#define MAX_COMMAND_SIZE 2048
#define SMALL_CMD_BUF 256


#define WEBROOT "./"
#define TEMPLATEROOT "./templates/"
char webserver_root[MAX_FILE_PATH]=WEBROOT; // <- change this to the directory that contains your content if you dont want to use the default public_html dir..
char templates_root[MAX_FILE_PATH]=TEMPLATEROOT;

unsigned int enableBatteryPrompt=0;
unsigned int stopWebInterface=0;
char * page=0;
unsigned int pageLength=0;

//----------------------------------------------------------
//Advertised Service switches
bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Web Interface");
    stopWebInterface=1;
    return true;
}

//The deceleration of some dynamic content resources..
struct AmmServer_Instance  * default_server=0;
struct AmmServer_Instance  * admin_server=0;

struct AmmServer_RH_Context indexPage={0};
struct AmmServer_RH_Context settings={0};
struct AmmServer_RH_Context stats={0};
struct AmmServer_RH_Context form={0};
struct AmmServer_RH_Context base_image={0};
struct AmmServer_RH_Context top_image={0};
struct AmmServer_RH_Context tf_image={0};
struct AmmServer_RH_Context map_image={0};



void replaceChar(char * input , char findChar , char replaceWith)
{
  #warning "TODO : Replace replaceChar with AmmServer_ReplaceCharInString"
  char * cur = input;
  char * inputEnd = input+strlen(input);
  while ( cur < inputEnd )
  {
     if (*cur == findChar ) { *cur = replaceWith; }
     ++cur;
  }
  return ;
}


//This function prepares the content of  stats context , ( stats.content )
void * prepare_index_content_callback(struct AmmServer_DynamicRequest  * rqst)
{
  //No range check but since everything here is static max_stats_size should be big enough not to segfault with the strcat calls!
  strncpy(rqst->content,"<html><head><title>Welcome to The Hobbit WebInterface</title>\
                  <meta http-equiv=\"refresh\" content=\"0; url=controlpanel.html\" />\
                  <body><center><br><br><br>\
                   <h1>The incredibly minimal WebInterface for Hobbit</h1><br><br>\
                   <h3><a href=\"controlpanel.html\">Click Here for remote operation control panel</a></h3>\
                   <h3><a href=\"facilitator_panel.html\">Click Here for facilitator panel</a></h3>\
                   </body></html> ",rqst->MAXcontentSize);
  rqst->contentSize=strlen(rqst->content);
  return 0;
}



int getBackCommandLine(char *  command , char * what2GetBack , unsigned int what2GetBackMaxSize)
{
 FILE *fp;
 /* Open the command for reading. */
 fp = popen(command, "r");
 if (fp == 0 )
       {
         fprintf(stderr,"Failed to run command (%s) \n",command);
         return 0;
       }

 /* Read the output a line at a time - output it. */
  unsigned int i=0;
  while (fgets(what2GetBack, what2GetBackMaxSize , fp) != 0)
    {
        ++i;
        //fprintf(stderr,"\n\nline %u = %s \n",i,output);
        break;
    }
  /* close */
  pclose(fp);
  return 1;
}


int getBackCPUAndMEMUsageForProcess(unsigned int pid,float * cpuUsage , float * memUsage)
{
  if ( (cpuUsage==0) || (memUsage==0) ) { return 0; }

  *cpuUsage=0.0;
  *memUsage=0.0;

  char what2GetBack[1024]={0};
  char what2Execute[1024];
  snprintf(what2Execute,1024,"ps aux | awk '{print $2\",\"$3\",\"$4\",\"$11}' | grep %u",pid);
  //fprintf(stderr,"Executing %s\n",what2Execute);
  if ( getBackCommandLine(what2Execute, what2GetBack , 1024 ) )
  {
      //We get back something like 21230,20.8,0.4,../../../devel/lib/skeleton_detector/skeleton_detector
      //fprintf(stderr,"Got Back %s \n",what2GetBack);
      char * firstComma = strchr(what2GetBack,',');
      if (firstComma==0) { return 0; }
      char * secondComma = strchr(firstComma+1,',');
      if (secondComma ==0) { return 0; }
      char * thirdComma = strchr(secondComma+1,',');
      if (thirdComma ==0) { return 0; }

      *thirdComma=0;
      *secondComma=0;
      ++secondComma;
      ++firstComma;

      *cpuUsage = atof(firstComma);
      *memUsage = atof(secondComma);

      return 1;
  }
  return 0;
}

int getBackCPUAndMEMUsageForThisProcess(float * cpuUsage , float * memUsage)
{
  unsigned int pid=getpid();
  return getBackCPUAndMEMUsageForProcess(pid,cpuUsage,memUsage);
}

int processExists(char * safeProcessName)
{
  char what2Execute[MAX_COMMAND_SIZE];
  char what2GetBack[MAX_COMMAND_SIZE]={0};
  unsigned int what2GetBackMaxSize=MAX_COMMAND_SIZE;

  //cut -d ' ' -f2 or -f1 forsome fucking reason some times there is a space before and cut screws up
  snprintf(what2Execute,MAX_COMMAND_SIZE,"ps -A | grep %s | awk '{print $1}'",safeProcessName);
  //snprintf(what2Execute,MAX_COMMAND_SIZE,"/bin/bash -c \"ps -A | grep %s | cut -d ' ' -f1\" ",safeProcessName);
  if ( getBackCommandLine( what2Execute ,  what2GetBack , what2GetBackMaxSize) )
    {
      fprintf(stderr,"Run %s , got back %s which is %u\n",what2Execute,what2GetBack,atoi(what2GetBack));

      if (atoi(what2GetBack)>0)
       {
        return 1;
       }
    }
  return 0;
}


int screenExists(char * safeProcessName)
{
  unsigned int screenExistsResult = 0;
  char what2Execute[MAX_COMMAND_SIZE];
  char what2GetBack[MAX_COMMAND_SIZE]={0};
  unsigned int what2GetBackMaxSize=MAX_COMMAND_SIZE;

  snprintf(what2Execute,MAX_COMMAND_SIZE,"screen -ls | grep %s",safeProcessName);
  if ( getBackCommandLine( what2Execute ,  what2GetBack , what2GetBackMaxSize) )
    {
      if (strstr(what2GetBack,safeProcessName)!=0)
          {
           screenExistsResult=1;
          }
      fprintf(stderr,"Run %s , got back %s which is %u\n",what2Execute,what2GetBack,screenExistsResult );

    }
  return screenExistsResult;
}

int addServiceCheck(char * mem, char * label , char * processName )
{
  strcat(mem,"<tr><td>");
  strcat(mem,label);
  strcat(mem,"</td><td>");
  if (processExists(processName))  { strcat(mem,"<img src=\"statusOk.png\" height=15>"); } else
                                   { strcat(mem,"<img src=\"statusFailed.png\" height=15>"); }
  strcat(mem,"</td></tr>");

  return 0;
}



int addScreenCheck(char * mem, char * label , char * processName )
{
  strcat(mem,"<tr><td>");
  strcat(mem,label);
  strcat(mem,"</td><td>");
  if (screenExists(processName))  { strcat(mem,"<img src=\"statusOk.png\" height=15>"); } else
                                   { strcat(mem,"<img src=\"statusFailed.png\" height=15>"); }
  strcat(mem,"</td></tr>");

  return 0;
}

int addHeadRPICheck(char * mem, char * label)
{
  strcat(mem,"<tr><td>");
  strcat(mem,label);
  strcat(mem,"</td><td>");

  char what2GetBack[MAX_COMMAND_SIZE]={0};
  unsigned int what2GetBackMaxSize=MAX_COMMAND_SIZE;

  unsigned int rpiOK=0;
  if ( getBackCommandLine( (char*) "timeout 1 /opt/ros/hobbit_hydro/src/hobbit_launch/launch/System/systemCommands checkPi" ,  what2GetBack , what2GetBackMaxSize) )
    {
        if (strstr(what2GetBack,"alive")!=0)
          {
           rpiOK=1;
          }
    }



  if (rpiOK)  { strcat(mem,"<img src=\"statusOk.png\" height=15>"); } else
              { strcat(mem,"<img src=\"statusFailed.png\" height=15>"); }
  strcat(mem,"</td></tr>");

  return 0;
}


//This function prepares the content of  stats context , ( stats.content )
void * prepare_stats_content_callback(struct AmmServer_DynamicRequest  * rqst)
{
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  char temperatureState[MAX_COMMAND_SIZE]={0};
  getBackCommandLine((char*) "timeout 1 sensors | grep temp3 | cut -d ' ' -f9", temperatureState , MAX_COMMAND_SIZE );

  char batteryState[MAX_COMMAND_SIZE]={0};
  char chargingState[MAX_COMMAND_SIZE]={0};

  if (enableBatteryPrompt)
  {
    getBackCommandLine((char*) "timeout 1 rostopic echo /battery_state -n 1 | grep lifePercent | cut -d ':' -f2", batteryState , MAX_COMMAND_SIZE );
    float battery = atof(batteryState);

    if (battery<=1)   { fprintf(stderr,"Battery state is weird %0.2f , or %s \n",battery,batteryState); } else
    if (battery<=10)  { snprintf(batteryState,MAX_COMMAND_SIZE,"<img src=\"battery_10.png\" height=\"15\"> %0.2f %% ",battery); } else
    if (battery<=25)  { snprintf(batteryState,MAX_COMMAND_SIZE,"<img src=\"battery_25.png\" height=\"15\"> %0.2f %%",battery); } else
    if (battery<=50)  { snprintf(batteryState,MAX_COMMAND_SIZE,"<img src=\"battery_50.png\" height=\"15\"> %0.2f %%",battery); } else
    if (battery<=75)  { snprintf(batteryState,MAX_COMMAND_SIZE,"<img src=\"battery_75.png\" height=\"15\"> %0.2f %%",battery); } else
    if (battery<=100) { snprintf(batteryState,MAX_COMMAND_SIZE,"<img src=\"battery_100.png\" height=\"15\"> %0.2f %%",battery); }

    getBackCommandLine((char*) "timeout 1 rostopic echo /battery_state -n 1 | grep charging | cut -d ':' -f2", chargingState , MAX_COMMAND_SIZE );
  } else
  {
    strncpy(batteryState,"- N/A -",MAX_COMMAND_SIZE);
    strncpy(chargingState,"- N/A -",MAX_COMMAND_SIZE);
  }
  /*
  unsigned int charging=atoi(chargingState);
   "charging: True "it does not return an integer
  if (charging)  { snprintf(chargingState,MAX_COMMAND_SIZE,"<img src=\"plugged.png\" height=\"15\"> %u ",charging); } else
                 { snprintf(chargingState,MAX_COMMAND_SIZE," %u ",charging); }*/

  char cpuUsage[MAX_COMMAND_SIZE]={0};
  getBackCommandLine((char*) "grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage \"%\"}'" , cpuUsage , MAX_COMMAND_SIZE );


 //SVN version is kind of irrelevant
 // char svnVersion[MAX_COMMAND_SIZE]={0};
 // getBackCommandLine((char*) "/bin/bash -c \" svnversion \" " , svnVersion , MAX_COMMAND_SIZE );


  char statusControl[MAX_COMMAND_SIZE*8]={0};
  strcat(statusControl, (char*) "<center><table>");
   addHeadRPICheck(statusControl , (char*) "Head ");
   addServiceCheck(statusControl , (char*) "RGBDAcquisition"      , (char*)  "rgbd" );
   addScreenCheck (statusControl , (char*) "Base Camera"          , (char*)  "basecam" );
   addServiceCheck(statusControl , (char*)  "Skeleton Detector"   , (char*)  "skeleton" );
   //addServiceCheck(statusControl , (char*)  "Hand Gestures"       , (char*)  "hand" ); -- Hand Gestures are deprecated
   //addServiceCheck(statusControl , (char*)  "Face Detection"      , (char*)  "face_det" ); -- Face Detection is deprecated
   addServiceCheck(statusControl , (char*)  "Person Aggregator"   , (char*)  "person_aggre" );
   addServiceCheck(statusControl , (char*)  "Emergency Detection" , (char*)  "emergency" );
   //addServiceCheck(statusControl , (char*)  "Fitness Function"    , (char*)  "fitness" ); -- Face Detection is deprecated
   addServiceCheck(statusControl , (char*)  "Mira Center"         , (char*)  "mira" );
   addServiceCheck(statusControl , (char*)  "Joystick"            , (char*)  "joy" );
  strcat(statusControl, (char*) "</table></center>");


  //No range check but since everything here is static max_stats_size should be big enough not to segfault with the strcat calls!
  snprintf(rqst->content,rqst->MAXcontentSize,
            "<html>\
               <head><meta http-equiv=\"content-type\" content=\"text/html; charset=UTF-8\"/></head>\
               <body>\
                <center>\
                  %02d-%02d-%02d %02d:%02d:%02d \n<br><br> \
                </center>\
                CPU usage : %s \n<br> \
                CPU temp : %s \n<br> \
                Battery is : %s \n<br> \
                Charging : %s<br>\
                <br>\
                 %s <br>\
               </body>\
             </html>",
             tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900,   tm.tm_hour, tm.tm_min, tm.tm_sec,cpuUsage,temperatureState,batteryState,chargingState,statusControl);

  rqst->contentSize=strlen(rqst->content);
  return 0;
}


//This function prepares the content of  stats context , ( stats.content )
void * prepare_base_image(struct AmmServer_DynamicRequest  * rqst)
{
  unsigned int length=0;
  char * readContent = AmmServer_ReadFileToMemory((char*) "../../rgbd_acquisition/bin/frames/base/left0000.jpg",&length);

  if(readContent==0)
  {
     snprintf(rqst->content,rqst->MAXcontentSize,"<html><body><h1>Bad Image</h1></body></html>");
     rqst->contentSize=strlen(rqst->content);
  } else
  {
     if (rqst->MAXcontentSize<length)
     {
        length = rqst->MAXcontentSize;
        fprintf(stderr,"Error , not enough space to accomodate image \n");
     }
     memcpy(rqst->content,readContent,length);
     rqst->contentSize=length;
     free(readContent);
  }
  return 0;
}


//This function prepares the content of  stats context , ( stats.content )
void * prepare_top_image(struct AmmServer_DynamicRequest  * rqst)
{
  unsigned int length=0;
  char * readContent = AmmServer_ReadFileToMemory((char*) "../../rgbd_acquisition/bin/frames/top/left0000.jpg",&length);

  if(readContent==0)
  {
     snprintf(rqst->content,rqst->MAXcontentSize,"<html><body><h1>Bad Image</h1></body></html>");
     rqst->contentSize=strlen(rqst->content);
  } else
  {
     if (rqst->MAXcontentSize<length)
     {
        length = rqst->MAXcontentSize;
        fprintf(stderr,"Error , not enough space to accomodate image \n");
     }
     memcpy(rqst->content,readContent,length);
     rqst->contentSize=length;
     free(readContent);
  }
  return 0;
}


void * prepare_map_image(struct AmmServer_DynamicRequest  * rqst)
{
  unsigned int length=0;
  char * readContent = 0;

 if ( AmmServer_FileExists((char*) "../../interfaces_mira/maps/current_map/static.png") )
 {
     readContent = AmmServer_ReadFileToMemory((char*) "../../interfaces_mira/maps/current_map/static.png",&length);
 } else
 {
   int i=system("/bin/bash -c \"cd ../../rgbd_acquisition/bin/frames/map/ && timeout 2 rosrun map_server map_saver -f map  && convert map.pgm map.png\" ");

   if (i==0) {  readContent = AmmServer_ReadFileToMemory((char*) "../../rgbd_acquisition/bin/frames/map/map.png",&length);    }
 }

  if(readContent==0)
  {
     snprintf(rqst->content,rqst->MAXcontentSize,"<html><body><h1>Bad Image</h1></body></html>");
     rqst->contentSize=strlen(rqst->content);
  } else
  {
     if (rqst->MAXcontentSize<length)
     {
        length = rqst->MAXcontentSize;
        fprintf(stderr,"Error , not enough space to accomodate image \n");
     }
     memcpy(rqst->content,readContent,length);
     rqst->contentSize=length;
     free(readContent);
  }
  return 0;
}

void * prepare_tf_image(struct AmmServer_DynamicRequest  * rqst)
{
  unsigned int length=0;

  int i=system("/bin/bash -c \"cd ../../rgbd_acquisition/bin/frames/tf/ && rosrun tf view_frames && convert frames.pdf frames.png\" ");

  char * readContent = 0;
  if (i==0) {  readContent = AmmServer_ReadFileToMemory((char*) "../../rgbd_acquisition/bin/frames/tf/frames.png",&length);    }

  if(readContent==0)
  {
     snprintf(rqst->content,rqst->MAXcontentSize,"<html><body><h1>Bad Image</h1></body></html>");
     rqst->contentSize=strlen(rqst->content);
  } else
  {
     if (rqst->MAXcontentSize<length)
     {
        length = rqst->MAXcontentSize;
        fprintf(stderr,"Error , not enough space to accomodate image \n");
     }
     memcpy(rqst->content,readContent,length);
     rqst->contentSize=length;
     free(readContent);
  }
  return 0;
}

void joystickExecute(float x , float y )
{
   AmmServer_Warning("Joystick(%0.2f,%0.2f)\n",x,y);

   char commandToRun[MAX_COMMAND_SIZE]={0};
    snprintf(commandToRun,MAX_COMMAND_SIZE,                              //        Please note that 1 is needed to emulate joystick move trigger
           "rostopic pub /joy sensor_msgs/Joy \"{ axes: [ %0.2f, %0.2f, 0.0, 0.0, 0.0, 0.0 ] , buttons: [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0] }\" -1&"
            ,x,y
           );

    int i=system(commandToRun);
    if (i!=0) { AmmServer_Error("Command %s failed\n",commandToRun); } else
               { AmmServer_Success("Command %s success\n",commandToRun); }

}


/*
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
                     IF SOMEONE WANTS TO ADD SOMETHING JUST ADD TO THE FOLLOWING ( execute("command","param" )
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
*/

void rosservice_call(char * commandToRun,unsigned int maxCommandSize,char * serviceName)
{
  snprintf(commandToRun,maxCommandSize,"/bin/bash -c \"rosservice call %s\"",serviceName);
}


void rosparam_set(char * commandToRun,unsigned int maxCommandSize,char * paramName,char * paramValue)
{
  snprintf(commandToRun,maxCommandSize,"/bin/bash -c \"rosparam set %s \"%s\" \"",paramName,paramValue);
}


void rostopic_pub(char * commandToRun,unsigned int maxCommandSize,char * topicName,char * topicType,char * topicValue)
{
  snprintf(commandToRun,maxCommandSize,"/bin/bash -c \"rostopic pub %s %s %s -1\"",topicName,topicType,topicValue);
}

void MMUIExecute(char * command,char * param)
{
  char * commandToRun = (char*) malloc((MAX_COMMAND_SIZE+1) * sizeof(char));
  if (commandToRun==0) { AmmServer_Error("Could not allocate enough space for MMUI command execution"); return ; }

  commandToRun[0]=0;
  fprintf(stderr,"MMUIExecute(%s,%s)\n",command,param);


  //rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, F_VOICE],[Value, "Susan"]]}'
  if (strcmp(command,"LuiBackgroundSelector")==0) { snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, F_SETBG],[Value, \\\"%s\\\"]]}' \" ",param); }  else
  if (strcmp(command,"talkingSpeed")==0) { unsigned int speedInt = 10*atoi(param);
                                           snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, F_SPEED],[Value, \\\"%u\\\"]]}' \" ",speedInt); }  else
  if (strcmp(command,"voiceFemale")==0)  { snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, F_VOICE],[Value, \\\"%s\\\"]]}' \" ",param); }  else
  if (strcmp(command,"voiceMale")==0)    { snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, F_VOICE],[Value, \\\"%s\\\"]]}' \" ",param); } else
  if (strcmp(command,"volume")==0)       { unsigned int volumeInt = 10*atoi(param);
                                           snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, F_ABSVOLUME],[Value, \\\"%u\\\"]]}' \" ",volumeInt); }
  if (strcmp(command,"say")==0)          { snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, D_OK],[Text, \\\"%s\\\"],[Speak, \\\"%s\\\"],[\\\"wait\\\", \\\"1\\\"]]}'\"",param,param); } else
  if (strcmp(command,"ask")==0)              { snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, D_YESNO],[Text, \\\"%s\\\"],[Speak, \\\"%s\\\"],[\\\"wait\\\", \\\"1\\\"]]}'\"",param,param); }
  if (strcmp(command,"signalNameUpdate")==0) { snprintf(commandToRun,MAX_COMMAND_SIZE,"/bin/bash -c \"rosservice call MMUI '{header: auto, sessionID: abc, requestText: create, params: [[Type, F_NAMESUPDATE],[Value, \\\"%s\\\"]]}' \"",param); }

  if ( strlen(commandToRun)!=0 )
   {
     int i=system(commandToRun);
     if (i!=0) { AmmServer_Error("Command %s failed\n",commandToRun); } else
               { AmmServer_Success("Command %s success\n",commandToRun); }
   } else
   {
     fprintf(stderr,"Execute with unknown command or parameter\n");
   }


  free(commandToRun);
}

int shutdownRaspberryPi()
{
     int i=system("../../hobbit_launch/launch/System/systemCommands shutdownPi");
     if (i!=0)
        {
          AmmServer_Error("Shutting down the raspberry pi appears to have failed\n");
        }
         else
        {
          AmmServer_Success("Shutting down the raspberry pi appears to have succeeded , waiting a little for it to shutdown before returning \n");
          //Sleep for 1.0 sec , could also ping raspberry here to see if it still alive ( or not )
          usleep(1500*1000);
          return 1;
        }
  return 0;
}





int executeSystem(char * what2Execute)
{
 int i=system(what2Execute);
 if (i!=0) { AmmServer_Error("executeSystem command %s failed\n",what2Execute); } else
           { AmmServer_Success("executeSystem command %s success\n",what2Execute); }
 return (i==0);
}



void execute(char * command,char * param)
{
  fprintf(stderr,"Execute(%s,%s) \n",command,param);
  unsigned int cRLen=MAX_COMMAND_SIZE;
  char * cR = (char*) malloc((MAX_COMMAND_SIZE+1) * sizeof(char));

  if (cR==0) { AmmServer_Error("Could not allocate enough space for command execution"); return ; }
  cR[0]=0;

  //-------------------------------------------------
  // ULTRA UNSAFE , INJECTION PRONE PARAMS HERE
  //-------------------------------------------------
  #warning "This code is injection prone , there needs to be sanitization for param , that unfortunately I haven't done yet"

  if (strcmp(command,"signalNameUpdate")==0)      { MMUIExecute(command,param); }  else
  if (strcmp(command,"setRobotName")==0)           {
                                                    rosparam_set(cR,cRLen,(char *) "/Hobbit/robot_name",param);
                                                    executeSystem(cR);
                                                    rosparam_set(cR,cRLen,(char *) "/ROBOT/robot_name",param);
                                                  }  else
  if (strcmp(command,"setUserName")==0)          {
                                                    rosparam_set(cR,cRLen,(char *) "/Hobbit/user_name",param);
                                                    executeSystem(cR);
                                                    rosparam_set(cR,cRLen,(char *) "/ROBOT/user_name",param);
                                                  }  else
  if (strcmp(command,"setSocialRole")==0)         { rosparam_set(cR,cRLen,(char *) "/Hobbit/social_role",param);     }  else
  if (strcmp(command,"askForSocialRole")==0)      { rostopic_pub(cR,cRLen,(char *) "/Command", (char *) "hobbit_msgs/Command", (char *) "\\\"{command: 'C_SOCIALROLE' , params: [ {name: 'facilitator' , value: 'true'} ] }\\\"" );   }  else
  if (strcmp(command,"setUserAway")==0)           { rosparam_set(cR,cRLen,(char *) "/Hobbit/user_away",param);       }  else
  if (strcmp(command,"setCurrentEmotion")==0)     { rosparam_set(cR,cRLen,(char *) "/Hobbit/current_emotion",param); }  else
  if (strcmp(command,"setLatestWakingUpTime")==0) { rosparam_set(cR,cRLen,(char *) "/Hobbit/wakeup_time",param);     }  else
  if (strcmp(command,"setLatestSleepingTime")==0) { rosparam_set(cR,cRLen,(char *) "/Hobbit/sleep_time",param);      }  else
  //---------------
  if (strcmp(command,"LuiBackgroundSelector")==0) {
                                                    char hexValue[32]={0};
                                                    if (param[0]=='#') { strncpy(hexValue,param,32);        } else
                                                                       { snprintf(hexValue,32,"#%s",param); }
                                                    fprintf(stderr,"Background Color was `%s` , it now is `%s` which should have a #XXXXXX format \n",param,hexValue);
                                                    MMUIExecute(command,hexValue);
                                                    if (param[0]=='#') { strncpy(hexValue,param+1,31);        } else
                                                                       { strncpy(hexValue,param,32); }
                                                    fprintf(stderr,"Background Color was `%s` , it now is `%s` which should have a XXXXXX format \n",param,hexValue);
                                                    rosparam_set(cR,cRLen,(char *) "USER/BGCOLOUR",hexValue); // rosparam set doesn't like #
                                                  }  else
  if (strcmp(command,"talkingSpeed")==0)          { MMUIExecute(command,param); rosparam_set(cR,cRLen,(char *) "USER/Voice/Speed",param);   }  else
  if (strcmp(command,"gender")==0)                { MMUIExecute(command,param); rosparam_set(cR,cRLen,(char *) "USER/Voice/Default",param); }  else
  if (strcmp(command,"voiceFemale")==0)           { MMUIExecute(command,param); rosparam_set(cR,cRLen,(char *) "USER/Voice/FEMALE",param);  }  else
  if (strcmp(command,"voiceMale")==0)             { MMUIExecute(command,param); rosparam_set(cR,cRLen,(char *) "USER/Voice/MALE",param);    }  else
  if (strcmp(command,"volume")==0)                { MMUIExecute(command,param); rosparam_set(cR,cRLen,(char *) "USER/Voice/Volume",param);  }  else


  // ULTRA UNSAFE , INJECTION PRONE PARAMS HERE
  //-------------------------------------------------
  if (strcmp(command,"node")==0)
  {
    if (strcmp(param,"webintfEnableBattery")==0)       { enableBatteryPrompt=1;    }   else
    if (strcmp(param,"webintfDisableBattery")==0)      { enableBatteryPrompt=0;    }   else

    if (strcmp(param,"switchToMapping")==0)      { strncpy(cR,"../../hobbit_launch/launch/switchMira.sh mapping",cRLen);    }   else
    if (strcmp(param,"switchToLearning")==0)     { strncpy(cR,"../../hobbit_launch/launch/switchMira.sh learning",cRLen);   }   else
    if (strcmp(param,"switchToNavigation")==0)   { strncpy(cR,"../../hobbit_launch/launch/switchMira.sh navigation",cRLen); }   else

    if (strcmp(param,"niteTrigger")==0)      {  rosservice_call(cR,cRLen,(char *) "/rgbd_acquisition/trigger_peopletracker");   } else
    if (strcmp(param,"nitePause")==0)        {  rosservice_call(cR,cRLen,(char *) "/rgbd_acquisition/pause_peopletracker");     } else
    if (strcmp(param,"niteResume")==0)       {  rosservice_call(cR,cRLen,(char *) "/rgbd_acquisition/resume_peopletracker");    } else
    if (strcmp(param,"nitePausePoint")==0)   {  rosservice_call(cR,cRLen,(char *) "/rgbd_acquisition/pause_pointing_gesture_messages");    } else
    if (strcmp(param,"niteResumePoint")==0)  {  rosservice_call(cR,cRLen,(char *) "/rgbd_acquisition/resume_pointing_gesture_messages");   } else

    if (strcmp(param,"vsPause")==0)                {  rosservice_call(cR,cRLen,(char *) "/vision_system/pauseEverything");   } else
    if (strcmp(param,"vsResume")==0)               {  rosservice_call(cR,cRLen,(char *) "/vision_system/resumeEverything");   } else
    if (strcmp(param,"vsFollowUser")==0)           {  rosservice_call(cR,cRLen,(char *) "/vision_system/followUser");   } else
    if (strcmp(param,"vsLocateUser")==0)           {  rosservice_call(cR,cRLen,(char *) "/vision_system/locateUser");   } else
    if (strcmp(param,"vsFitnessFunction")==0)      {  rosservice_call(cR,cRLen,(char *) "/vision_system/fitnessFunction");   } else
    if (strcmp(param,"vsWhereIsUserPointing")==0)  {  rosservice_call(cR,cRLen,(char *) "/vision_system/seeWhereUserIsPointing");   } else
    if (strcmp(param,"vsNavigating")==0)           {  rosservice_call(cR,cRLen,(char *) "/vision_system/navigating");   } else
    if (strcmp(param,"vsIdle")==0)                 {  rosservice_call(cR,cRLen,(char *) "/vision_system/idle");   } else
    if (strcmp(param,"vsScan3DObject")==0)         {  rosservice_call(cR,cRLen,(char *) "/vision_system/scan3DObject");   } else


    if (strcmp(param,"forthSKStartVisualization")==0)     {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/visualize_on");        } else
    if (strcmp(param,"forthSKStopVisualization")==0)      {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/visualize_off");       } else
    if (strcmp(param,"forthSKPause")==0)     {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/pause");        } else
    if (strcmp(param,"forthSKResume")==0)    {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/resume");       } else
    if (strcmp(param,"forthSKAdvanced")==0)  {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/advanced");     } else
    if (strcmp(param,"forthSKSimple")==0)    {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/simple");       } else

    if (strcmp(param,"startRecording")==0)   {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/startDump");    } else
    if (strcmp(param,"stopRecording")==0)    {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/stopDump");     } else
    if (strcmp(param,"clearRecording")==0)   {  rosservice_call(cR,cRLen,(char *) "/skeleton_detector/clearDump");    } else

    if (strcmp(param,"emergencyStartVisualization")==0)     {  rosservice_call(cR,cRLen,(char *) "/emergency_detector/visualize_on");        } else
    if (strcmp(param,"emergencyStopVisualization")==0)      {  rosservice_call(cR,cRLen,(char *) "/emergency_detector/visualize_off");       } else
    if (strcmp(param,"emergencyTrigger")==0) {  rosservice_call(cR,cRLen,(char *) "/emergency_detector/trigger");    } else
    if (strcmp(param,"emergencyPause")==0)   {  rosservice_call(cR,cRLen,(char *) "/emergency_detector/pause");      } else
    if (strcmp(param,"emergencyResume")==0)  {  rosservice_call(cR,cRLen,(char *) "/emergency_detector/resume");     } else

    if (strcmp(param,"gesturePause")==0)              {  rosservice_call(cR,cRLen,(char *) "/hand_gestures/pause");          } else
    if (strcmp(param,"gestureResume")==0)             {  rosservice_call(cR,cRLen,(char *) "/hand_gestures/resume");         } else
    if (strcmp(param,"gestureStartVisualization")==0) {  rosservice_call(cR,cRLen,(char *) "/hand_gestures/visualize_on");   } else
    if (strcmp(param,"gestureStopVisualization")==0)  {  rosservice_call(cR,cRLen,(char *) "/hand_gestures/visualize_off");  } else

    if (strcmp(param,"aggregatorPrecise")==0)         {  rosservice_call(cR,cRLen,(char *) "/person_aggregator/precise");          } else
    if (strcmp(param,"aggregatorRaw")==0)             {  rosservice_call(cR,cRLen,(char *) "/person_aggregator/raw");         } else
    if (strcmp(param,"aggregatorPause")==0)           {  rosservice_call(cR,cRLen,(char *) "/person_aggregator/pause");   } else
    if (strcmp(param,"aggregatorResume")==0)          {  rosservice_call(cR,cRLen,(char *) "/person_aggregator/resume");  } else

    if (strcmp(param,"followUserPause")==0)              {  rosservice_call(cR,cRLen,(char *) "/follow_user/pause");    } else
    if (strcmp(param,"followUserResume")==0)             {  rosservice_call(cR,cRLen,(char *) "/follow_user/resume");   } else
    if (strcmp(param,"followUserStartVisualization")==0) {  rosservice_call(cR,cRLen,(char *) "/follow_user/visualize_on");    } else
    if (strcmp(param,"followUserStopVisualization")==0)  {  rosservice_call(cR,cRLen,(char *) "/follow_user/visualize_off");   } else

    if (strcmp(param,"faceTrigger")==0)      {  rosservice_call(cR,cRLen,(char *) "/face_detection/trigger");  } else
    if (strcmp(param,"facePause")==0)        {  rosservice_call(cR,cRLen,(char *) "/face_detection/pause");    } else
    if (strcmp(param,"faceResume")==0)       {  rosservice_call(cR,cRLen,(char *) "/face_detection/resume");   } else

    if (strcmp(param,"fitnessTrigger")==0)      {  rosservice_call(cR,cRLen,(char *) "/fitness_coordinator/trigger");  } else
    if (strcmp(param,"fitnessPause")==0)        {  rosservice_call(cR,cRLen,(char *) "/fitness_coordinator/pause");    } else
    if (strcmp(param,"fitnessResume")==0)       {  rosservice_call(cR,cRLen,(char *) "/fitness_coordinator/resume");   }
    // Else there is an unknown node command
      else
    {  fprintf(stderr,"Unknown node command ( param %s ) \n", param); }

  } else
  if (strcmp(command,"camera")==0)
  {
    if (strcmp(param,"refreshTop")==0)
        { strncpy(cR,"/bin/bash -c \"cd ../../rgbd_acquisition/bin/frames/top/ && timeout 1 rosrun image_view image_saver image:=/headcam/rgb/image_rect_color\" ",cRLen); }
    if (strcmp(param,"refreshBottom")==0)
        { strncpy(cR,"/bin/bash -c \"cd ../../rgbd_acquisition/bin/frames/base/ && timeout 1 rosrun image_view image_saver image:=/basecam/rgb/image_raw\" ",cRLen); }
    if (strcmp(param,"refreshTF")==0)
      { strncpy(cR,"/bin/bash -c \"cd ../../rgbd_acquisition/bin/frames/tf/ && rosrun tf view_frames && convert frames.pdf frames.png\" ",cRLen); }

  } else
  if (strcmp(command,"head")==0)
  {
    if (strcmp(param,"default")==0)       { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"center_center\"");  } else
    if (strcmp(param,"restart")==0)       { rostopic_pub(cR,cRLen,(char *) "/head/cmd ",(char *) "std_msgs/String",(char *) "\"restart\"");        } else
    if (strcmp(param,"start")==0)         { rostopic_pub(cR,cRLen,(char *) "/head/cmd ",(char *) "std_msgs/String",(char *) "\"startup\"");        } else
    //------------------
    if (strcmp(param,"up_right")==0)      { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"up_right\"");       } else
    if (strcmp(param,"up_center")==0)     { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"up_center\"");      } else
    if (strcmp(param,"up_left")==0)       { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"up_left\"");        } else
    if (strcmp(param,"center_right")==0)  { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"center_right\"");   } else
    if (strcmp(param,"center_center")==0) { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"center_center\"");  } else
    if (strcmp(param,"center_left")==0)   { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"center_left\"");    } else
    if (strcmp(param,"down_right")==0)    { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"down_right\"");     } else
    if (strcmp(param,"down_center")==0)   { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"down_center\"");    } else
    if (strcmp(param,"down_left")==0)     { rostopic_pub(cR,cRLen,(char *) "/head/move",(char *) "std_msgs/String",(char *) "\"down_left\"");      }
  }
   else
  if (strcmp(command,"emotion")==0)
  {
    if (strcmp(param,"happy")==0)     {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"HAPPY\"");       } else
    if (strcmp(param,"vhappy")==0)    {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"VHAPPY\"");      } else
    if (strcmp(param,"ltired")==0)    {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"LTIRED\"");      } else
    if (strcmp(param,"vtired")==0)    {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"VTIRED\"");      } else
    if (strcmp(param,"concerned")==0) {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"CONCERNED\"");   } else
    if (strcmp(param,"sad")==0)       {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"SAD\"");         } else
    if (strcmp(param,"wondering")==0) {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"WONDERING\"");   } else
    if (strcmp(param,"neutral")==0)   {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"NEUTRAL\"");     } else
    if (strcmp(param,"sleeping")==0)  {  rostopic_pub(cR,cRLen,(char *) "/head/emo",(char *) "std_msgs/String",(char *) "\"SLEEPING\"");    }
  }
   else
  if (strcmp(command,"hand")==0)
  {
    if (strcmp(param,"calibrate")==0) {  rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_ARM_REFERENCE'\""); }
  }
   else
  if (strcmp(command,"body")==0)
  {
    if (strcmp(param,"reset")==0)   { rosservice_call(cR,cRLen,(char *) "/reset_motorstop");  } else
    if (strcmp(param,"360")==0)     { rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Turn 360'\"");   } else
    if (strcmp(param,"360ccw")==0)  { rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Turn 360'\"");   } else
    if (strcmp(param,"360cw")==0)   { rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Turn -360'\"");  } else
    if (strcmp(param,"right")==0)   { rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Turn -30'\"");   } else
    if (strcmp(param,"left")==0)    { rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Turn 30'\"");    } else
    if (strcmp(param,"forward")==0) { rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Move 0.30'\"");  } else
    if (strcmp(param,"back")==0)    { rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Move -0.30'\"");  }else
    if (strcmp(param,"stop")==0)    {
                                      joystickExecute(0.0,0.0);
                                      rostopic_pub(cR,cRLen,(char *) "/DiscreteMotionCmd",(char *) "std_msgs/String",(char *) "\"data: 'Stop'\"");
                                    }
  }
   else
  if (strcmp(command,"bring")==0)
  {
    if (strcmp(param,"aspirin")==0) { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"{command: 'C_BRING' , params: [ {name: 'Name' , value: 'ASPIRIN'} ] }\""); } else
    if (strcmp(param,"sugar")==0)   { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"{command: 'C_BRING' , params: [ {name: 'Name' , value: 'SUGAR'} ] }\""); }
  }
   else
  if (strcmp(command,"move")==0)
  {
    if (strcmp(param,"charging")==0)   { /*TODO IMPLEMENT THIS*/ } else
    if (strcmp(param,"kitchen")==0)    { /*TODO IMPLEMENT THIS*/ } else
    if (strcmp(param,"livingroom")==0) { /*TODO IMPLEMENT THIS*/ } else
    if (strcmp(param,"bedroom")==0)    { /*TODO IMPLEMENT THIS*/ }
  }
   else
  if (strcmp(command,"robot")==0)
  {
    if (strcmp(param,"systemRestart")==0)      { strncpy(cR,"../../hobbit_launch/launch/System/systemCommands restart",cRLen); }  else
    if (strcmp(param,"systemShutdown")==0)
         {
           //When we shut down we also need to shutdown the raspberry pi's and this requires us to add another command
           shutdownRaspberryPi();
           //Assuming that the raspberry pi is off we can then shutdown XPC
           strncpy(cR,"../../hobbit_launch/launch/System/systemCommands shutdown",cRLen);
         } else
    if (strcmp(param,"systemUpdate")==0)       { strncpy(cR,"../../hobbit_launch/launch/System/update.sh",cRLen); }   else
    if (strcmp(param,"systemUpdateGlobal")==0) { strncpy(cR,"../../hobbit_launch/launch/System/systemCommands updateGlobal",cRLen); }   else
    if (strcmp(param,"systemCleanROS")==0)     { strncpy(cR,"rm -rf /localhome/demo/.ros/log",cRLen); }   else

    //-------------------------------
    if (strcmp(param,"yes")==0)    { rostopic_pub(cR,cRLen,(char *) "/Event",(char *) "hobbit_msgs/Event",(char *) "\"event: 'G_YES'\"");           } else
    if (strcmp(param,"no")==0)     { rostopic_pub(cR,cRLen,(char *) "/Event",(char *) "hobbit_msgs/Event",(char *) "\"event: 'G_NO'\"");            } else
    if (strcmp(param,"stop")==0)   { rostopic_pub(cR,cRLen,(char *) "/Command",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_STOP'\"");   } else
    if (strcmp(param,"reward")==0) { rostopic_pub(cR,cRLen,(char *) "/Command",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_REWARD'\""); } else
    if (strcmp(param,"cancel")==0) { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"{command: 'C_SPEAK' , params: [ {name: 'CANCEL' , value: ''} ] }\""); } else
    //-------------------------------
    if (strcmp(param,"call")==0)     { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'E_CALLHOBBIT'\""); } else
    if (strcmp(param,"settings")==0) { rostopic_pub(cR,cRLen,(char *) "/Command",(char *) "hobbit_msgs/Command",(char *) "\"command: 'F_SETTINGS'\""); } else
    if (strcmp(param,"closemic")==0) { rostopic_pub(cR,cRLen,(char *) "/Command",(char *) "hobbit_msgs/Command",(char *) "\"command: 'F_ASR_OFF'\""); } else
    if (strcmp(param,"openmic")==0)  { rostopic_pub(cR,cRLen,(char *) "/Command",(char *) "hobbit_msgs/Command",(char *) "\"command: 'F_ASR_ON'\""); } else
    //-------------------------------
    if (strcmp(param,"hobbit")==0)      { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_WAKEUP'\"");      } else
    if (strcmp(param,"wake")==0)        { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_WAKEUP'\"");      } else
    if (strcmp(param,"sleep")==0)       { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_SLEEP'\"");       } else
    if (strcmp(param,"clearfloor")==0)  { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_CLEARFLOOR'\"");  } else
    if (strcmp(param,"bringobject")==0) { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_BRING'\"");       } else
    //-------------------------------
    if (strcmp(param,"learnobject")==0) { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'C_LEARN'\"");       } else
    if (strcmp(param,"helpme")==0)      { rostopic_pub(cR,cRLen,(char *) "/ActionSequence",(char *) "hobbit_msgs/Command",(char *) "\"command: 'G_FALL'\"");        }
  }
   else
  if (strcmp(command,"say")==0)
  {
     char internalString[MAX_COMMAND_SIZE+1]={0};
     strncpy(internalString,param,cRLen);
     replaceChar(internalString,'+',' ');

     #if USE_OLD_SAY_COMMAND
      //rostopic pub /ActionSequence HobbitMsgs/Command "{command: 'C_SPEAK' , params: [ name: 'INFO' , value: 'lobbit' ] }" -1
      snprintf(cR,cRLen,"rostopic pub /ActionSequence hobbit_msgs/Command \"{command: 'C_SPEAK' , params: [ {name: 'INFO' , value: '%s'} ] }\" -1\n",internalString);
     #else
      MMUIExecute((char*) "say",internalString);
        cR[0]=0;
      //Dont return because it leaks memory -> return ;
     #endif // USE_OLD_SAY_COMMAND
  }

  if ( strlen(cR)!=0 )
   {
     int i=system(cR);
     if (i!=0) { AmmServer_Error("Command %s failed\n",cR); } else
               { AmmServer_Success("Command %s success\n",cR); }
   } else
   {
     fprintf(stderr,"Execute with unknown command or parameter\n");
   }

   if (cR!=0) { free(cR); }

  return ;
}


/*
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
                     IF SOMEONE WANTS TO ADD SOMETHING JUST ADD TO THE PREVIOUS FUNCTION  ( execute("command","param" )
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
      ===========================================================================================================================
*/



//This function prepares the content of  form context , ( content )
void * store_new_configuration_callback(struct AmmServer_DynamicRequest  * rqst)
{
  fprintf(stderr,"store_new_configuration_callback called\n");

  unsigned int signalNamesChanged=0;
  unsigned int successfullStore = 0;
  unsigned int fullSetOperation = 0;
  unsigned int genderIsMale=0;
  int i=0;

  if  ( rqst->GET_request != 0 )
    {
      if ( strlen(rqst->GET_request)>0 )
       {
         if ( AmmServer_SaveDynamicRequest((char*) "hobbit_raw.ini",default_server,rqst) )
           {

               int i=system("tr \"\\&\" \"\\n\" < hobbit_raw.ini > hobbit.ini");
               if (i==0) {  successfullStore = 1; }
           }
       }
    }

  if  ( rqst->GET_request != 0 )
    {
      if ( strlen(rqst->GET_request)>0 )
       {
         AmmServer_Warning("Setting variables at once is unsafe since it doesnt check for injection");
         char * bufferCommand = (char *) malloc ( SMALL_CMD_BUF * sizeof(char) );
         if (bufferCommand!=0)
          {
            if ( _GET(default_server,rqst,(char*)"fullSetOperation",bufferCommand,SMALL_CMD_BUF) )
                  {
                    AmmServer_Warning("Full Set Operation..!");
                    fullSetOperation=1;
                    i=system("cp startupClean.dcfg startup.dcfg");
                    if (i==0) {  successfullStore = 1; }
                  }
            if ( _GET(default_server,rqst,(char*)"LuiBackgroundSelector",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"LuiBackgroundSelector",bufferCommand);  }
            if ( _GET(default_server,rqst,(char*)"userName",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"setUserName",bufferCommand); signalNamesChanged=1; }
            if ( _GET(default_server,rqst,(char*)"robotName",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"setRobotName",bufferCommand); signalNamesChanged=1; }
            if ( _GET(default_server,rqst,(char*)"socialRole",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"setSocialRole",bufferCommand);  }
            if ( _GET(default_server,rqst,(char*)"askForSocialRole",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"askForSocialRole",bufferCommand);  }

            if ( _GET(default_server,rqst,(char*)"userAway",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"setUserAway",bufferCommand);  }
            if ( _GET(default_server,rqst,(char*)"currentEmotion",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"setCurrentEmotion",bufferCommand);  }
            if ( _GET(default_server,rqst,(char*)"latestWakingUpTime",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"setLatestWakingUpTime",bufferCommand);  }
            if ( _GET(default_server,rqst,(char*)"latestSleepingTime",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"setLatestSleepingTime",bufferCommand);  }


            if ( _GET(default_server,rqst,(char*)"talkingSpeed",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"talkingSpeed",bufferCommand);  }
            if ( _GET(default_server,rqst,(char*)"gender",bufferCommand,SMALL_CMD_BUF) )
                   {
                     if (strcasecmp("MALE",bufferCommand)==0)   { genderIsMale=1; } else
                     if (strcasecmp("FEMALE",bufferCommand)==0) { genderIsMale=0; }
                     execute((char*)"gender",bufferCommand);
                    }
            if ( _GET(default_server,rqst,(char*)"voiceFemale",bufferCommand,SMALL_CMD_BUF) )
                   {
                     if (!fullSetOperation)  { execute((char*)"voiceFemale",bufferCommand);     } else
                     if (genderIsMale)       { AmmServer_Warning("Hiding Female voice command\n"); } else
                                             { execute((char*)"voiceFemale",bufferCommand);     }
                   }
            if ( _GET(default_server,rqst,(char*)"voiceMale",bufferCommand,SMALL_CMD_BUF) )
                   {
                     if (!fullSetOperation)  { execute((char*)"voiceMale",bufferCommand);     } else
                     if (!genderIsMale)      { AmmServer_Warning("Hiding Male voice command\n"); } else
                                             { execute((char*)"voiceMale",bufferCommand);     }
                   }
            if ( _GET(default_server,rqst,(char*)"volume",bufferCommand,SMALL_CMD_BUF) )  { execute((char*)"volume",bufferCommand);  }

            char * commandToRun = (char*) malloc((MAX_COMMAND_SIZE+1) * sizeof(char));
            if (commandToRun!=0)
            {
              if ( _GET(default_server,rqst,(char*)"LuiBackgroundSelector",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXCOLORXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"city",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXCITYXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"voice",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXVOICEXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"emergencyContactTel1",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXEMERGENCYTELXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"contactTel1",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE1NUMXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"contactName1",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE1NAMEXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }

              if ( _GET(default_server,rqst,(char*)"contactTel2",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE2NUMXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"contactName2",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE2NAMEXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }

              if ( _GET(default_server,rqst,(char*)"contactTel3",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE3NUMXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"contactName3",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE3NAMEXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }

              if ( _GET(default_server,rqst,(char*)"contactTel4",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE4NUMXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"contactName4",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE4NAMEXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }

              if ( _GET(default_server,rqst,(char*)"contactTel5",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE5NUMXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"contactName5",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE5NAMEXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }

              if ( _GET(default_server,rqst,(char*)"contactTel6",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE6NUMXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }
              if ( _GET(default_server,rqst,(char*)"contactName6",bufferCommand,SMALL_CMD_BUF) )  {  snprintf(commandToRun,MAX_COMMAND_SIZE,"sed -i 's/XXXXXTELEPHONE6NAMEXXXXX/%s/' startup.dcfg",bufferCommand); i=system(commandToRun);  }

              free(commandToRun);
            }




            free(bufferCommand);
          }
        }
     }


//Save rosparameters set
i=system("rosparam save /opt/ros/hobbit_hydro/src/hobbit_params.yaml");
if (i==0)
  {
   successfullStore=1;
  }

if (signalNamesChanged)
{
  execute((char*) "signalNameUpdate",(char*) "no_parameter");
}


if (successfullStore)
     {
         strncpy(rqst->content,"<html><head><meta http-equiv=\"refresh\" content=\"15; url=controlpanel.html\" /></head>\
                  <body><center><br><br><br><h1>Settings stored , dcfg file is ready..</h1><br><br><h3><a href=\"controlpanel.html\">You are beeing redirected to control panel</a><br>\
                   <a href=\"startup.dcfg\">Right click here and save as to download dcfg file</a></h3></body></html> ",rqst->MAXcontentSize);
     } else
     {
         strncpy(rqst->content,"<html><body><center><br><br><br><h1>FAILED to store settings :( </h1><br><br><h3><a href=\"#\" onclick=\"javascript:window.history.go(-1)\">Click here to go back</a></h3></body></html> ",rqst->MAXcontentSize);
     }

  rqst->contentSize=strlen(rqst->content);
  return 0;
}



//This function prepares the content of  form context , ( content )
void * prepare_form_content_callback(struct AmmServer_DynamicRequest  * rqst)
{
  strncpy(rqst->content,page,pageLength);
  rqst->content[pageLength]=0;
  rqst->contentSize=pageLength;


  if  ( rqst->GET_request != 0 )
    {
      if ( strlen(rqst->GET_request)>0 )
       {
         char * bufferCommand = (char *) malloc ( 256 * sizeof(char) );
         if (bufferCommand!=0)
          {
            if ( _GET(default_server,rqst,(char*)"node",bufferCommand,256) )  { execute((char*)"node",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"camera",bufferCommand,256) )  { execute((char*)"camera",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"head",bufferCommand,256) )  { execute((char*)"head",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"body",bufferCommand,256) )  { execute((char*)"body",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"hand",bufferCommand,256) )  { execute((char*)"hand",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"emotion",bufferCommand,256) )  { execute((char*)"emotion",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"body",bufferCommand,256) )
                 {
                     if (strcmp(bufferCommand,"joystick")==0)
                         {
                             float x=0.0,y=0.0;
                             char xString[256]={0};
                             char yString[256]={0};

                             if ( _GET(default_server,rqst,(char*)"x",xString,256) ) { x=atof(xString); } else { AmmServer_Warning("Could not find X coord"); }
                             if ( _GET(default_server,rqst,(char*)"y",yString,256) ) { y=atof(yString); } else { AmmServer_Warning("Could not find Y coord"); }

                             joystickExecute(x,y);
                           //Parse joystick command
                         } else
                         {
                           execute((char*)"body",bufferCommand);
                         }
                 } else
            if ( _GET(default_server,rqst,(char*)"bring",bufferCommand,256) )  { execute((char*)"bring",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"move",bufferCommand,256) )  { execute((char*)"move",bufferCommand);  } else
            if ( _GET(default_server,rqst,(char*)"robot",bufferCommand,256) ) { execute((char*)"robot",bufferCommand); } else
            if ( _GET(default_server,rqst,(char*)"rtd",bufferCommand,256) ) { execute((char*)"rtd",bufferCommand); } else
            if ( _GET(default_server,rqst,(char*)"say",bufferCommand,256) )   { execute((char*)"say",bufferCommand);   } else
                                                                              {  AmmServer_Error("Could not understand command to be executed ( ? not updated binary ? )\n"); }

            free(bufferCommand);
          }

       }
    }


  return 0;
}

//This function adds a Resource Handler for the pages stats.html and formtest.html and associates stats , form and their callback functions
int init_dynamic_content()
{
  fprintf(stderr,"Please note that control panel , is only refreshed once per startup for min resource consumption\n");

  page=AmmServer_ReadFileToMemory((char*)"controlpanel.html",&pageLength);
  if ( (page==0) || (pageLength==0) ) { fprintf(stderr,"Cannot initialize dynamic content , controlpanel.html is missing or cannot be read\n"); return 0; }
  if (! AmmServer_AddResourceHandler(default_server,&form,(char*)"/controlpanel.html",webserver_root,pageLength+1,0,(void* ) &prepare_form_content_callback,SAME_PAGE_FOR_ALL_CLIENTS) )
               { AmmServer_Error("Failed adding control page\n"); }
  AmmServer_DoNOTCacheResourceHandler(default_server,&form);

  //startup.dcfg / startupClean.dcfg  should always be served fresh
  AmmServer_DoNOTCacheResource(default_server,(char*) WEBROOT "startup.dcfg");
  AmmServer_DoNOTCacheResource(default_server,(char*) WEBROOT "startupClean.dcfg");


  if (! AmmServer_AddResourceHandler(default_server,&indexPage,(char*)"/index.html",webserver_root,4096,0,(void*) &prepare_index_content_callback,SAME_PAGE_FOR_ALL_CLIENTS) )
              { AmmServer_Error("Failed adding index page\n"); }
  if (! AmmServer_AddResourceHandler(default_server,&stats,(char*) "/stats.html",webserver_root,8000,0,(void*) &prepare_stats_content_callback,SAME_PAGE_FOR_ALL_CLIENTS) )
              { AmmServer_Error("Failed adding stats page\n"); }

  if (! AmmServer_AddResourceHandler(default_server,&settings,(char*)"/settings.html",webserver_root,4096,0,(void* ) &store_new_configuration_callback,SAME_PAGE_FOR_ALL_CLIENTS) )
              { AmmServer_Error("Failed adding settings page\n"); }

  if (! AmmServer_AddResourceHandler(default_server,&base_image,(char*)"/base_image.jpg",webserver_root,640*480*3,0,(void* ) &prepare_base_image,SAME_PAGE_FOR_ALL_CLIENTS) )
              { AmmServer_Error("Failed adding prepare base_image page\n"); }

  if (! AmmServer_AddResourceHandler(default_server,&top_image,(char*)"/top_image.jpg",webserver_root,640*480*3,0,(void* ) &prepare_top_image,SAME_PAGE_FOR_ALL_CLIENTS) )
              { fprintf(stderr,"Failed adding prepare top_image page\n"); }

  if (! AmmServer_AddResourceHandler(default_server,&tf_image,(char*)"/tf.png",webserver_root,640*480*3,0,(void* ) &prepare_tf_image,SAME_PAGE_FOR_ALL_CLIENTS) )
              { fprintf(stderr,"Failed adding prepare tf_image page\n"); }

  if (! AmmServer_AddResourceHandler(default_server,&map_image,(char*)"/map.png",webserver_root,640*480*3,0,(void* ) &prepare_map_image,SAME_PAGE_FOR_ALL_CLIENTS) )
              { fprintf(stderr,"Failed adding prepare map_image page\n"); }



  return 1;
 }

//This function destroys all Resource Handlers and free's all allocated memory..!
void close_dynamic_content()
{
    AmmServer_RemoveResourceHandler(default_server,&indexPage,1);
    AmmServer_RemoveResourceHandler(default_server,&settings,1);
    AmmServer_RemoveResourceHandler(default_server,&form,1);
    AmmServer_RemoveResourceHandler(default_server,&base_image,1);
    AmmServer_RemoveResourceHandler(default_server,&top_image,1);

    AmmServer_RemoveResourceHandler(default_server,&stats,1);
    AmmServer_RemoveResourceHandler(default_server,&form,1);
}




int main(int argc, char **argv)
{
   ROS_INFO("Starting Up!!");

   int i=system("./videos/getVideos.sh");
   if (i!=0 ) { ROS_INFO("Could not check for video datasets");       } else
              { ROS_INFO("Successfully checked for video datasets");  }

   try
   {
     ROS_INFO("Initializing ROS");
     ros::init(argc, argv, NODE_NAME);
     ros::start();

     ros::NodeHandle nh;
     nhPtr = &nh;

     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer stopWebService     = nh.advertiseService("web_interface/terminate", terminate);

     ros::Rate loop_rate(1); //  1hz should be our target spin time


     //Is there some race condition with rosmaster ?
     unsigned int countdown=5;
     fprintf(stderr,"Giving some time to ROS before the multithreaded stuff kicks in\n");
     while (countdown>0)
     {
        ros::spinOnce();
        fprintf(stderr," %u \n",countdown);
        --countdown;
        loop_rate.sleep();
     }
     //---------------------------------------------




     printf("\nAmmar Server %s starting up..\n",AmmServer_Version());
     //Do not registering termination signal , AmmServer_RegisterTerminationSignal((void*) &close_dynamic_content);


     char bindIP[MAX_IP_STRING_SIZE];
     strcpy(bindIP,"0.0.0.0");

     unsigned int port=DEFAULT_BINDING_PORT;




    default_server = AmmServer_StartWithArgs(
                                             (char*) "hobbitWebInterface",
                                              argc,argv , //The internal server will use the arguments to change settings
                                              //If you don't want this look at the AmmServer_Start call
                                              bindIP,
                                              port,
                                              0, /*This means we don't want a specific configuration file*/
                                              webserver_root,
                                              templates_root
                                              );

    if (!default_server) { AmmServer_Error((char*) "Could not start server , shutting down everything.."); exit(1); }


    //Create dynamic content allocations and associate context to the correct files
    if ( init_dynamic_content() )
    {
       //pages should be availiable from now on..!
       //Create our context
       //---------------------------------------------------------------------------------------------------
	   //////////////////////////////////////////////////////////////////////////
	   while ( (AmmServer_Running(default_server))  && (ros::ok()) && (!stopWebInterface) )
		{
             ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread

             //Main thread should just sleep and let the background threads do the hard work..!
             //In other applications the programmer could use the main thread to do anything he likes..
             //The only caveat is that he would takeup more CPU time from the server and that he would have to poll
             //the AmmServer_Running() call once in a while to make sure everything is in order
             //usleep(60000);
             //sleep(1);

             //We now sleep using ROS mechanism for sleep..
             loop_rate.sleep();
		 }
      AmmServer_Warning("Web Interface is now Shutting down.. ");


     //Delete dynamic content allocations and remove stats.html and formtest.html from the server
     close_dynamic_content();
    }

    //Stop the server and clean state
    AmmServer_Stop(default_server);
    AmmServer_Warning("Ammar Server stopped\n");



	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
