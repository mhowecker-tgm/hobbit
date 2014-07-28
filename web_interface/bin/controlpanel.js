
   var centerX=51;
   var centerY=54;
   
   var speedMultiplier=0.04; //Normal Speed selected by default 
   
   var deadZoneSize = 6; 
   var deadZoneLeft = centerX-deadZoneSize;
   var deadZoneUp = centerY-deadZoneSize;
   var deadZoneRight = centerX+deadZoneSize;
   var deadZoneBottom = centerY+deadZoneSize;

   var limitLeft = 25;
   var limitUp = 27;
   var limitRight = 75;
   var limitBottom = 78;
 
   var ledX = 5;
   var ledY = 5;

   function getCommandPartOfURL(str) 
   {
    return str.split('?')[1];
   }

   function makeUniqueURLs()
   {  
     var links = document.links;
     var i = links.length;
     //Simple var randomnumber=Math.floor(Math.random()*100000);
     
     while (i--) 
     {
       if (links[i].href.indexOf("javascript") == -1) 
          { 
            //Simple links[i].href = links[i].href+"&t="+randomnumber; 
            if (links[i].href.indexOf("controlpanel.html?") != -1) 
              {
                links[i].href = "javascript:command(\'"+getCommandPartOfURL(links[i].href)+"\');"; 
              }
          }
     } 
   }
 
    function refreshFeed(name,imageName)
    {
      command('camera=refresh');
      document.getElementById(name).style.visibility='visible';
      var randomnumber=Math.floor(Math.random()*100000);
      document.getElementById("videoFeedImage").src="base_image.jpg?t="+randomnumber;
    }

    function makeFeedVisible(name)
    {
      document.getElementById(name).style.visibility='visible';
    }

    function makeFeedInvisible(name)
    {
      document.getElementById(name).style.visibility='hidden';
    }

    function makeTurnControlsVisible() 
                               {
                                       document.getElementById("turnLeft").style.visibility='visible';
                                       document.getElementById("turnRight").style.visibility='visible';
                               }

    function makeTurnControlsInvisible() 
                               {
                                       document.getElementById("turnLeft").style.visibility='hidden';
                                       document.getElementById("turnRight").style.visibility='hidden';
                               }

    function handleJoystickActiveSwitch(cb)
    {
       if (cb.checked)
           {
             makeTurnControlsVisible();
           } else
           {
             makeTurnControlsInvisible();
           }
    }

   function httpGet(theUrl)
    {
    var xmlHttp = null;

    xmlHttp = new XMLHttpRequest();
    xmlHttp.open( "GET", theUrl, true ); //Second parameter is async
    xmlHttp.send( null );
    return xmlHttp.responseText;
    }

     function joystickExecute(joyX,joyY)
     {
        var randomnumber=Math.floor(Math.random()*100000);
        if ( (joyX==0) && (joyY==0) ) 
          {
            httpGet("controlpanel.html?body=stop&t="+randomnumber); 
          } else
          {
            httpGet("controlpanel.html?body=joystick&x="+joyX+'&y='+joyY+"&t="+randomnumber); 
          }
     }

    
     function command(theCommand)
     {
       var randomnumber=Math.floor(Math.random()*100000);
       httpGet("controlpanel.html?"+theCommand+"&t="+randomnumber); 
     } 

     function facilitatorCommand(theCommand)
     {
       var randomnumber=Math.floor(Math.random()*100000);
       httpGet("facilitator_panel.html?"+theCommand+"&t="+randomnumber); 
     } 


function updateStatusMonitor()
{
    var iframe = document.getElementById('statusMonitor');
    iframe.reload(true);
}

setTimeout('updateStatusMonitor()', 10000);

function updateFormRandomNumber()
    {
       var randomnumber=Math.floor(Math.random()*100000);
       document.getElementById("FormRandomField").value = randomnumber;
    }

 
function allowDrop(ev)
{
 //ev.preventDefault();
  return true;
}

function drag(ev)
{  
  ev.dataTransfer.setData("Text",ev.target.id);
}

function drop(ev)
{
  ev.preventDefault(); 
  var data=ev.dataTransfer.getData("Text");
  ev.target.appendChild(document.getElementById(data));
  return true;
} 