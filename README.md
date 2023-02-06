# HelloRestCar2
Requirements:   

-Arduino Uno WiFi REV2   
-4tronix Initio 4WD Robocar   
-Library:  WiFiNINA, NewPing, ArduinoHttpClient, SPI   


   
Add SSID.h with your ssid information:   

#define SSID      "Your gateway id"   

#define KEY       "Your gateway WPA pass"   

## Description:  

Car makes http-requests to webserver for fetching actions it's going to execute next.
It must implement all actions which are defined on web server backend client api.
Default implementation fo webserver is visible on https://robo.sukelluspaikka.fi

## Intro:  

Based on [Intio 4tronix building kit](https://4tronix.co.uk/blog/?p=169)  
    Robot car has two 170-size DC-motors,one for each side.  
    You can set the speed and direction separatelly on each motors,
    enabling it to move curve paths or even turn around in place.  
    Robot can be controlled by defining the distance of selected movement.  
    It has wheel encoders on bought sides doing distance measurement,
    on curve moves the longest paths is matched on given distance while 
    going straight, robot keeps track during the movement that bought sides 
     are progressing simultaniously so that straight movement is ensured.  
    Distance parameter on api is given in centimeters.  
    It also has two sonars (front and back) which are able to measure 
    a distansce to nearests obstacles ahead or back between 4-400cm.   
    Automatic stop  functionality has been added into movement algorithms 
    to avoid unnessecarry collitions and omitting the wished distance.  
    On top of the rear sonar three ledlights (greeen, yeallow and red) 
    indicates the obstacles on away. the indication is bind to the direction of movement
    if bought sides are moving forward, front sonar is doing the measurement, otherwise 
    rear sonar.  
    Robot can be set into 'manual mode' (api: /car/manual) for few minutes. It allows controlling robot manually in real time instead of pre written source code (default). In this mode the seqvense does not stop automatically after theres no source code left to execute. By setting the mode to auto or exceeding the four minute time limit, Robot return back to pre programmable mode.  
        
    Normally, once all the pre-defined actions in current project has been done, robot 
    signals the server asking to start next project waiting it's turn in que.  
       
    This Robot is a sand box implementation, operating in a single room. First test device for conseptual testing of CloudMachines.
