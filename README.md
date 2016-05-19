##Night Guard Changes to jamorham/wixel-xDrip (Parakeet)

This fork contains small modifications to original project:
- added define, so that Night Guard WS is always called
- added parameter connector_uuid to WS call, so that device can be identified by WS
- configuration is moved to my_transmitter_id.h, file is created with all required parameters (which can be automaticaly genearated by Night Guard Website, so that all confuration is done there)


##What is the Parakeet?
The Parakeet is a portable home-built device which receives wireless signals from a commercial continuous glucose sensor worn on the body. It transmits these over the phone network to a private or cloud internet server.

Powered by either AAA or AA rechargeable batteries as shown below. AAA Maha batteries run for 2-3 days.

![](https://github.com/jamorham/zz-misc-jamorham/blob/master/blob/images/parakeet-aaa-and-aa-boxed-small.jpg)

###What is the Parakeet for?
The Parakeet is primarily designed to allow parents and carers to be able to monitor the blood sugar of a diabetic child even if they are a long distance away, for example, carried in the pocket of a school bag. The real-time blood sugar information would be available on the parent’s mobile phone — potentially many miles away.

##Building a Parakeet
Building the unit requires soldering the components together, locating them in a suitable enclosure and installing the software on the microprocessor. 

![](https://github.com/jamorham/zz-misc-jamorham/blob/master/blob/images/parakeet-wiring-snapshot-800.png)

###What makes this different to xDrip / Nightscout etc?
The Parakeet builds on the existing xDrip system with very similar design goals to Nightscout. The difference with this device is that it does not need a CGM receiver or Smartphone to be with the child in order to function. This is especially important due to school policies which prohibit mobile phones in classrooms. Even where mobile phones are not prohibited, there are advantages to the unit being independent.

###Additional features include:
* A new ultra-lightweight network protocol designed to minimize cellular data usage and keep running costs low.
* Configuration and status checking using standard text messaging. 
* Optional geo-positioning feature allows location tracking and machine learning potential.   

![](https://github.com/jamorham/zz-misc-jamorham/blob/master/blob/images/parakeet-r2-text-message-configure-818.png)

###How to get the Parakeet working?
The easiest way to receive data from the Parakeet is using the [Google App Engine receiver](https://github.com/jamorham/Parakeet-App-Engine) - Instructions for setting this up are here: [Parakeet & Google App Engine Guide (PDF)](https://drive.google.com/file/d/0B6mvYVNVC-fObUxVSmZmdF9FRnc/view?usp=sharing)

My personal research version of xDrip called xDrip+ has extra support for Parakeet features such as Geo-Location and Battery Status. [The xDrip+ project page is here.](https://jamorham.github.io/#xdrip-plus) Other links to information about the Parakeet including binary firmware images [are here.](https://jamorham.github.io/#parakeet)

![](https://github.com/jamorham/zz-misc-jamorham/blob/master/blob/images/jamorham-parakeet-sim800-sim-orientation-redux.jpg)

###Why is it called Parakeet?
The parakeet name was chosen for this unit as it is fully mobile (can fly like a bird) plus it listens and repeats the glucose sensor information (like parakeet speech mimicry). Shorter unique “nicknames” make referencing technical items easier and reduce confusion. The pet animal concept can make it more emotionally appealing, especially to children. The segments on the wings in the logo are a representation of the Yagi antenna of the Wixel board and the GSM antenna.

##Further Information
To read more about the Parakeet design objectives, the research project of which it is a part, as well as safety and contact information, please see: [What is the Parakeet (PDF)](https://drive.google.com/file/d/0B6mvYVNVC-fOQU5XQS14NERwYjA/view?usp=sharing)

###Thanks
None of this would be possible without the extensive work of many others who have produced the open source tools upon which this builds directly.  There are too many to credit individually; thanks goes to everyone involved in xDrip and Nightscout.
