EDTracker UI V3.0.2


30/09/2014 - Fix auto-centre bug for exponential. Add more auto-centre options. Fix temperature display.



Requires .Net Framework 4.5 available from http://www.microsoft.com/en-us/download/details.aspx?id=30653

To Install:

Unzip somewhere.

To Run :

EDTrackerUI3.exe

What's New:

	It's written in .Net so should be much more stable and compatible across Windows PC's.

	COM Port drop down list shows full description of devices making it easier to spot attached EDTracker (Arduino Leanoardo/Micro etc).

	Four axis Chart shows relevent info.

	In Calibration Mode:

		Temperature Delta (change in temperature. Zero indicates stable temps)
		Raw X Gyro
		Raw Y Gyro
		Raw Z Gyro 

	Once temps are stable, adjust bias until the Raw Gyro vvalues are as close to zero as possible.

	In Main Sketch /Drift Compensation Mode:

		 Temperature Delta
		 Yaw Drift
		 Yaw Value (greatly magnified)
		 Pitch Value 

		 Allow to warm up and then click "Save drift compensation".  Aim for a Yaw Drift of less than 0.1;

		
	Hot Key Support (currently limited to Function keys 1 to 10 only)
		
		Just select required Hot Key and click the check-box.  Minimise the app (will sit in the System Tray and use minimal CPU).


Known Issues:

May crash if no com ports detected.
May hang if try to connect to a device that is not an EDTracker
Help not yet implemented

Coming Up Next:

Looking at joystick button binding for reset view.
Adding ability to adjust/turn off auto centring
Support for trackers build on MPU9150 (magnetometer).



