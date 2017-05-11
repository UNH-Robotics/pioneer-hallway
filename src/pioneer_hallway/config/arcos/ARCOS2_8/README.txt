Advanced Robot Control & Operations Software (ARCOS)
Copyright (c) 2004-2005 ActivMedia Robotics, LLC.
Copyright (c) 2006-2009 MobileRobots Inc.
All rights reserved.
=======================================================================

ARCOS is the embedded software (firmware) that runs your Pioneer 3-DX, -AT,
Performance PeopleBot, PowerBot and MapperBot SH2-based microcontroller.
Your robot's microcontroller also contains Maintenance Mode utilities
(ARCOSstub GDB interface) and robot operating parameters in its FLASH memory.

Use the ARCOScf tool to update your ARCOS firmware and to manage your robot's
operating FLASH parameters. Consult your robot's Operations Manual for details.

             =============IMPORTANT=============
Use ONLY the ARCOScf configuration program that comes with this
distribution to manage your ARCOS updates and robot FLASH parameters. 

Also, USB-to-serial converters don't often work with this program.
             ===================================

What's New in Version 2?
==================================
- uiChange FLASH parameter
- Directional rotation speeds
- IMU Precision Calibration
- BumpStall stallCount
- Support for new SPI-based gyro and IMU
- SPI Gyro ranging FLASH parameter and client command
- Gyro bit 4 in FLAGS
- Low-battery command and flag
- Freespace
- LCD-based calibration routine
- Fixed windup and stutter when max vels not achievable
- Fixed reconnection odometry error
- Cf program updates
- GyroRateLimit

========
uiChange
========
Added a FLASH parameter which, when zero, restricts LCD interactive
access to power down the PC or the LRF, to break a client connection, to
enter Maintenance Mode, or to engage Calibration Mode.

===========================
Directional rotation speeds
===========================
You may now set different speeds for clockwise (CW) versus counter-
clockwise (CCW) rotations of the robot with client command #14. Positive
arguments set the CCW speed in deg/sec; negative ones set the CW speed; zero
disables their effects; 1 blocks rotations CCW; -1 blocks CW rotations.
Neither directional speed can be faster than the last specified rotVelMax
(command #10). 

====================
Updated IMU
====================
The Analog Devices IMU accessory provides a precision calibration utility
built in to its firmware that provides slightly better performance for the
device. By setting the IMUCal parameter in FLASH to a 1, the IMU will perform
the precision calibration on start up or microcontroller reset. It takes
30 seconds, during which time you will not be able to establish a client-
server connection.

====================
BumpStall stallCount
====================
To have your robot quickly stop when a bumper gets triggered in the direction
of contact, you need to enable bumpStall. Do that in FLASH and/or by the
client command #44. The stall gets reflected in the respective bit 0 of the
stall bytes in the Standard SIP as well as the bumper bits in the IOpac.
A firmware-level reflexive behavior causes the robot to stop quickly and
disengage its motors during the stall. That works with earlier version.
But, like a motor stall, a bumper-related stall should've kept the motors
disabled for stallCount ms, then re-engage the motors. It didn't. With this
version and later ones, it does.

======================
SPI-based Gyro and IMU
======================
The new SPI-based gyro (hasGyro 3) and IMU (hasGyro 4) have been integrated
with the robot's firmware. Like the server-side analog gyro (hasGyro 2),
they use the gyroCW and gyroCCW calibration values and related gyroCalCW
and gyroCalCCW client calibration commands #38 and #39; and are turned
off/on with the client command #58. (The firmware uses the IMU's Z-axis
gyro for odometry; gyroCW and gyroCCW are typically in the range 730-750.) 

You also may request a new IMUpac SIP when using either the single-axis gyro
or multi-axis IMU. The SIP contains gyro, accelerometer and temperature
readings which the firmware automatically acquires for each axis over a 25ms
period. Request (command 26) one IMUpac with the command argument one, or have
the server send one after every standard SIP (argument > 1); 0 stops the
cycling.

IMUpac Contents:
Label	Value           Description
Header  0xFA,0xFB       Packet header
Count    byte           Byte count
ID       0x9A           SIP Type
timeSince byte	        Ms since standard SIP last sent (see below)
nX	byte            Readings per axis (depends on SIP speed;
			  typically 4 per cycle)
nG       byte 1|3       Number of gyro axes
For each nX oldest to youngest:
rG       byte 1|2|3    	Range 1=75/80, 2=150/160, or 3=300/320 deg/sec
aG    signed ints       Reading for each nG axis
nA       0|3            Number of accelerometer axes
For each nX oldest to youngest:
rA    signed ints       Reading for each nA axis
nT       1|3            Number of temperature axis
For each nX oldest to youngest:
rT    signed ints       Reading for each nT axis

You may use the command #27 with a 2-byte argument (device command followed
by the command value) to change the device settings during a client-server
session. For example, 27 185 2 sets the gyro(s) to a +-150/160 d/s range.

Consult the Analog Devices ADIS16250/ADIS16255 gyro or ADIS16355 IMU
specifications for further details.
 
Note that the new accessories interface with the microcontroller through the 
Gripper/User I/O connector. Consequently, they cannot be used with the Gripper.

=================
IMUpac Timing
=================
Although the firmware updates its internal odometry every 5 ms, it uses the
integrated gyros to correct that odometry every 25 ms, due to the device
bandwidth. Accordingly, careful adjusted synchronicity in the firmware has
the latest gyro reading acquired immediately (within 1 ms) before it's used
to compute the odometry, and which happens immediately before (within that
same 1 ms) the odometry gets reported in the standard SIP.

The IMUpac, when requested, gets sent immediately following the standard SIP.
One of the first data values in that packet is the timeSince byte, which is
the number of ms since the standard SIP was assembled and subsequently
transmitted to the client. Although the IMUpac immediately follows the
standard SIP, its timing will vary depending on the length of the standard
SIP and the communication baud rate.

Note also, that although one of the readings for each axis was completed
immediately prior to the standard SIP, the youngest reading in the IMUpac
may be acquired just before the IMUpac gets sent to the client if the
timeSince exceeds 25 ms.

For every 25ms IMU cycle:
-- Gyro X at t=2, 10 and 17ms
-- Gyro Y at t=4, 11 and 19ms
-- Gyro Z axis at t=3, 8, 13, 16, and 23ms
-- Temperature X at t=1ms
-- Temperature Y at t=9ms
-- Temperature Z at t=16ms
-- Accel X at t=1, 4, 7, 10, 13, 16, 19 and 22ms
-- Accel Y at t=2, 5, 8, 11, 14, 17, 20 and 23ms
-- Accel Z at t=3, 6, 9, 12, 15, 18, 21 and 24ms

=================
Gyro Ranging
=================
The SPI-based IMU and SAG both support varying gyro ranges: (IMU/SAG) 75/80,
150/160, and 300/320 degrees per second. Set the FLASH parameter GyroRange to
1, 2 or 3, respectively, to select a particular range. (Values 0 or >3 default
to the 300/320 d/s range.) When using an SPI-based gyro, your robot's maximum
rotational velocity, normally set by the RotVelTop FLASH parameter,
instead gets limited to 80-90% of the current top range of the gyro. Otherwise,
if your robot's rotVel exceeds the selected range, you would lose heading.

Use the client command #24 with argument 0, 1, 2 or 3 to change the ranging
on the fly. GyroRange reverts to the FLASH default on client disconnect.

EXPERIMENTAL: The command #24 value 0 has the firmware autoselect the
appropriate range the IMU or SAG uses while rotating: range 1 when below
55 deg/sec; range 2 when below 110 deg/sec, and range 3 otherwise.
 
======================
Gyro Faults and Status
======================
When disabled internally by a fault or externally by the client command #58
and argument 0, the server-side integrated gyros (hasGyro 2-4) now
set the bit 4 of the Fault (FLAGS2) int in the standard SIP.

When the integrated gyros are operating properly and are enabled (default
on startup, or when commanded successfully by #58 with argument 1), the bit
is off. Errors get reported on the LCD, too: A 'G' flashes on the LCD accessory
if the gyro is turned off intentionally or by fault.

Note that if the SPI data faults, the firmware automatically stops using the
device and will stop reporting IMU data, though requested IMU packets will
continue to stream. The gyro-related fault flag bit gets set, too. For some
errors, the SAG/IMU may restart itself, though the fault flag will appear in
at least one SIP cycle. If the device doesn't restart automatically, You
may be able to restart the gyro/IMU with the 58,1 command, or by disconnecting
from the server and reconnecting. The device resets upon disconnection from
the client. 

=================
Low-Battery
=================
Bit 3 of the FLAGS2 integer in the standard SIP gets set
when shutdown gets initiated by the low battery condition.

A new client command #54 with int argument of voltsX10 (120=12.0V, for example)
lets your software override the FLASH default low-battery value. Send an
argument of 0 or disconnect to revert to the default. 

==================
FreeSpace
==================
FreeSpace is the percent of free storage space available for maps and data
on the mapping PC times 100 (10,000=100.00%). Value must be sent from client
via command 102 for display on the LCD (replaces driving cart). Each incoming
freeSpace value is tested by the server against the freeSpace FLASH parameter
for low-space fault condition. Command 102 with value -1 disables the display.

==============
GyroRateLimit
==============
Due to drift and noise, especially with marginal gyros, spurious rates
can occur. For the new firmware-based gyro, the marginal rate was fixed
at 2 degrees per second, below which the robot's heading is based on encoder
feedback, not the gyro. With this version you may set that marginal
limit as a FLASH parameter or on-the-fly with the client command #25. Express
the limit as deg/sec X 10; 10 is 1 d/s, for example.

===========================
Calibration Routine
===========================
Three parameters intimately affect the performance of your mobile robot:
driftFactor, revCount and ticksMM. Two more if you have a gyro:
gyroCW and gyroCCW. Each of these are settable empirically to match your
individual robot and operating conditions, usually by running
the ARIA demo and measuring actual versus reported parametrics. With
the new firmware, a calibration tool is embedded in the firmware and accessible
through LCD Interactive mode to help you measure and compute these
operations parameters.

Please read the robot's documentation about the parameters, and follow the
LCD's onscreen instructions. You can drive the robot with a joystick or
just push it. You'll need a run of about 3-5m, and it helps to have a simple
laser line generator attached to the nose of your robot to help you track
straight lines and complete exact rotations.

Note that you must SAVE the derived values to FLASH with the cf program in
order to have them take effect.

==========================
Fixed Windup and Stutter
==========================
If your transVelMax and/or rotVelMax values are set above what your robot
can actually achieve, you may have noticed the platform stutter after driving
a long distance or rotating several revolutions. In extreme cases, the robot
might actually reverse direction for a short time. Also, after a long drive,
the robot might take much longer than expected to slow down and stop.

These behaviors happened because the ideal versus actual setpoints for
operation grew wildly apart, leading to very large windups and value over-
runs in the trajectory controller. These conditions are now recognized and
ameliorated in the firmware.
  
===========================
Reconnection Odometry Error
===========================
If you stop and restart the client-server connection quickly, such as in
the case when the client does not cleanly shut down the server before
disconnecting and then makes a reconnection, the server did not have time
to properly re-initialize its systems, particularly its odometry estimates,
leading to a "jump" in the odometry moments after reconnection. Now, the
server disallows reconnection until after it runs through its close-server
initialization loop.

==================
Cf Program Updates
==================
We've made several improvements to the configuration program. First,
it consults /etc/Aria.args (simple text file listing Aria startup arguments)
for the serial connection port. The -rp startup arguments still works, of
course, and takes precedence.

Now, too, the program warns you if you attempt to manage a later version of
the firmware. Firmware uploads by an older configuration program are okay,
but you could lose FLASH parameters.

Finally, you cannot use a configurator from a different platform; uARCScf
to upload/manage an ARCOS-based Pioneer 3-DX, for example.

===================================
ARCOS distributions for Release 2.7
===================================
Linux:
------
Normally located in /usr/local,'tar -zxvf ARCOS2_7.tgz' creates:
   ARCOS2_7/
	README (me)
	ARCOS2_7.mot (ARCOS image)
	ARCOScf (Download and configuration tool)
	mapperbot.rop (default FLASH parameters for MapperBot)
	p3dx.rop (default FLASH parameters for the P3DX)
	p3at.rop (default FLASH parameters for the P3AT)
	p3atiw.rop (default FLASH parameters for the P3AT with indoor wheels)
	peoplebot.rop (default FLASH parameters for the Performance PeopleBot)
	powerbot.rop (default FLASH parameters for the PowerBot)

Windows
-------
Double-click or otherwise execute the self-extracting archive ARCOS2_5.exe.
Creates:

   C:\Program Files\MobileRobots\ARCOS\
	README (me)
	ARCOS2_7.mot (ARCOS image)
	ARCOScf.exe (Download and configuration tool)
	mapperbot.rop (default FLASH parameters for MapperBot)
	p3dx.rop (default FLASH parameters for the P3DX)
	p3at.rop (default FLASH parameters for the P3AT)
	p3atiw.rop (default FLASH parameters for the P3AT with indoor wheels)
	peoplebot.rop (default FLASH parameters for the Performance PeopleBot)
	powerbot.rop (default FLASH parameters for the PowerBot)

=========================
Operating ARCOScf
=========================
Like previous robot configuration tools, ARCOScf runs on a Linux
or Windows PC and communicates with the robot controller through a serial
connection. Accordingly, either download and run ARCOScf on your robot's
embedded PC (recommended) or use a pass-through serial cable to connect your
PC's serial port--COM1 or /dev/ttyS0, by default--to the SYSTEM serial port
on your robot's User Control Panel.

ARCOScf Startup Arguments
----------------------------- 
-h                ; help message and exit
-rp [serial dev]  ; specify port if other than the default COM1 or /dev/ttyS0
-rb [baudrate]    ; specify initial HOST baudrate if other than the
 		    default 9600. ARCOScf eventually autobauds if you get
		    the rate wrong.
-n                ; don't connect with controller
-u <path/motfile> ; upload ARCOS motfile image to controller
		    must be a *.mot file
-l <ropfile>      ; load a Robot Operating Parameters (.rop) file
-s <ropfile>      ; saves the ROP to disk
-b <options>      ; execute non-interactive command options, then exit
-r <ropfile>      ; restore params from ROP file; uploads if batch mode
-f                ; ignore versioning

For example, to upload the new ARCOS image through your PC's serial port
COM3 (/dev/ttyS2):

(Linux) $ cd /usr/local/ARCOS$ ./ARCOScf -rp /dev/ttyS2 -u ARCOS2_7.mot

(Windows) Launch a DOS-like command window (cmd.exe), then
>cd C:\Program Files\MobileRobots\ARCOS>ARCOScf -rp COM3 -u ARCOS2_7.mot

Once connected, ARCOS lets you interactively manage FLASH and related files.

ARCOS Interaction Options
--------------------------
'?' or 'h' or 'help' to see menu again
'v' or 'view' for current parameters
'a' or 'arm' P2 Arm accessory values
'r [ropfile]' or 'restore [ropfile]' to restore FLASH or [ropfile] rop values
's [ropfile]' or 'save [ropfile]' to save changes to FLASH
   or to disk if [ropfile]
'u <motfile>' or 'upload <motfile>' to upload new ARCOS image
   (.mot file required)
'q' or 'quit' to exit

--------------------------------------------------------------------------
Contact tech support through http://robots.mobilerobots.com with questions,
concerns and/or comments.

=======HISTORY=======
v2.8   11-05-09 Fixed SPI and I/O startup conflict
		Fixed discrepancy between uC- and FILE-based strings
		uiChange to restrict LCD interactive options

v2.7   09-23-09 Gyro running average for gZero
                IMUCal parameter to allow for precision calibration
		Buzzer init addressing error fixed

v2.6   02-05-09 BumpStall stallCount fixed

v2.5   11-14-08 cf connection allowed with old stubs
                Starts dock on startup and close
		IMU accels read every 3ms
		Fixed wrapGyro180 rounding error

v2.4   07-04-08 Improved positional accuracy to ~+-1 degree head, mm move
		Updated IMUpac
		Resolved IMU/SAG error due to delay on open client
		Fixed no-stall behavior
		Fixed SAG/IMU restart odometry after power fail
		Client notified if any gyro/IMU failure in SIP cycle
		Removed deleterious wraparound in PID

v2.3   04-16-08	Always believe gyro when vel >+-2mm/s or rot >+-2d/s
		CONFIGpac reports gyroRateLimit in degx10/sec now
		Fixed bug in useGyro transitions effect on heading
		SPI-based gyro and IMU integration with related commands,
		  (26 and 27) and SIP (0x9A; 154)
		GyroRange command #24 and FLASH parameter
		FLAGS2 bit 4 useGyro indicator
		Gyro acquisition sync'd with StateEstimator
		Shutdown bit 3 of FLAGS2
		Low-battery client command #54
		Added GyroRateLimit command 25 and FLASH param
		Fixed Calibrator driftFactor sign

v2.2   05-21-07 Refinement to trajectory controller overruns
		Added freeSpace command, param and LCD display
		Fixed no-argument single packet request
		Updated LCD to new charset

v2.1   04-03-07 Added Calibration utility to LCD interactive mode
		Fixed trajectory controller windup and over-runs
		Fixed reconnection odometry error
		Updates to ARCOScf

v2.0   01-24-07 25ms re-issue of HEAD if HasGyro 2
		Fixed decel distance computation with gyro
		Got rid of rotation recoil with gyro
		Gyro correction in firmware
		Sync'd position integration with Std SIP timing
		Fixed rotVel for sipcycle vagaries
		Implemented (UN)SAFE joydrive option
		WatchDog timeout audible warning
		Added test in ARCOScf for version control

v1.9   09-27-06 PowerBot charging state bit inhibited by 68,0
		PCGood() init on start up for genpowerd monitor

v1.8   09-01-06 Removed stalls triggered by maximizing PWM
		Max error now maxPIDout/Kp..Kd; removes large windup
		Debugged command queue

v1.7   04-25-06 Fixed driftCompensator reverse-drive code
		View and change selected options with LCD's toggles
                Looping while mapping enabled
		DIGOUT state persists through connections (separate ioInits)
		  and accounts for gripper
		JOYpac sent even though joystick not enabled
	        LCD monitor restores it if signal or power temporarily
                  distrupted
		Debounced the bumpers
		rotVel sint at end of std SIP based on revCount
		ARCOScf now supports '-r <ropfile>' startup argument
		faultFlags int after rotVel in std SIP

v1.6   01-24-06 Fixed sonar max range reported value
		Updated LCD start up messages

v1.5   09-12-05 Stub and OS version numbers displayed by ARCOScf ?/help
		No joystick packet if joystick disabled
		Joystick req's trigger on/off cycle
		Grip state byte now reflected properly in std SIP
		Fixed drift problem in Trig position-integration calcs
		Fixed drift compensator
		Version Major and Minor strings at end of ConfigPac
		Version build number displayed on LCD when left toggle pressed
		ChargeThreshold (PowerBot only relevant) at end of ConfigPac
		Docking state byte at end of std SIP (bulk/overcharge/float)
		Debounced AT's E-Stop button
		ARCOScf updated so works even if ARIA doesn't find .p file

v1.4   05-10-05 Relieved bumpstall surge
		Sonar open disables unused arrays
		Sonar range max 5m (ARIA doesn't update if > 5m)
		Sonar_enable command only activates arrays that have poll
		Gripper no longer re-initialized upon client disconnection
		Version major/minor ascii numbers stored @ 0x8400,1

v1.3   03-05-05 Fixed the infamous "unhandled exception" error bug
		Added PowerBot docking code

v1.2   01-03-05 Added motors self-test
		More distinctive sounds for stall, etc.
		Support for legacy joystick
		Fixed Windows connection and addded autobauding in cf
		Disabled I2C interface (highly problematic)
		Added LCD to aux serial with FLASH setting

v1.1a  11-18-04 Corrected initialization for gripper.v1.1
	9-14-04 Added support for the Pioneer Arm

v1.0	9-1-04	Initial production release of ARCOS.
