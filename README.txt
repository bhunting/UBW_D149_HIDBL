UBW Firmware D version 1.4.9 Release 09/04/2011

* Added fourth paramter to F command, from 1 to 99, specifying PWM percentage (optional)
* Tested with MPLAB 8.76, C18 v3.40, USB stack v2.9a - all works

UBW Firmware D version 1.4.8 Release 03/28/2011
 
 * No functionality change, just modified main.c, linker scripts, and project file 
	to allow for easy conversion to compiling without the bootloader support. (i.e. 
	stand-alone). Simply comment out #define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER 
	in HardwareProfile_UBW.h and set your proper crystal speed in config bits in main.c 
	Also remove the /uPROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER from the 
	Project->Build Options->Project->MPLINK Linker->Use Alternate Settings command line.

UBW Firmware D version 1.4.8 Release 02/27/2011

* Fixed BS command so it can take	all binary data

UBW Firmware D version 1.4.7 Release 02/21/2011

* Same version as before, but projects have been updated to use latest version
	of the C18 compiler (v3.37.01). Because Microchip changed all of the paths
	around, the projects will no longer compile on any version of C18 other than
	this.
* Uncommented BC, BS, BO commands and fixed some code in them. They are untested 
	at this point but should work.

UBW Firmware D version 1.4.7 Release 02/20/2011

* Same version number as before, but we no longer put any instructions in the boot
	block (0x000 - 0x7FF). In previous versions of 1.4.7 there were reset and interrupt
	vectors located at 0x000. This would mess up a UBW who's bootloader was not properly
	protected and if the user's download tool allowed writing to the boot block. 
	(MCHPFSUSB does not - so Windows users are safe even without this change.) New
	versions of the C startup libraries without the GOTO _start at 0x0000 are now
	included in the FW D directory.

UBW Firmware D version 1.4.7 Release 02/16/2011

* Same version number as before, but now there are 3 projects, for 2455, 2554 and 4550 PICs.
	To compile in MPLAB, open the FW_D.mcw workspace, then right click on the project
		you want to compile and select "Set Active". Then, from the Configure menu, choose 
		Select Device and choose the device. (Silly MPLAB can't have different processors in the
		same workspace. MPLAB X fixes this.) Then do F10 to rebuild.

UBW Firmware D version 1.4.7 Release 02/10/2011

* By Brian Schmalz of Schmalz Haus LLC
* See top of user.c for change list
* Note that starting with this release, the Microchip USB stack must be located in the 
    Microchip folder wich must be at the same level on the drive as the D folder
	(which contains this file). You can download the free Microchip Application 
	Library (MAL) - which includes the latest USB stack and the Microchip folder -
	from here : http://www.microchip.com/MAL
	For more information on building this firmware and how the folders must be set up
	to make it all happy, see http://schmalzhaus.com/UBW32/doc/FirmwareBuildInstructions.html
	(Those instructions apply to the UBW code starting with v1.4.7, not just UBW32)
* See http://schmalzhaus.com/UBW/ for more information on the UBW
* This source code is released under the Creative Commons licence, see the UBW website
    for more information on the licence
* Please contact brian@schmalzhaus.com if you have any questions