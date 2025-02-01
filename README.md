This repository contains the µMD Windows Graphical User Interface (GUI) and associated firmware for Micro Measurement Displays µMD0, µMD1, and µMD2.

* Micro Measurement Display (µMD) is a Windows GUI for use with µMD0, µMD1, and µMD2 microcomputer boards and associated firmware which provides real time display of data from homodyne interferometers, encoders, and other
  devices providing Quadrature output; and heterodyne interferometers using two frequency HeNe lasers.  It supports most of the functions of a commercial display like the HP-5508A and more.  A PC with a USB port running Windows XP
  or later and .NET 4.0 or later is required for the custom µMD application.  Almost any Windows PC more recent than the Jurassic period should be satisfactory. :-)  So dust off that old laptop and put it to good use! Sorry,
  µMD does not run on a phone - yet!  (However, data may be streamed in real-time wirelessly to an Android device using additional hardware and firmware, NOT INCLUDED.)  The Graphical User Interface (µMD GUI) provides
  real-time displacement and velocity measurements using a variety of standard and custom interferometer optics, as well as optional frequency analysis of the data.  µMD also supports straightness and angle measurements with
  appropriate optics and allows for their parameters to be entered if not standard.  Environmental compensation parameters (temperature, pressure, humidity) may be entered manually.  Up to three measurement axes are supported.

  While the source code (written in Microsoft Visual Basic) for µMD is neither open source or in the public domain, it may be provided upon request for serious applications.  But we will not be expected to provide or help with
  enhancements or even minor bug fixes, in part because I (for one) have no clue how it works at this point as my brain was backed up quite a while ago and I've lost the passwords. ;( ;-)

  For general information on the µMD GUI, please see any of the links for µMD0, µMD1, or µMD2, below.  (The information on the µMD GUI in them is all similar.)  For specifics, select the one most apprpriate for your application.
  The communications protocol between the firmware and GUI is documented in all of these so a custom GUI could be used with the firmware or vice-versa.

The following are the required firmware which interfaces with the µMD GUI via USB on Windows PCs:

* Micro Measurement Display 0 (µMD0) firmware running on the Atmega328p Nano 3.0 (SG-µMD0 PCB) for low cost homodyne interferometers, encoders, and other devices providing Quad Sin/Cos or Quad A/B signals. µMD0 has a limited
   slew rate but is good for an introduction to metrology systems in education and industry.  For more information, please see: https://repairfaq.org/sam/uMD0/uMD0.htm .

* Micro Measurement Display 1 (µMD1) firmware running on the Microchip PIC32 (ChipKit DP32 or SG-µMD1 clone PCB) for heterodyne interferometers using two frequency HeNe lasers.  For more information, please see:
   https://repairfaq.org/sam/uMD1/uMD1.htm .

* Micro Measurement Display 2 (µMD2) firmware running on the Teensy 4.0 (SG-µMD2 PCB) for high performance homodyne interferometers, encoders, and other devices producing Quad A/B signals; and high performance heterodyne interferometers using two
   frequency HeNe lasers.  For more information, please see: https://repairfaq.org/sam/uMD2/uMD2.htm .

Copyright® Samuel M. Goldwasser and Jan Beck, 1994-2025, all rights reserved.
