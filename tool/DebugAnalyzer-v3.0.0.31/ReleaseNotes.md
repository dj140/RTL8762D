# Realtek Semiconductor Corp.

RTL8762A, RTL8762C, RTL8763B and RTL8761ATT Log Debug Analyser

# Release Notes

November 15, 2018

### Changes and Enhancement:
- Nothing

### Bug Fixes:
- Cannot decode log of BTIF_MSG_SCO_DATA_IND type

### Known Issues:
- TimeStamp of decoding is not correct
- FormatPtr is large than decoderFile.Length

----------------------------------------------------------------------------------------------


## Version 2.1.8.2

November 13, 2018

### Changes and Enhancement:
- Add decoding of BTLIB(ic type = 63)
- Add decoding of BBPro2(ic type = 65)
- Prefix .cfa file name with COM number

### Known Issues:
- TimeStamp of decoding is not correct
- FormatPtr is large than decoderFile.Length


----------------------------------------------------------------------------------------------


## Version 2.1.8.1

May 2, 2018

### Changes and Enhancement:
- Support custom .bin file name

### Bug Fixes:
- unchanged bluetooth address in a log

### Known Issues:
- TimeStamp of decoding is not correct
- FormatPtr is large than decoderFile.Length


----------------------------------------------------------------------------------------------


## Version 2.1.8.0

January 23, 2018

### Notices:
- Support for using with Ellisys
- Development environment: vs2015
- Support for using with command line

### Changes and Enhancement:
- Add manipulation with command line
- Add serial port selection on Main dialog
- Add "Cancel" button and Patch/ROM index path selection on Detail Settings dialog
- Adjust the layout of Detail Settings dialog
- Change the name from DebugAnalyser to DebugAnalyzer
- Select serial port automatically when plug/remove an IC

### Bug Fixes:
- Max Display Rows crashed when enterring a large number
- Filter dialog is always on top of view

### Known Issues:
- Some BTIF messages missing


----------------------------------------------------------------------------------------------


## Version 2.1.7.0

November 1, 2017

### Notices:
- Support for using with Ellisys
- Development environment: vs2015

### Changes and Enhancement:
- Automatic selection of IC
- Add index parse of patch log

### Bug Fixes:
- The mistaken checksum error
- Some exceptions
- Some bugs in UI

### Known Issues:
- NULL