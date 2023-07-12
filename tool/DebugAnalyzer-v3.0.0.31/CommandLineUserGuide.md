# DebugAnalyzer Command Line User Guide

Please assure the DebugAnalyzer you used is after vesion 2.1.7.

## Usage

    DebugAnalyser [-j0] [-fs] [-fu] [-df bool] [-rf bool] [-ln bool] [-sn bool] [-ts bool] [-it bool]

                  [-sf bool] [-ei bool] [-sl bool] [-rt bool] [-lt int] [-so int] [-sp portNumber]

                  [-br baudrate] [-pr parity] [-sb stopbits] [-fc flowcontrol] [-ar path] [-ri path] [-aa path]

                  [-ap path] [-pi path] [-ae path] [-da path] [-du path] [-dv path] [-dp path] [-fp path]

                  [-ce bool] [-ud bool] [-ucfg bool] [-config path] [-reopen int]


## Config file(.ini) parameters:

The composition and parameters of config file which is transfered to command line are alike to default.ini.
The parameter and value are divided by a comma.

## Priority:

cmd line parameter > custom .ini > .default.ini

## Parameters defination

- -j0: Don't Show UI
- -fs[.ini: DecodeFromSerial]: Decode from serial
- -fu[.ini: DecodeFromUSB]: Decode from usb
- -df[.ini: isSaveLogFile]: Save decoded log file, default true
- -rf[.ini: isSaveRawFile]: Save raw data file, default true
- -ln[.ini: isShowLineNumber]: Include line number, default true
- -sn[.ini: isShowSeqNumber]: Include sequence number, default true
- -ts[.ini: isParseTimeStamp]: Include time stamp, default true
- -it[.ini: isShowInternalTime]: Include internal time, default true
- -sf[.ini: isSaveSnoopFile]: Save snoop file, default true
- -ei[.ini: isUseEllisysInject]: Use Ellisys Inject, default true
- -sl[.ini: isShowLogs]: Show output logs (if show UI,default true; if hide UI, default false)
- -rt[.ini: isRealtimeDecode]: Real-time decode, default true
	 
     If false, just save raw data as .bin file
- -so[.ini: SplitOrientation]: Set UI split orientation (if show UI), default 2

	 1: Horizental

	 2: Vertical

- -lt[.ini: ShowLogType]: Set log type shown on UI (if show UI), default 1

	 1: ARM
     
	 2: DSP

	 3: ARM & DSP

- -sp[.ini: SerialPortName]: Set serial port number, default COM1
- -br[.ini: BaudRate]: Set baud rate, default 2000000
- -pr[.ini: Parity]: Set parity, default none
- -sb[.ini: StopBits]: Set stop bits, default one
- -fc[.ini: FlowControl]: Set flow control, default none
- -ar[.ini: ROMTraceFile]: Set ARM ROM trace file
- -ri[.ini: ROMIndexFile]: Set ARM ROM index file
- -aa[.ini: AppTraceFile]: Set ARM App trace file
- -ap[.ini: PatchTraceFile]: Set ARM Patch trace file
- -pi[.ini: PatchIndexFile]: Set ARM Patch index file
- -ae[.ini: PatchExtensionTraceFile]: Set ARM Patch_extension trace file
- -da[.ini: DAppTraceFile]: Set DSP App trace file
- -du[.ini: DAudioTraceFile]: Set DSP Audio trace file
- -dv[.ini: DVoiceTraceFile]: Set DSP Voice trace file
- -dp[.ini: DPatchTraceFile]: Set DSP Patch trace file
- -fp[.ini: OutputDir]: Indicate .cfa, .bin and .log files' common save path
- (All the above paths should be enclosed in double quotes)
- -ce[.ini: isParseHCICmdEvt]: Parse HCI command and event, default true
- -ud[.ini: isShowUndecodedData]: Show undecoded data (if show UI), default true
- -ucfg: Indicate whether to update the parameters derived from cmd line to default.ini
- -config: Use .ini file to set parameters, default the default.ini file
- -reopen: If serial port is removed exceptedly, reopen the port for n times with internal 10secs. 