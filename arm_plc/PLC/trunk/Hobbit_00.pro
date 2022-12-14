CoDeSys+1  ?                   @        @   2.3.9.41    @?    @                                     г8U ?   c:\documents and settings\administrator\desktop\hobbit\programm\libs\;k:\01_projects\514006(hobbit)\01_software\plc\libs\;s:\hella.automation\20127 - hobbit\software\plc_tu_svn\libs\ @      ?C??3?p?             K?Q        E?  @   t   C:\TWINCAT\PLC\LIB\TcUtilities.lib @                                                                                ?          ARG_TO_CSVFIELD           pSrc               ` ??           Pointer to the source buffer    pDest               ` ??       #    Pointer to the destination buffer    cbMax         ` ??           Max. number of input bytes    cbScan         ` ??           Input stream data byte number    cbReturn         ` ??           Number of result data bytes       in                 T_Arg   ??       T    Input data in PLC format (any data type, string, integer, floating point value...)    bQM            ??	       h    TRUE => Enclose result data in quotation marks, FALSE => Don't enclose result data in quotation marks.    pOutput           ??
       /    Address of output buffer (destination buffer)    cbOutput           ??       !    Max. byte size of output buffer       ARG_TO_CSVFIELD                                     ??P  ?    ????        
   BCD_TO_DEC        
   RisingEdge                 R_TRIG ` ??                 START            ??              BIN           ??                 BUSY            ??              ERR            ??              ERRID           ??	              DOUT           ??
       ?   
	Error codes:
		0x00		: No Errors
		0x0F	: Parameter value NOT correct. Wrong BCD input value in Low Nibble.
		0xF0	: Parameter value NOT correct. Wrong BCD input value in High Nibble.
            ??P  ?    ????           BE128_TO_HOST               in                T_UHUGE_INTEGER   ??                 BE128_TO_HOST                T_UHUGE_INTEGER                             ??P  ?    ????           BE16_TO_HOST               in           ??                 BE16_TO_HOST                                     ??P  ?    ????           BE32_TO_HOST           parr    	                            ` ??                 in           ??                 BE32_TO_HOST                                     ??P  ?    ????           BE64_TO_HOST               in                T_ULARGE_INTEGER   ??                 BE64_TO_HOST                T_ULARGE_INTEGER                             ??P  ?    ????           BYTEARR_TO_MAXSTRING               in   	  ?                       ??                 BYTEARR_TO_MAXSTRING               T_MaxString                             ??P  ?   ????           CSVFIELD_TO_ARG           pSrc               ` ??           Pointer to the source buffer    pDest               ` ??       $     Pointer to the destination buffer    cbMax         ` ??           Max. number of output bytes    cbScan         ` ??           Input stream data byte number    cbReturn         ` ??           Number of result data bytes    bQMPrior          ` ??       c    TRUE => Previous character was quotation mark. FALSE => Previous character was not quotation mark       pInput           ??       G    Address of input buffer with data in CSV field format (source buffer )   cbInput           ??	           Byte size of input data    bQM            ??
       \    TRUE => Remove enclosing quotation marks. FALSE => Don't remove enclosing quotation marks.    out                 T_Arg   ??       U    Output data in PLC format (any data type, string, integer, floating point value...)       CSVFIELD_TO_ARG                                     ??P  ?    ????           CSVFIELD_TO_STRING           cbField         ` ??                 in               T_MaxString   ??       "    Input string in CSV field format    bQM            ??	       \    TRUE => Remove enclosing quotation marks. FALSE => Don't remove enclosing quotation marks.       CSVFIELD_TO_STRING               T_MaxString                             ??P  ?    ????           DATA_TO_HEXSTR           iCase         ` ??              pCells    	  ?                          ` ??              idx         ` ??                 pData           ??           Pointer to data buffer    cbData             U              ??           Byte size of data buffer    bLoCase            ??       9    Default: use "ABCDEF", if TRUE use "abcdef" characters.       DATA_TO_HEXSTR               T_MaxString                             ??P  ?   ????        
   DCF77_TIME     "      DataBits   	  <                         ??              BitNo            ??              dtCurr            ??              dtNext            ??              tziCurr               E_TimeZoneID    ??       6    Time zone information extracted from latest telegram    tziPrev               E_TimeZoneID    ??       8    Time zone information extracted from previous telegram    tDiff            ??           Two telegram time difference    bCheck             ??              Step         ` ??!           	   StartEdge                 R_TRIG ` ??"              RisingPulse                 R_TRIG ` ??$              FallingPulse                 F_TRIG ` ??%           	   LongPulse                    TON ` ??&           
   ShortPulse                    TON ` ??'           
   DetectSync                    TOF ` ??(              NoDCFSignal                    TON ` ??)              DCFCycleLen                    TON ` ??*           	   bIsRising          ` ??,           
   bIsFalling          ` ??-              bIsLong          ` ??.              bIsShort          ` ??/              Working          ` ??0           	   DataValid          ` ??2           
   ParitySum1         ` ??3           
   ParitySum2         ` ??4           
   ParitySum3         ` ??5              i         ` ??7           	   DummyByte         ` ??8              DummyString    Q       Q  ` ??9              Hour         ` ??;              Minute         ` ??<              Year         ` ??=              Month         ` ??>              Day         ` ???              	   DCF_PULSE            ??           DCF77 pulse: 101010101...    RUN            ??       *    Enable/disable function block execution.       BUSY            ??           TRUE = Decoding in progress    ERR            ??	       ,    Error flag: TRUE = Error, FALSE = No error    ERRID           ??
           Error code    ERRCNT           ??           Error counter    READY            ??       1    TRUE => CDT is valid, FALSE => CDT is not valid    CDT           ??           date and time information             ??P  ?    ????           DCF77_TIME_EX     #      DataBits   	  <                         ??           Decoded data bits    BitNo            ??           Decoded bit number    dtCurr            ??       %    Time extracted from latest telegram    dtNext            ??            Supposed next time    tziCurr               E_TimeZoneID    ??!       6    Time zone information extracted from latest telegram    tziPrev               E_TimeZoneID    ??"       8    Time zone information extracted from previous telegram    tDiff            ??#       )    Time difference of two latest telegrams    bCheck             ??$       H    TRUE = Plausibility check over two telegrams enabled, FALSE = disabled    Step         ` ??&           	   StartEdge                 R_TRIG ` ??'              RisingPulse                 R_TRIG ` ??)              FallingPulse                 F_TRIG ` ??*           	   LongPulse                    TON ` ??+           
   ShortPulse                    TON ` ??,           
   DetectSync                    TOF ` ??-              NoDCFSignal                    TON ` ??.              DCFCycleLen                    TON ` ??/           	   bIsRising          ` ??1           
   bIsFalling          ` ??2              bIsLong          ` ??3              bIsShort          ` ??4              Working          ` ??5           	   DataValid          ` ??7           
   ParitySum1         ` ??8           
   ParitySum2         ` ??9           
   ParitySum3         ` ??:              i         ` ??<           	   DummyByte         ` ??=              DummyString    Q       Q  ` ??>              Hour         ` ??@              Minute         ` ??A              Year         ` ??B              Month         ` ??C              Day         ` ??D           	   DayOfWeek         ` ??E              	   DCF_PULSE            ??           DCF77 pulse: 101010101...    RUN            ??       *    Enable/disable function block execution.    TLP    ?      ??           Short/long pulse split point       BUSY            ??	           TRUE = Decoding in progress    ERR            ??
       ,    Error flag: TRUE = Error, FALSE = No error    ERRID           ??           Error code    ERRCNT           ??           Error counter    READY            ??       1    TRUE => CDT is valid, FALSE => CDT is not valid    CDT           ??           date and time information    DOW                         ??       0     ISO 8601 day of week: 1 = Monday.. 7 = Sunday    TZI               E_TimeZoneID   ??           time zone information    ADVTZI            ??       1    MEZ->MESZ or MESZ->MEZ time change notification    LEAPSEC            ??           TRUE = Leap second    RAWDT   	  <                        ??           Raw decoded data bits             ??P  ?    ????        
   DEC_TO_BCD        
   RisingEdge                 R_TRIG ` ??                 START            ??              DIN           ??                 BUSY            ??              ERR            ??              ERRID           ??	              BOUT           ??
       h   
	Error codes:
		0x00		: No errors
		0xFF	: Parameter value NOT correct. Wrong DECIMAL input value.
            ??P  ?    ????        
   DEG_TO_RAD               ANGLE                        ??              
   DEG_TO_RAD                                                  ??P  ?    ????           DINT_TO_DECSTR               in           ??           
   iPrecision           ??	                 DINT_TO_DECSTR               T_MaxString                             ??P  ?    ????           DT_TO_FILETIME           ui64                T_ULARGE_INTEGER ` ??                 DTIN           ??                 DT_TO_FILETIME             
   T_FILETIME                             ??P  ?    ????           DT_TO_SYSTEMTIME           sDT             ` ??              nDay         ` ??              b   	                 
    24(16#30)      0    ` ??              TS                   
   TIMESTRUCT ` ??           	   Index7001                            DTIN           ??                 DT_TO_SYSTEMTIME                   
   TIMESTRUCT                             ??P  ?    ????           DWORD_TO_BINSTR           bits   	                        ` ??       6    array of ASCII characters (inclusive null delimiter)    iSig         ` ??           number of significant bits    iPad         ` ??           number of padding zeros    i         ` ??           	   Index7001                            in           ??           
   iPrecision           ??                 DWORD_TO_BINSTR               T_MaxString                             ??P  ?    ????           DWORD_TO_DECSTR           dec   	                       ` ??       6    array of ASCII characters (inclusive null delimiter)    iSig         ` ??           number of significant nibbles    iPad         ` ??           number of padding zeros    i         ` ??              divider     ʚ; ` ??              number         ` ??           	   Index7001                            in           ??           
   iPrecision           ??                 DWORD_TO_DECSTR               T_MaxString                             ??P  ?    ????           DWORD_TO_HEXSTR           hex   	                       ` ??       6    array of ASCII characters (inclusive null delimiter)    iSig         ` ??           number of significant nibbles    iPad         ` ??           number of padding zeros    i         ` ??           	   Index7001                            in           ??           
   iPrecision           ??              bLoCase            ??	       8   Default: use "ABCDEF", if TRUE use "abcdef" characters.       DWORD_TO_HEXSTR               T_MaxString                             ??P  ?    ????           DWORD_TO_LREALEX               in           ??                 DWORD_TO_LREALEX                                                  ??P  ?    ????           DWORD_TO_OCTSTR           oct   	                       ` ??       6    array of ASCII characters (inclusive null delimiter)    iSig         ` ??           number of significant nibbles    iPad         ` ??           number of padding zeros    i         ` ??           	   Index7001                            in           ??           
   iPrecision           ??                 DWORD_TO_OCTSTR               T_MaxString                             ??P  ?    ????           F_ARGCMP               typeSafe            ??              arg1                 T_Arg   ??              arg2                 T_Arg   ??                 F_ARGCMP                                     ??P  ?    ????           F_ARGCPY               typeSafe            ??                 F_ARGCPY                               dest                  T_Arg  ??
              src                  T_Arg  ??                   ??P  ?    ????           F_ARGISZERO               arg                 T_Arg   ??                 F_ARGIsZero                                      ??P  ?    ????        	   F_BIGTYPE               pData           ??            Address pointer of data buffer    cbLen           ??           Byte length of data buffer    	   F_BIGTYPE                 T_Arg                             ??P  ?    ????           F_BOOL                   F_BOOL                 T_Arg                       in            ??                   ??P  ?    ????           F_BYTE                   F_BYTE                 T_Arg                       in           ??                   ??P  ?    ????           F_BYTE_TO_CRC16_CCITT               value           ??           Data value    crc           ??       >    Initial value (16#FFFF or 16#0000) or previous CRC-16 result       F_BYTE_TO_CRC16_CCITT                                     ??P  ?    ????           F_CHECKSUM16        	   wChkSum_I         ` ??	       %    internal ChkSum                        dataWord         ` ??
       %    current data byte                      iIdx         ` ??       %    current data buffer index              ptrData               ` ??       %    pointer to current data byte           	   dwSrcAddr           ??       %    address of data buffer                 cbLen           ??       %    length of data buffer                  wChkSum           ??       %    init value (16#0000) or last ChkSum       F_CheckSum16                                     ??P  ?    ????           F_CRC16_CCITT           wCRC_I         ` ??
       $    internal CRC                          dataWord         ` ??       $    current data byte                     iIdx         ` ??       $    current data buffer index             ptrData               ` ??       $    pointer to current data byte          	   dwSrcAddr           ??       $    address of data buffer                cbLen           ??       $    length of data buffer                 wLastCRC           ??       $    init value (16#FFFF) or last CRC16       F_CRC16_CCITT                                     ??P  ?    ????           F_CREATEHASHTABLEHND           p                     T_HashTableEntry      ` ??              i         ` ??                 pEntries                     T_HashTableEntry        ??       C    Pointer to the first entry of hash table database (element array) 	   cbEntries           ??       ;    Byte size (length) of hash table database (element array)       F_CreateHashTableHnd                                hTable         	               T_HHASHTABLE  ??           Hash table handle         ??P  ?    ????           F_CREATELINKEDLISTHND           p                   T_LinkedListEntry      ` ??           Temp. previous node    n                   T_LinkedListEntry      ` ??           Temp. next node    i         ` ??           loop iterator       pEntries                   T_LinkedListEntry        ??       @    Pointer to the first linked list node database (element array) 	   cbEntries           ??       <    Byte size (length) of linked list database (element array)       F_CreateLinkedListHnd                                hList         	               T_HLINKEDLIST  ??           Linked list handle         ??P  ?    ????           F_DATA_TO_CRC16_CCITT           i         ` ??                 pData           ??           Pointer to data    cbData           ??           Length of data    crc           ??       >    Initial value (16#FFFF or 16#0000) or previous CRC-16 result       F_DATA_TO_CRC16_CCITT                                     ??P  ?    ????           F_DINT                   F_DINT                 T_Arg                       in           ??                   ??P  ?    ????           F_DWORD                   F_DWORD                 T_Arg                       in           ??                   ??P  ?    ????           F_FORMATARGTOSTR     	      pOut               ` ??              longword         ` ??              double                      ` ??              single          ` ??              short         ` ??              small         ` ??              longint         ` ??              iPaddingLen         ` ??              iCurrLen         ` ??           
      bSign            ??           Sign prefix flag    bBlank            ??           Blank prefix flag    bNull            ??           Null prefix flag    bHash            ??           Hash prefix flag    bLAlign            ??       4    FALSE => Right align (default), TRUE => Left align    bWidth            ??       C    FALSE => no width padding, TRUE => blank or zeros padding enabled    iWidth           ??	           Width length parameter 
   iPrecision           ??
           Precision length parameter    eFmtType               E_TypeFieldParam   ??           Format type field parameter    arg                 T_Arg   ??           Format argument       F_FormatArgToStr                               sOut                 T_MaxString  ??                   ??P  ?    ????           F_GETDAYOFMONTHEX           dom         ` ??           Day of month    dow         ` ??           Day of week    n         ` ??              dwYears         ` ??              dwDays         ` ??                 wYear     A  A  kx             ??           Year: 1601..30827    wMonth                         ??           Month: 1..12    wWOM                         ??       ?     Week of month. Occurrence of the day of the week within the month (1..5, where 5 indicates the final occurrence during the month if that day of the week does not occur 5 times).   wDOW                           ??       4    Day of week (0 = Sunday, 1 = Monday.. 6 = Saturday       F_GetDayOfMonthEx                                     ??P  ?    ????           F_GETDAYOFWEEK           sysTime                   
   TIMESTRUCT ` ??	                 in           ??                 F_GetDayOfWeek                                     ??P  ?    ????           F_GETDOYOFYEARMONTHDAY           bLY          ` ??
                 wYear           ??           Year: 0..2xxx    wMonth           ??           Month 1..12    wDay           ??           Day: 1..31       F_GetDOYOfYearMonthDay                                     ??P  ?    ????           F_GETFLOATREC     
   	   ptrDouble    	                               ??              fValue                         ??
              fBegin                         ??              nBegin            ??              fDiv                         ??              nDig            ??              nDigit            ??              fMaxPrecision                         ??              i            ??              nLastDecDigit            ??                 fVal                        ??           
   iPrecision           ??              bRound            ??                 F_GetFloatRec              
   T_FloatRec                             ??P  ?    ????           F_GETMAXMONTHDAYS               wYear           ??	              wMonth           ??
                 F_GetMaxMonthDays                                     ??P  ?    ????           F_GETMONTHOFDOY           bLY          ` ??	              wMonth         ` ??
                 wYear           ??           Year: 0..2xxx    wDOY           ??           Year's day number: 1..366       F_GetMonthOfDOY                                     ??P  ?    ????           F_GETVERSIONTCUTILITIES               nVersionElement           ??       d   
	Possible nVersionElement parameter:
	1	:	major number
	2	:	minor number
	3	:	revision number
      F_GetVersionTcUtilities                                     ??P  ?    ????           F_GETWEEKOFTHEYEAR           date_sec         ` ??           date seconds    dow         ` ??	           day of week    year         ` ??
              KWStart         ` ??              first    ??yG ` ??              ff                      ` ??                 in           ??                 F_GetWeekOfTheYear                                     ??P  ?    ????           F_HUGE                   F_HUGE                 T_Arg                       in                 T_HUGE_INTEGER  ??                   ??P  ?    ????           F_INT                   F_INT                 T_Arg                       in           ??                   ??P  ?    ????           F_LARGE                   F_LARGE                 T_Arg                       in                 T_LARGE_INTEGER  ??                   ??P  ?    ????           F_LREAL                   F_LREAL                 T_Arg                       in                        ??                   ??P  ?    ????           F_LTRIM           pChar               ` ??              pStr                 T_MaxString      ` ??	                 in               T_MaxString   ??                 F_LTrim               T_MaxString                             ??P  ?    ????           F_REAL                   F_REAL                 T_Arg                       in            ??                   ??P  ?    ????           F_RTRIM           n         ` ??              pChar               ` ??	                 in               T_MaxString   ??                 F_RTrim               T_MaxString                             ??P  ?    ????           F_SINT                   F_SINT                 T_Arg                       in           ??                   ??P  ?    ????           F_STRING                   F_STRING                 T_Arg                       in                 T_MaxString  ??                   ??P  ?    ????        
   F_SWAPREAL           pReal    	                               ??              pResult    	                               ??                 fVal            ??              
   F_SwapReal                                      ??P  ?    ????           F_SWAPREALEX           pIN    	                            ` ??              wSave         ` ??	                     F_SwapRealEx                                fVal            ??                   ??P  ?    ????        	   F_TOLCASE           pDest               ` ??              idx                 MIN_SBCS_TABLE   MAX_SBCS_TABLE ` ??	                 in               T_MaxString   ??              	   F_ToLCase               T_MaxString                             ??P  ?   ????        	   F_TOUCASE           pDest               ` ??              idx                 MIN_SBCS_TABLE   MAX_SBCS_TABLE ` ??	                 in               T_MaxString   ??              	   F_ToUCase               T_MaxString                             ??P  ?   ????           F_TRANSLATEFILETIMEBIAS           ui64In                T_ULARGE_INTEGER ` ??       E    Input file time as 64 bit unsigned integer (number of 100ns ticks)     ui64Bias                T_ULARGE_INTEGER ` ??       ?    Bias value as 64 bit unsigned integer (number of 100ns ticks)    ui64Out                T_ULARGE_INTEGER ` ??       @    Local time as 64 bit unsigned integer (number of 100ns ticks)        in             
   T_FILETIME   ??       1    Input time in file time format to be translated    bias           ??       y    Bias value in minutes. The bias is the difference, in minutes, between Coordinated Universal Time (UTC) and local time.    toUTC            ??       U    TRUE => Translate from local time to UTC, FALSE => Translate from UTC to local Time       F_TranslateFileTimeBias             
   T_FILETIME                             ??P  ?    ????           F_UDINT                   F_UDINT                 T_Arg                       in           ??                   ??P  ?    ????           F_UHUGE                   F_UHUGE                 T_Arg                       in                 T_UHUGE_INTEGER  ??                   ??P  ?    ????           F_UINT                   F_UINT                 T_Arg                       in           ??                   ??P  ?    ????           F_ULARGE                   F_ULARGE                 T_Arg                       in                 T_ULARGE_INTEGER  ??                   ??P  ?    ????           F_USINT                   F_USINT                 T_Arg                       in           ??                   ??P  ?    ????           F_WORD                   F_WORD                 T_Arg                       in           ??                   ??P  ?    ????           F_YEARISLEAPYEAR               wYear           ??                 F_YearIsLeapYear                                      ??P  ?    ????           FB_ADDROUTEENTRY        
   fbAdsWrite       P    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_ADDREMOTE, IDXOFFS := 0 )              	   T_AmsPort         !                 ADSWRITE ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??           	   dataEntry                ST_AmsRouteSystemEntry ` ??                 sNetID            
   T_AmsNetID   ??       &    TwinCAT network address (ams net id)    stRoute                    ST_AmsRouteEntry   ??       !    Structure with route parameters    bExecute            ??       -    Rising edge starts function block execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??
              bError            ??              nErrID           ??                       ??P  ?   ????           FB_AMSLOGGER        
   fbAdsWrite       [    ( PORT:= AMSPORT_AMSLOGGER, IDXGRP:= AMSLOGGER_IGR_GENERAL, IDXOFFS:= AMSLOGGER_IOF_MODE )              	   T_AmsPort                          ADSWRITE ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              stReq                ST_AmsLoggerReq ` ??                 sNetId           ''    
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    eMode           AMSLOGGER_RUN       E_AmsLoggerMode   ??              sCfgFilePath           ''       T_MaxString   ??              bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??
              bError            ??              nErrId           ??                       ??P  ?   ????           FB_BASICPID           nERR_NOERROR           ??           no error						   nERR_INVALIDPARAM          ??           invalid parameter				   nERR_INVALIDCYCLETIME          ??           invalid cycle time				   fE               0.0            ??            error input					   fE_1               0.0            ??!           error input z^(-1)				   fY               0.0            ??"           control output				   fY_1               0.0            ??#           control output  z^(-1)			   fYP               0.0            ??$           P-part						   fYI               0.0            ??%           I-part						   fYI_1               0.0            ??&           I-part  z^(-1)					   fYD               0.0            ??'           D-T1-part					   fYD_1               0.0            ??(           D-T1-part  z^(-1)				   bInit            ??*       %    initialization flag for first cycle	   bIsIPart             ??+           I-part active ?				   bIsDPart             ??,           D-part active ?				   fDi               0.0            ??.           internal I param				   fDd               0.0            ??/           internal D param				   fCd               0.0            ??0           internal D param				   fCtrlCycleTimeOld               0.0            ??2              fKpOld               0.0            ??3              fTnOld               0.0            ??4              fTvOld               0.0            ??5              fTdOld               0.0            ??6                 fSetpointValue                        ??           setpoint value							   fActualValue                        ??           actual value							   bReset            ??           controller values    fCtrlCycleTime                        ??	       (    controller cycle time in seconds [s]			   fKp                        ??           proportional gain Kp	(P)					   fTn                        ??           integral gain Tn (I) [s]						   fTv                        ??       "    derivative gain Tv (D-T1) [s]				   fTd                        ??       (    derivative damping time Td (D-T1) [s]		      fCtrlOutput                        ??           controller output command				   nErrorStatus           ??       1    controller error output (0: no error; >0:error)	            ??P  ?    ????           FB_BUFFEREDTEXTFILEWRITER           fbFile                               FB_TextFileRingBuffer ` ??           
   closeTimer                    TON ` ??           auto close timer    bRemove          ` ??              nStep         ` ??                 sNetId           ''    
   T_AmsNetId ` ??           ams net id 	   sPathName           'c:\Temp\data.dat'       T_MaxString ` ??	       6    file buffer path name (max. length = 255 characters)    ePath           PATH_GENERIC    
   E_OpenPath ` ??
           default: Open generic file    bAppend         ` ??       )    TRUE = append lines, FALSE = not append 
   tAutoClose    ?   ` ??              tTimeout    ?   ` ??                 bBusy          ` ??              bError          ` ??              nErrID         ` ??                 fbBuffer                 FB_StringRingBuffer` ??           string ring buffer         ??P  ?   ????           FB_CONNECTSCOPESERVER           stRecordDesc       d    (nRunMode:=0, nSopMode:=0, bStoreOnDisk:=FALSE, bUseLocalServer:=FALSE, bStartServerFromFile:=TRUE)                                #   ST_ScopeServerRecordModeDescription    ??              nState            ??              nErrorState            ??           
   fbAdsWrite                          ADSWRITE    ??              fbQueryRegistry                                 FB_RegQueryValue    ??              sScopeServerDir                ??              sScopeServerPath                ??              fbStartServer                              NT_StartProcess    ??              fbWait                    TON    ??               bTriggerServerStart            ??!              nDwellTimeCounter            ??"              nPort           27110    	   T_AmsPort   ??%              Connect_IdxGrp     u     ??&          0x7500      sNetId           ''    
   T_AmsNetId   ??              bExecute            ??              sConfigFile    Q       Q    ??              tTimeout    ?     ??                 bBusy            ??              bDone            ??              bError            ??              nErrorId           ??                       ??P  ?   ????           FB_CSVMEMBUFFERREADER           state         ` ??              getBufferIndex         ` ??              scanPtr               ` ??              scanSize         ` ??              bField          ` ??              cbCopied         ` ??           
   bFirstChar          ` ??              bDQField          ` ??           	   bDQBefore          ` ??              pField         ` ??       U    If successfull then this variable returns the address of the first/next field value    cbField         ` ??       W    If successfull then this variable returns the byte size of the first/next field value    bEOF          ` ??           TRUE => End of field found    bBreak          ` ??                 eCmd           eEnumCmd_First       E_EnumCmdType   ??       )    Command type: read first or next field ?   pBuffer           ??       #    Address ( pointer) of data buffer    cbBuffer           ??           Max. byte size of data buffer       bOk            ??	       &    TRUE => Successfull, FALSE => Failed    getValue           ''       T_MaxString   ??
       N    If successfull then this output returns the first/next field value as string    pValue           ??       s    Pointer to the first value byte (HINT: String values are not null terminated. Empty string returns Null pointer )    cbValue           ??           Field value byte size    bCRLF            ??       .    TRUE => End of record separator found (CRLF)    cbRead           ??       )    Number of successfully parse data bytes             ??P  ?    ????           FB_CSVMEMBUFFERWRITER           fbReader                                    FB_CSVMemBufferReader ` ??              temp   	  ,                    ` ??           Temp buffer    cbTemp         ` ??       %    Number of data bytes in temp buffer    cbCopied         ` ??       9    Number of data bytes copied to the external data buffer    bNewLine         ` ??           TRUE => start with new line       eCmd           eEnumCmd_First       E_EnumCmdType   ??       *    Command type: write first or next field ?   putValue           ''       T_MaxString   ??       &    New first/next field value as string    pValue           ??       C    OPTIONAL: Pointer to external buffer containing field value data.    cbValue           ??       F    OPTIONAL: Byte size of external buffer containing field value data.     bCRLF            ??       0    TRUE = > Append end of record separator (CRLF)    pBuffer           ??	       #    Address ( pointer) of data buffer    cbBuffer           ??
           Max. byte size of data buffer       bOk            ??       &    TRUE => Successfull, FALSE => Failed    cbSize           ??           Number fo used data bytes    cbFree           ??           Number of free data bytes    nFields           ??           Number of fields    nRecords           ??           Number of records    cbWrite           ??       +    Number of successfully written data bytes             ??P  ?    ????           FB_DBGOUTPUTCTRL           fbFormat                                     FB_FormatString ` ??              fbBuffer        	               FB_StringRingBuffer ` ??              fbFile       +    (ePath := PATH_BOOTPATH, bAppend := TRUE )                 PATH_GENERIC    
   E_OpenPath                      FB_BufferedTextFileWriter ` ??              buffer   	  '                   ` ??              state         ` ??              nItems         ` ??              k         ` ??               bInit         ` ??!           Hex logging    i         ` ??$              cells   	                               ` ??%              pCells                 T_MaxString      ` ??&              cbL1         ` ??'              cbL2         ` ??'              idx         ` ??'              pSrc1               ` ??(              pSrc2               ` ??(                 dwCtrl         ` ??       &    Debug message target: DBG_OUTPUT_LOG    sFormat           ''       T_MaxString ` ??           Debug message format string    arg1                 T_Arg ` ??           Format string argument    arg2                 T_Arg ` ??              arg3                 T_Arg ` ??	              arg4                 T_Arg ` ??
              arg5                 T_Arg ` ??              arg6                 T_Arg ` ??              arg7                 T_Arg ` ??              arg8                 T_Arg ` ??              arg9                 T_Arg ` ??              arg10                 T_Arg ` ??              sFilter           ''       T_MaxString ` ??                 bError          ` ??              nError         ` ??           	   nOverflow         ` ??                       ??P  ?   ????           FB_DISCONNECTSCOPESERVER        
   fbAdsWrite                          ADSWRITE    ??              nState            ??                 sNetId            
   T_AmsNetId   ??              bExecute            ??              tTimeout    ?     ??                 bBusy            ??              bDone            ??	              bError            ??
              nErrorId           ??                       ??P  ?   ????           FB_ENUMFINDFILEENTRY        
   fbAdsRdWrt       B    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_FFILEFIND )              	   T_AmsPort         ?                ADSRDWRT ` ??           
   fbAdsWrite       D    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_CLOSEHANDLE )              	   T_AmsPort         o              ADSWRITE ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??           	   dataEntry                            ST_AmsFindFileSystemEntry ` ??              eFindCmd               E_EnumCmdType ` ??                 sNetId            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id) 	   sPathName               T_MaxString   ??       %    dir/path/file name, wildcards [*|?]    eCmd           eEnumCmd_First       E_EnumCmdType   ??           Enumerator navigation command    bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??              bError            ??              nErrID           ??              bEOE            ??           End of enumeration 
   stFindFile                     ST_FindFileEntry   ??           Find file entry             ??P  ?   ????           FB_ENUMFINDFILELIST           fbEnum                              FB_EnumFindFileEntry ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              cbEntry         ` ??              nEntries         ` ??              pEntry                      ST_FindFileEntry      ` ??                 sNetId            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id) 	   sPathName               T_MaxString   ??       %    dir/path/file name, wildcards [*|?]    eCmd           eEnumCmd_First       E_EnumCmdType   ??           Enumerator navigation command 	   pFindList           ??       &    POINTER TO ARRAY OF ST_FindFileEntry 
   cbFindList           ??       (    Byte size of ARRAY OF ST_FindFileEntry    bExecute            ??	       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??
           Max fb execution time       bBusy            ??              bError            ??              nErrID           ??              bEOE            ??           End of enumeration 
   nFindFiles           ??           Number of find files             ??P  ?   ????           FB_ENUMROUTEENTRY        	   fbAdsRead       Z    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_ENUMREMOTE (*, IDXGRP := index *) )              	   T_AmsPort         #             ADSREAD ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              index    ???? ` ??           	   dataEntry                ST_AmsRouteSystemEntry ` ??                 sNetID            
   T_AmsNetID   ??       '    TwinCAT network address (ams net id )    eCmd           eEnumCmd_First       E_EnumCmdType   ??           Enumerator navigation command    bExecute            ??       -    Rising edge starts function block execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??
              bError            ??              nErrID           ??              bEOE            ??       l    End of enumeration. This value is TRUE after the first read that attempts to read next non existing entry.    stRoute                    ST_AmsRouteEntry   ??       !    Structure with route parameters             ??P  ?   ????           FB_ENUMSTRINGNUMBERS           pSrc               ` ??              pDest               ` ??              pNext               ` ??              char         ` ??              state         ` ??              bEat          ` ??                 sSearch               T_MaxString   ??           Source string    eCmd           eEnumCmd_First       E_EnumCmdType   ??           Enumerator navigation command    eType           eNumGroup_Float       E_NumGroupTypes   ??           String number format type       sNumber               T_MaxString   ??           Found string number    nPos           ??	       )    <> 0 => Next scan/search start position    bEOS            ??
           TRUE = End of string             ??P  ?    ????           FB_FILERINGBUFFER           fbOpen                             FB_FileOpen ` ??              fbClose                      FB_FileClose ` ??              fbWrite                           FB_FileWrite ` ??              fbRead                            FB_FileRead ` ??               fbSeek                         FB_FileSeek ` ??!              nStep         ` ??"       X    0=idle, 1=init, 10,11=open, 20,21=seek, 30,31=read, 40,41=write, 50,51=close, 100=exit    bInit          ` ??#       5    TRUE=reading length chunk, FALSE=reading data chunk    bExit          ` ??$       O    FALSE=repeat reading/writing, TRUE=abort reading/writing, go to the exit step    bReopen          ` ??%       t    Open mode: TRUE=try to open existing file, FALSE=create new file, if open fails => try to create and open new file    bOpen          ` ??&       %    TRUE=file opened, FALSE=file closed    bGet          ` ??'       $    TRUE=get entry, FALSE=remove entry    bOW          ` ??(       b    TRUE=removing oldest entry (bOverwrite=TRUE), FALSE=don't remove oldest entry (bOverwrite=FALSE)    cbOW         ` ??)       /    Temp length of ovwerwritten length/data chunk    cbMoved         ` ??*       =    Number of successfully read/written length/data chunk bytes    ptrSaved         ` ??+       M    Seek pointer previous position (used by A_GetHead or read buffer underflow)    ptrMax         ` ??,       D    Seek pointer max. position = SIZEOF(ring buffer header) + cbBuffer    eCmd           eFileRBuffer_None       E_FileRBufferCmd ` ??-              eOldCmd           eFileRBuffer_None       E_FileRBufferCmd ` ??.                 sNetId           ''    
   T_AmsNetId   ??           ams net id 	   sPathName           'c:\Temp\data.dat'       T_MaxString   ??       6    file buffer path name (max. length = 255 characters)    ePath           PATH_GENERIC    
   E_OpenPath   ??           default: Open generic file    nID           ??           user defined version ID    cbBuffer          ??           max. file buffer byte size 
   bOverwrite            ??	       :    FALSE = don't overwrite, TRUE = overwrite oldest entries 
   pWriteBuff           ??
       "    pointer to external write buffer 
   cbWriteLen           ??       $    byte size of external write buffer 	   pReadBuff           ??       !    pointer to external read buffer 	   cbReadLen           ??       #    byte size of external read buffer    tTimeout    ?     ??                 bBusy            ??              bError            ??              nErrID           ??       ?    ADS or function specific error codes:
	16#8000	= (File) buffer empty or overflow 
	16#8001 = (Application) buffer underflow (cbReadLen to small),  	
	16#8002	= Buffer is not opened  
	16#8003	= Invalid input parameter value    cbReturn           ??       !    number of recend read data bytes   stHeader                          ST_FileRBufferHead   ??           buffer status             ??P  ?   ????            FB_FILETIMETOTZSPECIFICLOCALTIME           fbBase       !    ( wStdYear := 0, wDldYear := 0 )                                   "   FB_TranslateUtcToLocalTimeByZoneID ` ??           Underlaid base function block       in             
   T_FILETIME   ??       ?    Time to be converted (UTC, file time format), 64-bit value representing the number of 100-nanosecond intervals since January 1, 1601   tzInfo                     ST_TimeZoneInformation   ??           Time zone settings       out             
   T_FILETIME   ??       *    Converted time in local file time format    eTzID           eTimeZoneID_Unknown       E_TimeZoneID   ??	       "    Daylight saving time information    bB            ??
            FALSE => A time, TRUE => B time            ??P  ?    ????           FB_FORMATSTRING     	      pFormat               ` ??           pointer to the format string    pOut               ` ??           pointer to the result string 
   iRemOutLen         ` ??       $    Max remaining length of sOut buffer   bValid          ` ??       8    if set, the string character is valid format parameter    stFmt                              ST_FormatParameters ` ??           
   nArrayElem         ` ??           	   nArgument         ` ??              parArgs   	  
                     T_Arg              ` ??              sArgStr               T_MaxString ` ??                 sFormat               T_MaxString   ??              arg1                 T_Arg   ??              arg2                 T_Arg   ??              arg3                 T_Arg   ??              arg4                 T_Arg   ??              arg5                 T_Arg   ??              arg6                 T_Arg   ??	              arg7                 T_Arg   ??
              arg8                 T_Arg   ??              arg9                 T_Arg   ??              arg10                 T_Arg   ??                 bError            ??              nErrId           ??              sOut               T_MaxString   ??                       ??P  ?   ????           FB_GETADAPTERSINFO     
   	   fbAdsRead       f    ( PORT:=AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_IPHELPERAPI, IDXOFFS:= IPHELPERAPI_ADAPTERSINFO )              	   T_AmsPort         ?                ADSREAD ` ??           
   fbRegQuery       P    ( sSubKey:= '\Software\Beckhoff\TwinCAT\Remote', sValName := 'DefaultAdapter' )                        T_MaxString                    T_MaxString                   FB_RegQueryValue ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              cbInfo         ` ??              idx         ` ??              info   	                                      ST_IP_ADAPTER_INFO         ` ??           buffer for 12 entries    pInfo                                 ST_IP_ADAPTER_INFO      ` ??           
   nRealCount         ` ??           	   sDefaultA               T_MaxString ` ??                 sNetID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??	              bError            ??
              nErrID           ??              arrAdapters   	                                    ST_IpAdapterInfo           ??              nCount           ??           Max. number of found adapters    nGet           ??       %    Number of read adapter info entries             ??P  ?   ????           FB_GETDEVICEIDENTIFICATION        	   iDataSize       @` ??           
   byTagStart    <    ` ??           '<'    byTagEnd    >    ` ??           '>' 
   byTagSlash    /    ` ??           '/' 	   fbAdsRead                          ADSREAD ` ??              bExecutePrev          ` ??              iState         ` ??              iData   	                      ` ??           
   strActPath             ` ??              iLoopEndIdx         ` ??              iStructSize         ` ??              strHardwareCPU             ` ??              strTags   	  	        )       )          ` ??           	   iTagsSize   	  	                     ` ??              iCurrTag   	  (                     ` ??               iCurrTagData   	  P                     ` ??!           	   iPathSize         ` ??"              iTagIdx         ` ??$              iCurrTagIdx         ` ??%              iDataIdx         ` ??&              iCurrTagDataIdx         ` ??'              k         ` ??(              iMinCurrData         ` ??)           	   iFirstIdx         ` ??*              iLastIdx         ` ??+           	   bTagStart          ` ??-              bTagEnd          ` ??.           	   bTagSlash          ` ??/              bTagOpen          ` ??0                 bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??              sNetId            
   T_AmsNetId   ??           ams net id of target system       bBusy            ??              bError            ??	              nErrorID           ??
           
   stDevIdent                              ST_DeviceIdentification   ??       5    structure with available device identification data             ??P  ?   ????           FB_GETDEVICEIDENTIFICATIONEX        	   iDataSize       @` ??           
   byTagStart    <    ` ??           '<'    byTagEnd    >    ` ??           '>' 
   byTagSlash    /    ` ??           '/' 	   fbAdsRead                          ADSREAD ` ??              bExecutePrev          ` ??              iState         ` ??              iData   	                      ` ??           
   strActPath             ` ??              iLoopEndIdx         ` ??              iStructSize         ` ??              strHardwareCPU             ` ??              strTags   	  	        )       )          ` ??           	   iTagsSize   	  	                     ` ??              iCurrTag   	  (                     ` ??               iCurrTagData   	  P                     ` ??!           	   iPathSize         ` ??"              iTagIdx         ` ??$              iCurrTagIdx         ` ??%              iDataIdx         ` ??&              iCurrTagDataIdx         ` ??'              k         ` ??(              iMinCurrData         ` ??)           	   iFirstIdx         ` ??*              iLastIdx         ` ??+           	   bTagStart          ` ??-              bTagEnd          ` ??.           	   bTagSlash          ` ??/              bTagOpen          ` ??0                 bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??              sNetId            
   T_AmsNetId   ??           Ams net id of target system       bBusy            ??              bError            ??	              nErrorID           ??
           
   stDevIdent                              ST_DeviceIdentificationEx   ??       5    structure with available device identification data             ??P  ?   ????           FB_GETHOSTADDRBYNAME           fbAdsRW       j    ( PORT:= AMSPORT_R3_SYSSERV, IDXGRP:= SYSTEMSERVICE_IPHELPERAPI, IDXOFFS:= IPHELPERAPI_IPADDRBYHOSTNAME )              	   T_AmsPort         ?                
   ADSRDWRTEX ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id) 	   sHostName           ''       T_MaxString   ??       1    String containing host name. E.g. 'DataServer1'    bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ȯ     ??           Max fb execution time       bBusy            ??
              bError            ??              nErrID           ??              sAddr           ''    
   T_IPv4Addr   ??       S    String containing an (Ipv4) Internet Protocol dotted address. E.g. '172.16.7.199'    arrAddr           0, 0, 0, 0       T_IPv4AddrArr   ??       C    Byte array containing an (Ipv4) Internet Protocol dotted address.             ??P  ?   ????           FB_GETHOSTNAME        	   fbAdsRead       R    ( PORT :=  AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_IPHOSTNAME, IDXOFFS := 0 )              	   T_AmsPort         ?                 ADSREAD ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??	              bError            ??
              nErrID           ??           	   sHostName               T_MaxString   ??           The local host name             ??P  ?   ????           FB_GETLOCALAMSNETID           fbRegQueryValue       W    ( sNetId:= '', sSubKey := 'SOFTWARE\Beckhoff\TwinCAT\System', sValName := 'AmsNetId' )                    
   T_AmsNetId                    T_MaxString                    T_MaxString                   FB_RegQueryValue ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              tmpBytes               T_AmsNetIdArr ` ??                 bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeOut    ?     ??           Max fb execution time       bBusy            ??              bError            ??	              nErrId           ??
           
   AddrString           '0.0.0.0.0.0'    
   T_AmsNetId   ??       -    TwinCAT -specific network address as string 	   AddrBytes           0,0,0,0,0,0       T_AmsNetIdArr   ??       3    TwinCAT-specific network address as array of byte             ??P  ?   ????           FB_GETROUTERSTATUSINFO        	   fbAdsRead       &    ( PORT:= 1, IDXGRP:= 1, IDXOFFS:= 1 )              	   T_AmsPort                          ADSREAD ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              adsRes   	                       ` ??                 sNetId           ''    
   T_AmsNetID   ??           Ams net id    bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??	              bError            ??
              nErrID           ??              info                   ST_TcRouterStatusInfo   ??       #    TwinCAT Router status information             ??P  ?   ????           FB_GETTIMEZONEINFORMATION        	   fbAdsRead       p    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_TIMESERVICES, IDXOFFS := TIMESERVICE_TIMEZONINFORMATION )              	   T_AmsPort         ?                ADSREAD ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              res                ST_AmsGetTimeZoneInformation ` ??                 sNetID            
   T_AmsNetID   ??       &    TwinCAT network address (ams net id)    bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??              bError            ??	              nErrID           ??
              tzID               E_TimeZoneID   ??              tzInfo                     ST_TimeZoneInformation   ??                       ??P  ?   ????           FB_HASHTABLECTRL           p                     T_HashTableEntry      ` ??              n                     T_HashTableEntry      ` ??              nHash         ` ??                 key           ??       d    Entry key: used by A_Lookup, A_Remove method, the key.lookup variable is also used by A_Add method    putValue           ??           Entry value 	   putPosPtr                     T_HashTableEntry        ??	                 bOk            ??           TRUE = success, FALSE = error    getValue           ??           	   getPosPtr                     T_HashTableEntry        ??       R    returned by A_GetFirstEntry, A_GetNextEntry, A_Add, A_Lookup and A_Remove method       hTable         	               T_HHASHTABLE  ??           Hash table handle variable         ??P  ?    ????           FB_LINKEDLISTCTRL           p                   T_LinkedListEntry      ` ??           Temp. previous node    n                   T_LinkedListEntry      ` ??           Temp. next node       putValue           ??           Linked list node value 	   putPosPtr                   T_LinkedListEntry        ??           Linked list node pointer       bOk            ??           TRUE = success, FALSE = error    getValue           ??           Linked list node value 	   getPosPtr                   T_LinkedListEntry        ??           Linked list node pointer       hList         	               T_HLINKEDLIST  ??           Linked list table handle         ??P  ?    ????           FB_LOCALSYSTEMTIME     	      rtrig                 R_TRIG ` ??              state         ` ??              fbNT                   
   NT_GetTime ` ??              fbTZ                          FB_GetTimeZoneInformation ` ??              fbSET                           NT_SetTimeToRTCTime ` ??              fbRTC                             RTC_EX2 ` ??              timer                    TON ` ??              nSync         ` ??              bNotSup          ` ??                 sNetID           ''    
   T_AmsNetID   ??       +    The target TwinCAT system network address    bEnable            ??       `    Enable/start cyclic time synchronisation (output is synchronized to Local Windows System Time)    dwCycle           ?Q            ??       &    Time synchronisation cycle (seconds)    dwOpt          ??       R    Additional option flags: If bit 0 is set => Synchronize Windows Time to RTC time    tTimeout    ?     ??       J    Max. ADS function block execution time (internal communication timeout).       bValid            ??       \    TRUE => The systemTime and tzID output is valid, FALSE => systemTime and tzID is not valid 
   systemTime                   
   TIMESTRUCT   ??       "    Local Windows System Time struct    tzID           eTimeZoneID_Invalid       E_TimeZoneID   ??       )    Daylight/standard time zone information             ??P  ?   ????           FB_MEMBUFFERMERGE           pDest         ` ??              cbDest         ` ??                 eCmd           eEnumCmd_First       E_EnumCmdType   ??              pBuffer           ??           Pointer to destination buffer    cbBuffer           ??       &    Max. byte size of destination buffer    pSegment           ??       .    Pointer to data segment (optional, may be 0) 	   cbSegment           ??       -    Number of data segments (optional, may be 0)      bOk            ??       M    TRUE => Successfull, FALSE => End of enumeration or invalid input parameter    cbSize           ??           Data buffer fill state             ??P  ?    ????           FB_MEMBUFFERSPLIT           pSrc         ` ??              cbSrc         ` ??                 eCmd           eEnumCmd_First       E_EnumCmdType   ??              pBuffer           ??           Pointer to source data buffer    cbBuffer           ??       !    Byte size of source data buffer    cbSize           ??           Max. segment byte size       bOk            ??
       N    TRUE => Successfull, FALSE => End of segmentation or invalid input parameter    pSegment           ??           Pointer to data segment 	   cbSegment           ??           Byte size of data segment    bEOS            ??       7    TRUE = End/last segment, FALSE = Next segment follows             ??P  ?    ????           FB_MEMRINGBUFFER           idxLast         ` ??            byte index of last buffer byte    idxFirst         ` ??       "    byte buffer of first buffer byte    idxGet         ` ??              pTmp         ` ??              cbTmp         ` ??              cbCopied         ` ??                 pWrite           ??           pointer to write data    cbWrite           ??           byte size of write data    pRead           ??	           pointer to read data buffer    cbRead           ??
           byte size of read data buffer    pBuffer           ??       #    pointer to ring buffer data bytes    cbBuffer           ??           max. ring buffer byte size       bOk            ??       T    TRUE = new entry added or removed succesfully, FALSE = fifo overflow or fifo empty    nCount           ??           number of fifo entries    cbSize           ??       "    current byte length of fifo data    cbReturn           ??       ?    If bOk == TRUE => Number of recend realy returned (removed or get) data bytes
									   If bOk == FALSE and cbReturn <> 0 => Number of required read buffer data bytes (cbRead underflow)             ??P  ?    ????           FB_MEMRINGBUFFEREX           idxLast         ` ??       *    byte index of last (newest) buffer entry    idxFirst         ` ??       +    byte index of first (oldest) buffer entry    idxGet         ` ??           temporary index    idxEnd         ` ??       "    index of unused/free end segment    cbEnd         ` ??       &    byte size of unused/free end segment    cbAdd         ` ??!                 pWrite           ??           pointer to write data    cbWrite           ??           byte size of write data    pBuffer           ??       #    pointer to ring buffer data bytes    cbBuffer           ??           max. ring buffer byte size       bOk            ??       W    TRUE = new entry added or get, freed succesfully, FALSE = fifo overflow or fifo empty    pRead           ??       (    A_GetHead returns pointer to read data    cbRead           ??       *    A_GetHead returns byte size of read data    nCount           ??           number of fifo entries    cbSize           ??       "    current byte length of fifo data    cbFree           ??            biggest available free segment             ??P  ?    ????           FB_MEMSTACKBUFFER               pWrite           ??           pointer to write data    cbWrite           ??           byte size of write data    pRead           ??	           pointer to read data buffer    cbRead           ??
           byte size of read data buffer    pBuffer           ??       #    pointer to LIFO buffer data bytes    cbBuffer           ??           max. LIFO buffer byte size       bOk            ??       T    TRUE = new entry added or removed succesfully, FALSE = LIFO overflow or LIFO empty    nCount           ??           number of LIFO entries    cbSize           ??       "    current byte length of LIFO data    cbReturn           ??       ?    If bOk == TRUE => Number of recend realy returned (removed or get) data bytes
									   If bOk == FALSE and cbReturn <> 0 => Number of required read buffer data bytes (cbRead underflow)             ??P  ?    ????           FB_REGQUERYVALUE           fbAdsRdWrtEx       [    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_REG_HKEYLOCALMACHINE, IDXOFFS := 0 )              	   T_AmsPort         ?                  
   ADSRDWRTEX ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              s1Len         ` ??              s2Len         ` ??              ptr         ` ??              cbBuff         ` ??              tmpBuff                ST_HKeySrvRead ` ??                 sNetId            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    sSubKey               T_MaxString   ??       #    HKEY_LOCAL_MACHINE \ sub key name    sValName               T_MaxString   ??           Value name    cbData           ??           Number of data bytes to read    pData           ??       $    Points to registry key data buffer    bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeOut    ?     ??	           Max fb execution time       bBusy            ??              bError            ??              nErrId           ??              cbRead           ??       '    Number of succesfully read data bytes             ??P  ?   ????           FB_REGSETVALUE        
   fbAdsWrite       [    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_REG_HKEYLOCALMACHINE, IDXOFFS := 0 )              	   T_AmsPort         ?                  ADSWRITE ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              s1Len         ` ??              s2Len         ` ??              s3Len         ` ??              ptr         ` ??              nType         ` ??              cbBuff         ` ??              cbRealWrite         ` ??              tmpBuff                  ST_HKeySrvWrite ` ??                 sNetId            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    sSubKey               T_MaxString   ??       #    HKEY_LOCAL_MACHINE \ sub key name    sValName               T_MaxString   ??           Value name    eValType               E_RegValueType   ??           Value type    cbData           ??           Size of value data in bytes    pData           ??           Pointer to value data buffer   bExecute            ??	       6    Rising edge on this input activates the fb execution    tTimeOut    ?     ??
           Max fb execution time       bBusy            ??              bError            ??              nErrId           ??              cbWrite           ??       +    Number of successfully written data bytes             ??P  ?   ????           FB_REMOVEROUTEENTRY        
   fbAdsWrite       P    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_DELREMOTE, IDXOFFS := 0 )              	   T_AmsPort         "                 ADSWRITE ` ??                 sNetID            
   T_AmsNetID   ??       '    TwinCAT network address (ams net id )    sName                 ??           Route name as string    bExecute            ??       -    Rising edge starts function block execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??
              bError            ??              nErrID           ??                       ??P  ?   ????           FB_RESETSCOPESERVERCONTROL        
   fbAdsWrite                          ADSWRITE    ??              nState            ??                 sNetId            
   T_AmsNetId   ??              bExecute            ??              tTimeout    ?     ??                 bBusy            ??              bDone            ??	              bError            ??
              nErrorId           ??                       ??P  ?   ????           FB_SAVESCOPESERVERDATA           nState            ??           
   fbAdsWrite       D    ( PORT := AMSPORT_R3_SCOPESERVER, IDXGRP := 16#750E, IDXOFFS := 0 )              	   T_AmsPort         u                 ADSWRITE    ??                 sNetId            
   T_AmsNetId   ??              bExecute            ??           	   sSaveFile    Q       Q    ??              tTimeout    ?     ??                 bBusy            ??	              bDone            ??
              bError            ??              nErrorId           ??                       ??P  ?   ????           FB_SCOPESERVERCONTROL           eCurrentState           SCOPE_SERVER_IDLE       E_ScopeServerState    ??           	   fbConnect                                   FB_ConnectScopeServer    ??              fbStart        
                FB_StartScopeServer    ??              fbStop        
                FB_StopScopeServer    ??              fbSave        
                FB_SaveScopeServerData    ??              fbDisconnect        	               FB_DisconnectScopeServer    ??              fbReset        	               FB_ResetScopeServerControl    ??                  sNetId            
   T_AmsNetId   ??           	   eReqState               E_ScopeServerState   ??              sConfigFile    Q       Q    ??           	   sSaveFile    Q       Q    ??              tTimeout    ?     ??                 bBusy            ??              bDone            ??              bError            ??              nErrorId           ??                       ??P  ?   ????           FB_SETTIMEZONEINFORMATION        
   fbAdsWrite       o    ( PORT:= AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_TIMESERVICES, IDXOFFS	:= TIMESERVICE_TIMEZONINFORMATION )              	   T_AmsPort         ?                ADSWRITE ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              req                ST_AmsGetTimeZoneInformation ` ??                 sNetID           ''    
   T_AmsNetID   ??       &    TwinCAT network address (ams net id)    tzInfo       ]   ( (*West Euoropa Standard Time *)
					bias:=-60,
					standardName:='W. Europe Standard Time',
					standardDate:=(wYear:=0, wMonth:=10, wDayOfWeek:=0, wDay:=5, wHour:=3),
					standardBias:=0,
					daylightName:='W. Europe Daylight Time',
					daylightDate:=(wYear:=0, wMonth:=3, wDayOfWeek:=0, wDay:=5, wHour:=2),
					daylightBias:=-60 )    ????        W. Europe Standard Time                
   TIMESTRUCT             
                                W. Europe Daylight Time                
   TIMESTRUCT                                 ????   ST_TimeZoneInformation   ??              bExecute            ??       6    Rising edge on this input activates the fb execution    tTimeout    ?     ??           Max fb execution time       bBusy            ??              bError            ??              nErrID           ??                       ??P  ?   ????           FB_STARTSCOPESERVER           nState            ??           
   fbAdsWrite                          ADSWRITE    ??              nDummy   	                    0,0                     ??                 sNetId            
   T_AmsNetId   ??              bExecute            ??              tTimeout    ?     ??                 bBusy            ??              bDone            ??	              bError            ??
              nErrorId           ??                       ??P  ?   ????           FB_STOPSCOPESERVER           nState            ??           
   fbAdsWrite                          ADSWRITE    ??              nDummy   	                    0,0                     ??                 sNetId            
   T_AmsNetId   ??              bExecute            ??              tTimeout    ?     ??                 bBusy            ??              bDone            ??	              bError            ??
              nErrorId           ??                       ??P  ?   ????           FB_STRINGRINGBUFFER           fbBuffer                              FB_MemRingBuffer ` ??       4    Internal (low level) buffer control function block    
   bOverwrite            ??       8    TRUE = overwrite oldest entry, FALSE = don't overwrite    putValue           ''       T_MaxString   ??       %    String to add (write) to the buffer    pBuffer           ??	       #    Pointer to ring buffer data bytes    cbBuffer           ??
           Max. ring buffer byte size       bOk            ??       T    TRUE = new entry added or removed succesfully, FALSE = fifo overflow or fifo empty    getValue           ''       T_MaxString   ??       #    String removed (read) from buffer    nCount           ??           Number of fifo entries    cbSize           ??       "    Current byte length of fifo data             ??P  ?    ????        "   FB_SYSTEMTIMETOTZSPECIFICLOCALTIME           fbBase                                   "   FB_TranslateUtcToLocalTimeByZoneID ` ??           Underlaid base function block       in                   
   TIMESTRUCT   ??       p    Time to be converted (UTC, system time format). Structure that specifies the system time since January 1, 1601    tzInfo                     ST_TimeZoneInformation   ??           Time zone settings       out                   
   TIMESTRUCT   ??       ,    Converted time in local system time format    eTzID           eTimeZoneID_Unknown       E_TimeZoneID   ??	       "    Daylight saving time information    bB            ??
            FALSE => A time, TRUE => B time            ??P  ?    ????           FB_TEXTFILERINGBUFFER           fbOpen                             FB_FileOpen ` ??              fbClose                      FB_FileClose ` ??              fbPuts        	               FB_FilePuts ` ??              nStep         ` ??       @    0=idle, 1=init, 10,11=open, 40,41=write, 50,51=close, 100=exit    eCmd           eFileRBuffer_None       E_FileRBufferCmd ` ??                 sNetId           ''    
   T_AmsNetId ` ??           ams net id 	   sPathName           'c:\Temp\data.dat'       T_MaxString ` ??       6    file buffer path name (max. length = 255 characters)    ePath           PATH_GENERIC    
   E_OpenPath ` ??           default: Open generic file    bAppend         ` ??       #    TRUE = append, FALSE = not append    putLine           ''       T_MaxString ` ??	              cbBuffer        ` ??
       5    max. file buffer byte size(RESERVED for future use)    tTimeout    ?   ` ??                 bBusy          ` ??              bError          ` ??              nErrID         ` ??              bOpened          ` ??       )    TRUE = file opened, FALSE = file closed    getLine           ''       T_MaxString ` ??                       ??P  ?   ????        "   FB_TRANSLATELOCALTIMETOUTCBYZONEID           inLocal                   
   TIMESTRUCT ` ??       9    Input time in local system time format (time structure) 	   tziSommer                   
   TIMESTRUCT ` ??       A    tzInfo.daylightDate transition time in local system time format 	   tziWinter                   
   TIMESTRUCT ` ??       A    tzInfo.standardDate transition time in local system time Format    tziLocalSommer             
   T_FILETIME ` ??       ?    tzInfo.daylightDate transition time in local file time format    tziLocalWinter             
   T_FILETIME ` ??       ?    tzInfo.standardDate transition time in local file time Format    tziLocalSommerJump             
   T_FILETIME ` ??              tziLocalWinterJump             
   T_FILETIME ` ??              ui64LocalIn                T_ULARGE_INTEGER ` ??       (    Local input time as unsigned 64 number    ui64LocalSommer                T_ULARGE_INTEGER ` ??       5    Local tzInfo.daylightDate as unsigned 64 bit number    ui64LocalWinter                T_ULARGE_INTEGER ` ??       5    Local tzInfo.standardDate as unsigned 64 bit number    in_to_s         ` ??       <    Input time[Local] to tzInfo.daylightDate[Local] cmp result    in_to_w         ` ??       <    Input time[Local] to tzInfo.standardDate[Local] cmp result    s_to_w         ` ??       E    tzInfo.daylightDate[Local] to tzInfo.standardDate[Local] cmp result    in_to_s_jump         ` ??        2    Input time[Local] to tzInfo jump time cmp result    in_to_w_jump         ` ??!       2    Input time[Local] to tzInfo jump time cmp result    iStandardBias         ` ??#              iDaylightBias         ` ??$              ui64PreviousIn                T_ULARGE_INTEGER ` ??&              ui64FallDiff                T_ULARGE_INTEGER ` ??'           	   bFallDiff          ` ??(           Nur zu Testzwecken    dtSommerJump         ` ??*              dtWinterJump         ` ??+                 in             
   T_FILETIME   ??       /    Time to be converted (Local file time format)    tzInfo                     ST_TimeZoneInformation   ??           Time zone information    wDldYear           ??       p    Optional daylightDate.wYear value. If 0 => not used (default) else used only if tzInfo.daylightDate.wYear = 0.    wStdYear           ??       p    Optional standardDate.wYear value. If 0 => not used (default) else used only if tzInfo.standardDate.wYear = 0.       out             
   T_FILETIME   ??
       '    Converted time (UTC file time format)    eTzID           eTimeZoneID_Unknown       E_TimeZoneID   ??       +    Detected daylight saving time information    bB            ??            FALSE => A time, TRUE => B time   bias           ??           Bias value in minutes             ??P  ?    ????        "   FB_TRANSLATEUTCTOLOCALTIMEBYZONEID           inUtc                   
   TIMESTRUCT ` ??       7    Input time in UTC system time format (time structure)    bInAsStruct          ` ??       k    TRUE => inUtc is valid/converted to UTC system time format, FALSE => inUtc is not valid/not converted yet 	   tziSommer                   
   TIMESTRUCT ` ??       A    tzInfo.daylightDate transition time in local system time format 	   tziWinter                   
   TIMESTRUCT ` ??       A    tzInfo.standardDate transition time in local system time Format    tziLocalSommer             
   T_FILETIME ` ??       ?    tzInfo.daylightDate transition time in local file time format    tziLocalWinter             
   T_FILETIME ` ??       ?    tzInfo.standardDate transition time in local file time Format    tziUtcSommer             
   T_FILETIME ` ??       =    tzInfo.daylightDate transition time in UTC file time format    tziUtcWinter             
   T_FILETIME ` ??       =    tzinfo.standardDate transition time in UTC file time format 	   ui64UtcIn                T_ULARGE_INTEGER ` ??       &    UTC input time as unsigned 64 number    ui64UtcSommer                T_ULARGE_INTEGER ` ??       3    UTC tzInfo.daylightDate as unsigned 64 bit number    ui64UtcWinter                T_ULARGE_INTEGER ` ??       3    UTC tzInfo.standardDate as unsigned 64 bit number    in_to_s         ` ??       8    Input time[UTC] to tzInfo.daylightDate[UTC] cmp result    in_to_w         ` ??       8    Input time[UTC] to tzInfo.standardDate[UTC] cmp result    s_to_w         ` ??        A    tzInfo.daylightDate[UTC] to tzInfo.standardDate[UTC] cmp result    out_to_s         ` ??!       =    Output time[local] to tzInfo.daylightDate[local] cmp result    out_to_w         ` ??"       =    Output time[local] to tzInfo.standardDate[local] cmp result       in             
   T_FILETIME   ??       .    Time to be converted (UTC, file time format)    tzInfo                     ST_TimeZoneInformation   ??           Time zone information    wDldYear           ??       p    Optional daylightDate.wYear value. If 0 => not used (default) else used only if tzInfo.daylightDate.wYear = 0.    wStdYear           ??       p    Optional standardDate.wYear value. If 0 => not used (default) else used only if tzInfo.standardDate.wYear = 0.       out             
   T_FILETIME   ??
       (    Converted time (local file time format)   eTzID           eTimeZoneID_Unknown       E_TimeZoneID   ??       0    Detected daylight saving time/zone information    bB            ??            FALSE => A time, TRUE => B time   bias           ??           Bias value in minutes             ??P  ?    ????            FB_TZSPECIFICLOCALTIMETOFILETIME           fbBase       !    ( wStdYear := 0, wDldYear := 0 )                                         "   FB_TranslateLocalTimeToUtcByZoneID ` ??           Underlaid base function block       in             
   T_FILETIME   ??       }    Time zone's specific local file time. 64-bit value representing the number of 100-nanosecond intervals since January 1, 1601   tzInfo                     ST_TimeZoneInformation   ??           Time zone settings       out             
   T_FILETIME   ??       E    Converted time in Coordinated Universal Time (UTC) file time format    eTzID           eTimeZoneID_Unknown       E_TimeZoneID   ??	       "    Daylight saving time information    bB            ??
            FALSE => A time, TRUE => B time            ??P  ?    ????        "   FB_TZSPECIFICLOCALTIMETOSYSTEMTIME           fbBase                                         "   FB_TranslateLocalTimeToUtcByZoneID ` ??           Underlaid base function block       in                   
   TIMESTRUCT   ??       g    Time zone's specific local system time. Structure that specifies the system time since January 1, 1601   tzInfo                     ST_TimeZoneInformation   ??           Time zone settings       out                   
   TIMESTRUCT   ??       8    Coordinated Universal Time (UTC) in system time format    eTzID           eTimeZoneID_Unknown       E_TimeZoneID   ??	       "    Daylight saving time information    bB            ??
            FALSE => A time, TRUE => B time            ??P  ?    ????           FB_WRITEPERSISTENTDATA           fbAdsWrtCtl       9    ( ADSSTATE := ADSSTATE_SAVECFG, LEN := 0, SRCADDR := 0 )                          	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??       l    Contains the ADS port number of the PLC run-time system whose persistent data is to be stored (801, 811...)   START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time    MODE           SPDM_2PASS       E_PersistentMode   ??       D    =SPDM_2PASS: optimized boost; =SPDM_VAR_BOOST: boost per variable;       BUSY            ??              ERR            ??              ERRID           ??                       ??P  ?   ????           FILETIME_TO_DT           ui64                T_ULARGE_INTEGER ` ??	                 fileTime             
   T_FILETIME   ??           Windows file time.       FILETIME_TO_DT                                     ??P  ?    ????           FILETIME_TO_SYSTEMTIME     	      D         ` ??              M         ` ??              Y         ` ??           
   uiPastDays                T_ULARGE_INTEGER ` ??	              uiPastYears                T_ULARGE_INTEGER ` ??
              uiRemainder                T_ULARGE_INTEGER ` ??           
   dwPastDays         ` ??              dwPastYears         ` ??           
   dwYearDays         ` ??                 fileTime             
   T_FILETIME   ??                 FILETIME_TO_SYSTEMTIME                   
   TIMESTRUCT                             ??P  ?    ????           FIX16_TO_LREAL               in                 T_FIX16   ??                 FIX16_TO_LREAL                                                  ??P  ?    ????           FIX16_TO_WORD               in                 T_FIX16   ??           16 bit fixed point number       FIX16_TO_WORD                                     ??P  ?    ????           FIX16ADD               augend                 T_FIX16   ??              addend                 T_FIX16   ??                 FIX16Add                 T_FIX16                             ??P  ?    ????        
   FIX16ALIGN               in                 T_FIX16   ??       #    16 bit signed fixed point number.    n                           ??       ,    Number of fractional bits (decimal places)    
   FIX16Align                 T_FIX16                             ??P  ?    ????           FIX16DIV           tmpA         ` ??	                 dividend                 T_FIX16   ??              divisor                 T_FIX16   ??                 FIX16Div                 T_FIX16                             ??P  ?    ????           FIX16MUL           tmp         ` ??	                 multiA                 T_FIX16   ??              multiB                 T_FIX16   ??                 FIX16Mul                 T_FIX16                             ??P  ?    ????           FIX16SUB               minuend                 T_FIX16   ??           
   subtrahend                 T_FIX16   ??                 FIX16Sub                 T_FIX16                             ??P  ?    ????           GETREMOTEPCINFO        	   fbAdsRead       #    ( PORT:=1, IDXGRP:=3, IDXOFFS:=1 )              	   T_AmsPort                          ADSREAD ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??           
   RouterInfo   	  c            
                ST_AmsRouterInfoEntry         ` ??              iIndex         ` ??                 NETID            
   T_AmsNetId   ??       D    Target NetID, usually left as empty string for reading local infos    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??       
    Ads busy    ERR            ??	           Ads error    ERRID           ??
           Ads Error    RemotePCInfo               REMOTEPCINFOSTRUCT   ??       N    field with all NetID?s and PC names found in router, ordered as router gives             ??P  ?   ????           HEXSTR_TO_DATA           pSrc               ` ??
              pDest               ` ??              ascii         ` ??              nibble         ` ??              bAdd          ` ??              bLN          ` ??           hi/lo nibble       sHex               T_MaxString   ??           Hex string to convert    pData           ??           Pointer to destination buffer    cbData           ??       !    Byte size of destination buffer       HEXSTR_TO_DATA                                     ??P  ?    ????           HOST_TO_BE128               in                T_UHUGE_INTEGER   ??                 HOST_TO_BE128                T_UHUGE_INTEGER                             ??P  ?    ????           HOST_TO_BE16               in           ??                 HOST_TO_BE16                                     ??P  ?    ????           HOST_TO_BE32           parr    	                            ` ??                 in           ??                 HOST_TO_BE32                                     ??P  ?    ????           HOST_TO_BE64               in                T_ULARGE_INTEGER   ??                 HOST_TO_BE64                T_ULARGE_INTEGER                             ??P  ?    ????           INT64_TO_LREAL               in                T_LARGE_INTEGER   ??                 INT64_TO_LREAL                                                  ??P  ?    ????        
   INT64ADD64           bOV          ` ??	                 i64a                T_LARGE_INTEGER   ??              i64b                T_LARGE_INTEGER   ??              
   Int64Add64                T_LARGE_INTEGER                             ??P  ?    ????           INT64ADD64EX               augend                T_LARGE_INTEGER   ??              addend                T_LARGE_INTEGER   ??                 Int64Add64Ex                T_LARGE_INTEGER                       bOV            ??       3    TRUE => arithmetic overflow, FALSE => no overflow         ??P  ?    ????        
   INT64CMP64               i64a                T_LARGE_INTEGER   ??              i64b                T_LARGE_INTEGER   ??	              
   Int64Cmp64                                     ??P  ?    ????           INT64DIV64EX           bIsNegative          ` ??           
   sRemainder                T_ULARGE_INTEGER ` ??                 dividend                T_LARGE_INTEGER   ??              divisor                T_LARGE_INTEGER   ??                 Int64Div64Ex                T_LARGE_INTEGER                    	   remainder                 T_LARGE_INTEGER  ??                   ??P  ?    ????           INT64ISZERO               i64                T_LARGE_INTEGER   ??                 Int64IsZero                                      ??P  ?    ????           INT64NEGATE               i64                T_LARGE_INTEGER   ??                 Int64Negate                T_LARGE_INTEGER                             ??P  ?    ????           INT64NOT               i64                T_LARGE_INTEGER   ??                 Int64Not                T_LARGE_INTEGER                             ??P  ?    ????        
   INT64SUB64               i64a                T_LARGE_INTEGER   ??       	    minuend    i64b                T_LARGE_INTEGER   ??           substrahend    
   Int64Sub64                T_LARGE_INTEGER                             ??P  ?    ????           ISFINITE        	   ptrDouble    	                            ` ??           	   ptrSingle               ` ??	                 x                 T_ARG   ??                 IsFinite                                      ??P  ?    ????           LARGE_INTEGER            
   dwHighPart           ??           	   dwLowPart           ??                 LARGE_INTEGER                T_LARGE_INTEGER                             ??P  ?    ????           LARGE_TO_ULARGE               in                T_LARGE_INTEGER   ??                 LARGE_TO_ULARGE                T_ULARGE_INTEGER                             ??P  ?    ????           LREAL_TO_FIX16               in                        ??           LREAL number to convert    n                          ??       ,    Number of fractional bits (decimal places)       LREAL_TO_FIX16                 T_FIX16                             ??P  ?    ????           LREAL_TO_FMTSTR           rec              
   T_FloatRec ` ??              pOut               ` ??              iStart         ` ??              iEnd         ` ??              i         ` ??                 in                        ??
           
   iPrecision           ??              bRound            ??                 LREAL_TO_FMTSTR    ?      ?                             ??P  ?    ????           LREAL_TO_INT64               in                        ??                 LREAL_TO_INT64                T_LARGE_INTEGER                             ??P  ?    ????           LREAL_TO_UINT64           tmp                      ` ??                 in                        ??                 LREAL_TO_UINT64                T_ULARGE_INTEGER                             ??P  ?    ????           MAXSTRING_TO_BYTEARR           cbCopy         ` ??           	   Index7001                            in               T_MaxString   ??                 MAXSTRING_TO_BYTEARR   	  ?                                                 ??P  ?   ????           NT_ABORTSHUTDOWN           fbAdsWrtCtl       N    ( PORT := AMSPORT_R3_SYSSERV, ADSSTATE := ADSSTATE_POWERGOOD, DEVSTATE := 0 )              	   T_AmsPort         
               	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??              ERR            ??	              ERRID           ??
                       ??P  ?   ????        
   NT_GETTIME        	   fbAdsRead       i    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_TIMESERVICES, IDXOFFS := TIMESERVICE_DATEANDTIME )              	   T_AmsPort         ?                ADSREAD ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??              ERR            ??	              ERRID           ??
              TIMESTR                   
   TIMESTRUCT   ??           Local windows system time             ??P  ?   ????        	   NT_REBOOT           fbAdsWrtCtl       M    ( PORT := AMSPORT_R3_SYSSERV, ADSSTATE := ADSSTATE_SHUTDOWN, DEVSTATE := 1 )              	   T_AmsPort                       	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    DELAY           ??           Reboot delay time [seconds]    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           NT_SETLOCALTIME        
   fbAdsWrite       d    ( PORT:= AMSPORT_R3_SYSSERV, IDXGRP:= SYSTEMSERVICE_TIMESERVICES, IDXOFFS:=TIMESERVICE_DATEANDTIME)              	   T_AmsPort         ?                ADSWRITE ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    TIMESTR                   
   TIMESTRUCT   ??           New local system time    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           NT_SETTIMETORTCTIME        
   fbAdsWrite       :    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := 4, IDXOFFS := 0 )              	   T_AmsPort                           ADSWRITE ` ??           
   fbRegQuery       K    ( sSubKey := 'Software\Beckhoff\TwinCAT\System', sValName := 'NumOfCPUs' )                        T_MaxString                    T_MaxString                   FB_RegQueryValue ` ??           	   fbTrigger                 R_TRIG ` ??              bTmp         ` ??              state         ` ??              bInit         ` ??           	   numOfCPUs         ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    SET            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??              ERR            ??	              ERRID           ??
                       ??P  ?   ????           NT_SHUTDOWN           fbAdsWrtCtl       M    ( PORT := AMSPORT_R3_SYSSERV, ADSSTATE := ADSSTATE_SHUTDOWN, DEVSTATE := 0 )              	   T_AmsPort                        	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    DELAY           ??           Shutdown delay time [seconds]    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           NT_STARTPROCESS        
   fbAdsWrite       O    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP:=SYSTEMSERVICE_STARTPROCESS, IDXOFFS:=0 )              	   T_AmsPort         ?                 ADSWRITE ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              LenPath         ` ??              LenDir         ` ??           
   LenComLine         ` ??              req                  ST_AmsStartProcessReq ` ??           data buffer       NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PATHSTR               T_MaxString   ??              DIRNAME               T_MaxString   ??           	   COMNDLINE               T_MaxString   ??              START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??              ERR            ??              ERRID           ??                       ??P  ?   ????           OTSTRUCT_TO_TIME           tmpMilli         ` ??                 OTIN                    OTSTRUCT   ??                 OTSTRUCT_TO_TIME                                     ??P  ?    ????           PBOOL_TO_BOOL               in                  ??                 PBOOL_TO_BOOL                                      ??P  ?    ????           PBYTE_TO_BYTE               in                 ??                 PBYTE_TO_BYTE                                     ??P  ?    ????           PDATE_TO_DATE               in                 ??                 PDATE_TO_DATE                                     ??P  ?    ????           PDINT_TO_DINT               in                 ??                 PDINT_TO_DINT                                     ??P  ?    ????        	   PDT_TO_DT               in                 ??              	   PDT_TO_DT                                     ??P  ?    ????           PDWORD_TO_DWORD               in                 ??                 PDWORD_TO_DWORD                                     ??P  ?    ????           PHUGE_TO_HUGE               in                 T_HUGE_INTEGER        ??                 PHUGE_TO_HUGE                T_HUGE_INTEGER                             ??P  ?    ????           PINT_TO_INT               in                 ??                 PINT_TO_INT                                     ??P  ?    ????           PLARGE_TO_LARGE               in                 T_LARGE_INTEGER        ??                 PLARGE_TO_LARGE                T_LARGE_INTEGER                             ??P  ?    ????           PLC_READSYMINFO        	   fbAdsRead       3    ( IDXGRP := ADSIGRP_SYM_UPLOADINFO, IDXOFFS := 0 )       ?                 ADSREAD ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              SymInfoStruct   	                       ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??              START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??              SYMCOUNT           ??              SYMSIZE           ??                       ??P  ?   ????           PLC_READSYMINFOBYNAME           fbReadEx                                       PLC_ReadSymInfoByNameEx ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??              SYMNAME               T_MaxString   ??              START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??	           Max fb execution time       BUSY            ??              ERR            ??              ERRID           ??              SYMINFO                     SYMINFOSTRUCT   ??                       ??P  ?   ????           PLC_READSYMINFOBYNAMEEX        
   fbAdsRdWrt       5    ( IDXGRP := ADSIGRP_SYM_INFOBYNAMEEX, IDXOFFS := 0 )       	?                   ADSRDWRT ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              symInfoBuffer                            ST_AmsSymbolInfoEntry ` ??           
   nameLength         ` ??           
   typeLength         ` ??              commentLength         ` ??              nameAdrOffset         ` ??              typeAdrOffset         ` ??              commentAdrOffset         ` ??              nameCpyLength         ` ??              typeCpyLength         ` ??               commentCpyLength         ` ??!              endOfBufAdrOffset         ` ??"                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??              SYMNAME               T_MaxString   ??              START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??	           Max fb execution time       BUSY            ??              ERR            ??              ERRID           ??              SYMINFO                     SYMINFOSTRUCT   ??              OVTYPE            ??       @    TRUE => Type name string length overflow, FALSE => no overflow 	   OVCOMMENT            ??       >    TRUE => Comment string length overflow, FALSE => no overflow             ??P  ?   ????        	   PLC_RESET           fbAdsWrtCtl       F    ( ADSSTATE := ADSSTATE_RESET, DEVSTATE := 0, LEN := 0, SRCADDR := 0 )                              	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??              RESET            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????        	   PLC_START           fbAdsWrtCtl       D    ( ADSSTATE := ADSSTATE_RUN, DEVSTATE := 0, LEN := 0, SRCADDR := 0 )                              	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??              START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           PLC_STOP           fbAdsWrtCtl       E    ( ADSSTATE := ADSSTATE_STOP, DEVSTATE := 0, LEN := 0, SRCADDR := 0 )                              	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??              STOP            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           PLREAL_TO_LREAL               in                              ??                 PLREAL_TO_LREAL                                                  ??P  ?    ????           PMAXSTRING_TO_MAXSTRING               in                 T_MaxString        ??                 PMAXSTRING_TO_MAXSTRING               T_MaxString                             ??P  ?    ????           PREAL_TO_REAL               in                  ??                 PREAL_TO_REAL                                      ??P  ?    ????           PROFILER     
      MAX_DATABUFF_SIZE    d   @  ??              RisingEdgeStart                 R_TRIG ` ??              RisingEdgeReset                 R_TRIG ` ??              FallingEdgeStart                 F_TRIG ` ??              GETCPUACCOUNT1                GETCPUACCOUNT ` ??              OldCpuCntDW         ` ??              MeasureData   	  d                     ` ??              TimeSum         ` ??              MaxData        ` ??              idx         ` ??                 START            ??       0   rising edge starts measurement and falling stops   RESET            ??                 BUSY            ??              DATA                   PROFILERSTRUCT   ??                       ??P  ?   ????           PSINT_TO_SINT               in                 ??                 PSINT_TO_SINT                                     ??P  ?    ????           PSTRING_TO_STRING               in     Q       Q         ??                 PSTRING_TO_STRING    Q       Q                              ??P  ?    ????           PTIME_TO_TIME               in                 ??                 PTIME_TO_TIME                                     ??P  ?    ????           PTOD_TO_TOD               in                 ??                 PTOD_TO_TOD                                     ??P  ?    ????           PUDINT_TO_UDINT               in                 ??                 PUDINT_TO_UDINT                                     ??P  ?    ????           PUHUGE_TO_UHUGE               in                 T_UHUGE_INTEGER        ??                 PUHUGE_TO_UHUGE                T_UHUGE_INTEGER                             ??P  ?    ????           PUINT64_TO_UINT64               in                 T_ULARGE_INTEGER        ??                 PUINT64_TO_UINT64                T_ULARGE_INTEGER                             ??P  ?    ????           PUINT_TO_UINT               in                 ??                 PUINT_TO_UINT                                     ??P  ?    ????           PULARGE_TO_ULARGE               in                 T_ULARGE_INTEGER        ??                 PULARGE_TO_ULARGE                T_ULARGE_INTEGER                             ??P  ?    ????           PUSINT_TO_USINT               in                 ??                 PUSINT_TO_USINT                                     ??P  ?    ????           PWORD_TO_WORD               in                 ??                 PWORD_TO_WORD                                     ??P  ?    ????        
   RAD_TO_DEG               ANGLE                        ??              
   RAD_TO_DEG                                                  ??P  ?    ????           ROUTETRANSPORT_TO_STRING               eType               E_RouteTransportType   ??                 ROUTETRANSPORT_TO_STRING    Q       Q                              ??P  ?    ????           RTC           fbGetCpuCounter                 GETCPUCOUNTER ` ??           
   risingEdge                 R_TRIG ` ??              oldTick         ` ??              currTick         ` ??              nanoDiff         ` ??              nanoRest         ` ??              secDiff         ` ??              init         ` ??                 EN            ??              PDT           ??                 Q            ??              CDT           ??	                       ??P  ?    ????           RTC_EX           fbGetCpuCounter                 GETCPUCOUNTER ` ??           
   risingEdge                 R_TRIG ` ??              oldTick         ` ??              currTick         ` ??              nanoDiff         ` ??              nanoRest         ` ??              secDiff         ` ??              init         ` ??                 EN            ??              PDT           ??              PMSEK           ??                 Q            ??	              CDT           ??
              CMSEK           ??                       ??P  ?    ????           RTC_EX2     	      fbGetCpuCounter                 GETCPUCOUNTER ` ??           
   risingEdge                 R_TRIG ` ??              oldTick         ` ??              currTick         ` ??              nanoDiff         ` ??              nanoRest         ` ??              secDiff         ` ??              dateTime         ` ??              init         ` ??                 EN            ??              PDT                   
   TIMESTRUCT   ??              PMICRO           ??                 Q            ??	              CDT       ;    ( wYear := 1970, wMonth := 1, wDay := 1, wDayOfWeek := 4 )    ?                  
   TIMESTRUCT   ??
              CMICRO           ??                       ??P  ?    ????           SCOPEASCIIEXPORT        
   fbAdsWrite                          ADSWRITE    ??                 bExecute            ??           	   sFilePath               T_MaxString   ??              tTimeout    ?     ??                 bBusy            ??              bError            ??	              iErrorId           ??
                       ??P  ?   ????        	   SCOPEEXIT        
   fbAdsWrite                          ADSWRITE    ??           
   RisingEdge                 R_TRIG    ??              step            ??              fbDelay                    TON    ??                 bExecute            ??       -    Rising edge starts function block execution    tTimeOut    ?     ??       >    Maximum time allowed for the execution of the function block       bBusy            ??              bError            ??              iErrorId           ??	                       ??P  ?   ????           SCOPEGETRECORDLEN        	   fbAdsRead                          ADSREAD    ??                 bExecute            ??                 bBusy            ??              bError            ??              iErrorId           ??           
   fRecordLen                        ??	                       ??P  ?    ????           SCOPEGETSTATE        	   fbAdsRead                          ADSREAD    ??              State            ??                 bExecute            ??                 bBusy            ??              bError            ??              iErrorId           ??              bOnline            ??	                       ??P  ?    ????           SCOPELOADFILE        
   fbAdsWrite                          ADSWRITE    ??                 bExecute            ??           	   sFilePath               T_MaxString   ??              tTimeout    ?     ??                 bBusy            ??              bError            ??	              iErrorId           ??
                       ??P  ?   ????           SCOPEMANUALTRIGGER        
   fbAdsWrite                          ADSWRITE    ??                 bExecute            ??                 bBusy            ??              bError            ??              iErrorId           ??                       ??P  ?    ????           SCOPESAVEAS        
   RisingEdge                 R_TRIG ` ??           
   fbAdsWrite       D    ( NETID := '', PORT := 14000, IDXGRP := 16#2000, IDXOFFS := 16#11 )             
   T_AmsNetId                 	   T_AmsPort                           ADSWRITE ` ??              step         ` ??                 bExecute            ??       -    Rising edge starts function block execution 	   sFilePath               T_MaxString   ??           e.g. c:\Axis1.scp   tTimeout    ?     ??       >    Maximum time allowed for the execution of the function block       bBusy            ??              bError            ??	              iErrorId           ??
                       ??P  ?   ????           SCOPESETOFFLINE        
   fbAdsWrite                          ADSWRITE    ??                 bExecute            ??                 bBusy            ??              bError            ??              iErrorId           ??                       ??P  ?    ????           SCOPESETONLINE        
   fbAdsWrite                          ADSWRITE    ??                 bExecute            ??                 bBusy            ??              bError            ??              iErrorId           ??                       ??P  ?    ????           SCOPESETRECORDLEN        
   fbAdsWrite                          ADSWRITE    ??                 bExecute            ??           
   fRecordLen                        ??                 bBusy            ??              bError            ??              iErrorId           ??	                       ??P  ?    ????           SCOPEVIEWEXPORT        
   fbAdsWrite                          ADSWRITE    ??                 bExecute            ??           	   sFilePath               T_MaxString   ??              tTimeout    ?     ??                 bBusy            ??              bError            ??	              iErrorId           ??
                       ??P  ?   ????           STRING_TO_CSVFIELD           cbField         ` ??                 in               T_MaxString   ??       !    Input data in PLC string format    bQM            ??	       l    TRUE => Enclose result string in quotation marks, FALSE => Don't enclose result string in quotation marks.       STRING_TO_CSVFIELD               T_MaxString                             ??P  ?    ????           STRING_TO_SYSTEMTIME           b   	                 ?   16#31, 16#36, 16#30, 16#31, 	(* year 1601 *)
								16#2D(*-*), 16#30, 16#31(*01*),	(* month *)
								16#2D(*-*), 16#30, 16#31(*01*),	(* day *)
								16#2D(*-*), 16#30, 16#30(*00*),	(* hour *)
								16#3A(*:*), 16#30, 16#30(*00*),	(* minute *)
								16#3A(*:*), 16#30, 16#30(*00*),	(* second *)
								16#2E(*.*), 16#30, 16#30, 16#30(*000*), (* milliseconds *)
								16#00      1      6      0      1      -      0      1      -      0      1      -      0      0      :      0      0      :      0      0      .      0      0      0           ` ??           null delimiter    ts       *    ( wYear := 1601, wMonth := 1, wDay := 1 )    A               
   TIMESTRUCT ` ??              n         ` ??              bFmt          ` ??              dwYears         ` ??              dwDays         ` ??           	   Index7001                            in               ??       1    Input string, format: '2007-03-05-17:35:09.223'       STRING_TO_SYSTEMTIME                   
   TIMESTRUCT                             ??P  ?    ????           STRING_TO_UINT64           ptr               ` ??              constTen       &     ( dwHighPart := 0, dwLowPart := 10 )    
           T_ULARGE_INTEGER ` ??	                 in               ??                 STRING_TO_UINT64                T_ULARGE_INTEGER                             ??P  ?    ????           SYSTEMTIME_TO_DT           b   	                 ?    16#44, 16#54, 16#23(*DT#*),
										16#31, 16#39, 16#37, 16#30(*1970*),
										16#2D(*-*), 16#30, 16#31(*01*), 16#2D(*-*), 16#30, 16#31(*01*), 16#2D(*-*), 16#30, 16#30(*00*), 16#3A(*:*), 16#30, 16#30(*00*), 16#3A(*:*), 16#30, 16#30(*00*), 16#00      D      T      #      1      9      7      0      -      0      1      -      0      1      -      0      0      :      0      0      :      0      0           ` ??              str             ` ??
              nSeconds         ` ??           	   Index7001                            TIMESTR                   
   TIMESTRUCT   ??                 SYSTEMTIME_TO_DT                                     ??P  ?    ????           SYSTEMTIME_TO_FILETIME           tmp1                T_ULARGE_INTEGER ` ??	              tmp2                T_ULARGE_INTEGER ` ??
              pastDays         ` ??              i         ` ??              
   systemTime                   
   TIMESTRUCT   ??                 SYSTEMTIME_TO_FILETIME             
   T_FILETIME                             ??P  ?    ????           SYSTEMTIME_TO_STRING           b   	                 ?   16#31, 16#36, 16#30, 16#31(*1601*),		(* year *)
										16#2D(*-*), 16#30, 16#31(*01*),				(* month *)
										16#2D(*-*), 16#30, 16#31(*01*),				(* day *)
										16#2D(*-*), 16#30, 16#30(*00*),				(* hour *)
										16#3A(*:*), 16#30, 16#30(*00*),				(* minute *)
										16#3A(*:*), 16#30, 16#30(*00*),				(* second *)
										16#2E(*.*), 16#30, 16#30, 16#30(*000*),		(* milliseconds *)
										16#00      1      6      0      1      -      0      1      -      0      1      -      0      0      :      0      0      :      0      0      .      0      0      0           ` ??           	   Index7001                            in                   
   TIMESTRUCT   ??                 SYSTEMTIME_TO_STRING                                         ??P  ?    ????        	   TC_CONFIG           fbAdsWrtCtl       e    ( PORT := AMSPORT_R3_SYSSERV, ADSSTATE := ADSSTATE_RECONFIG, DEVSTATE := 0, LEN := 0, SRCADDR := 0 )              	   T_AmsPort                                	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    SET            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           TC_CPUUSAGE        	   fbAdsRead       5    ( PORT:= AMSPORT_R0_RTIME, IDXGRP:= 1, IDXOFFS:= 6 )              	   T_AmsPort                          ADSREAD ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??              USAGE           ??          in %            ??P  ?   ????        
   TC_RESTART           fbAdsWrtCtl       b    ( PORT := AMSPORT_R3_SYSSERV, ADSSTATE := ADSSTATE_RESET, DEVSTATE := 0, LEN := 0, SRCADDR := 0 )              	   T_AmsPort                                	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    RESTART            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           TC_STOP           fbAdsWrtCtl       a    ( PORT := AMSPORT_R3_SYSSERV, ADSSTATE := ADSSTATE_STOP, DEVSTATE := 0, LEN := 0, SRCADDR := 0 )              	   T_AmsPort                                	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    STOP            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??                       ??P  ?   ????           TC_SYSLATENCY        	   fbAdsRead       8    ( PORT := AMSPORT_R0_RTIME, IDXGRP := 1, IDXOFFS := 2 )              	   T_AmsPort                          ADSREAD ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??              tmpData   	                      ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??	              ERR            ??
              ERRID           ??              ACTUAL           ??           Actual latency in ?s    MAXIMUM           ??           Maximum latency in ?s             ??P  ?   ????           TIME_TO_OTSTRUCT           tmpMilli         ` ??                 TIN           ??                 TIME_TO_OTSTRUCT                    OTSTRUCT                             ??P  ?    ????           UINT32X32TO64           Tmp1         ` ??	              Tmp2         ` ??
              Tmp3         ` ??              Tmp4         ` ??              DW1         ` ??              DW2         ` ??              DW3         ` ??              DW4         ` ??                 ui32a           ??              ui32b           ??                 UInt32x32To64                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64_TO_LREAL               in                T_ULARGE_INTEGER   ??                 UINT64_TO_LREAL                                                  ??P  ?    ????           UINT64_TO_STRING        	   remainder                T_ULARGE_INTEGER ` ??              constTen       &     ( dwHighPart := 0, dwLowPart := 10 )    
           T_ULARGE_INTEGER ` ??	                 in                T_ULARGE_INTEGER   ??                 UINT64_TO_STRING                                         ??P  ?    ????           UINT64ADD64           bOV          ` ??	                 ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??                 UInt64Add64                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64ADD64EX               augend                T_ULARGE_INTEGER   ??              addend                T_ULARGE_INTEGER   ??                 UInt64Add64Ex                T_ULARGE_INTEGER                       bOV            ??       3    TRUE => arithmetic overflow, FALSE => no overflow         ??P  ?    ????        	   UINT64AND               ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??              	   UInt64And                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64CMP64               ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??	                 UInt64Cmp64                                     ??P  ?    ????           UINT64DIV16EX        	   pDividend    	                            ` ??              pResult    	                            ` ??              rest         ` ??                 dividend                T_ULARGE_INTEGER   ??              divisor           ??                 UInt64Div16Ex                T_ULARGE_INTEGER                    	   remainder                 T_ULARGE_INTEGER  ??                   ??P  ?    ????           UINT64DIV64        	   remainder                T_ULARGE_INTEGER ` ??	                 dividend                T_ULARGE_INTEGER   ??              divisor                T_ULARGE_INTEGER   ??                 UInt64Div64                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64DIV64EX           msBit       /    ( dwHighPart := 16#80000000, 	dwLowPart := 0 )            ?   T_ULARGE_INTEGER ` ??              bitShift         ` ??              cmp         ` ??              in   	                      T_ULARGE_INTEGER         ` ??              out   	                      T_ULARGE_INTEGER         ` ??           
   cbReturned         ` ??           	   Index7001                            dividend                T_ULARGE_INTEGER   ??              divisor                T_ULARGE_INTEGER   ??                 UInt64Div64Ex                T_ULARGE_INTEGER                    	   remainder                 T_ULARGE_INTEGER  ??                   ??P  ?    ????           UINT64ISZERO               ui64                T_ULARGE_INTEGER   ??                 UInt64isZero                                      ??P  ?    ????           UINT64LIMIT               ui64min                T_ULARGE_INTEGER   ??              ui64in                T_ULARGE_INTEGER   ??              ui64max                T_ULARGE_INTEGER   ??                 UInt64Limit                T_ULARGE_INTEGER                             ??P  ?    ????        	   UINT64MAX               ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??              	   UInt64Max                T_ULARGE_INTEGER                             ??P  ?    ????        	   UINT64MIN               ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??              	   UInt64Min                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64MOD64               dividend                T_ULARGE_INTEGER   ??              divisor                T_ULARGE_INTEGER   ??                 UInt64Mod64                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64MUL64           bOV          ` ??	                 multiplicand                T_ULARGE_INTEGER   ??           
   multiplier                T_ULARGE_INTEGER   ??                 UInt64Mul64                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64MUL64EX           bCarry          ` ??           	   bSumCarry          ` ??              n         ` ??              m         ` ??                 multiplicand                T_ULARGE_INTEGER   ??           
   multiplier                T_ULARGE_INTEGER   ??                 UInt64Mul64Ex                T_ULARGE_INTEGER                       bOV            ??       3    TRUE => Arithmetic overflow, FALSE => no overflow         ??P  ?    ????        	   UINT64NOT               ui64                T_ULARGE_INTEGER   ??              	   UInt64Not                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64OR               ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??                 UInt64Or                T_ULARGE_INTEGER                             ??P  ?    ????        	   UINT64ROL           bMSB          ` ??	                 ui64                T_ULARGE_INTEGER   ??              n           ??              	   UInt64Rol                T_ULARGE_INTEGER                             ??P  ?    ????        	   UINT64ROR           bLSB          ` ??	                 ui64                T_ULARGE_INTEGER   ??              n           ??              	   UInt64Ror                T_ULARGE_INTEGER                             ??P  ?    ????        	   UINT64SHL               ui64                T_ULARGE_INTEGER   ??              n           ??              	   UInt64Shl                T_ULARGE_INTEGER                             ??P  ?    ????        	   UINT64SHR               ui64                T_ULARGE_INTEGER   ??              n           ??              	   UInt64Shr                T_ULARGE_INTEGER                             ??P  ?    ????           UINT64SUB64               ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??                 UInt64Sub64                T_ULARGE_INTEGER                             ??P  ?    ????        	   UINT64XOR               ui64a                T_ULARGE_INTEGER   ??              ui64b                T_ULARGE_INTEGER   ??              	   UInt64Xor                T_ULARGE_INTEGER                             ??P  ?    ????           ULARGE_INTEGER            
   dwHighPart           ??           	   dwLowPart           ??                 ULARGE_INTEGER                T_ULARGE_INTEGER                             ??P  ?    ????           ULARGE_TO_LARGE               in                T_ULARGE_INTEGER   ??                 ULARGE_TO_LARGE                T_LARGE_INTEGER                             ??P  ?    ????           WORD_TO_FIX16               in           ??           16 bit fixed point number    n                           ??           number of fractional bits       WORD_TO_FIX16                 T_FIX16                             ??P  ?    ????           WRITEPERSISTENTDATA           fbAdsWrtCtl       H    ( ADSSTATE := ADSSTATE_SAVECFG, DEVSTATE := 0, LEN := 0, SRCADDR := 0 )                              	   ADSWRTCTL ` ??                 NETID            
   T_AmsNetId   ??       &    TwinCAT network address (ams net id)    PORT           ??       l    Contains the ADS port number of the PLC run-time system whose persistent data is to be stored (801, 811...)   START            ??       6    Rising edge on this input activates the fb execution    TMOUT    ?     ??           Max fb execution time       BUSY            ??
              ERR            ??              ERRID           ??                       ??P  ?   ????    o   C:\TWINCAT\PLC\LIB\TcBase.lib @                                                                                          FW_ADSCLEAREVENTS           STAMP_I            ??              ACCESSCNT_I            ??              BUSY_I             ??              ERR_I             ??              ERRID_I            ??           
   READ_SAV_I             ??              TICKSTART_I            ??                 sNetId               ??              bClear            ??              nMode           ??              tTimeout           ??                 bBusy            ??	              bError            ??
              nErrId           ??                       ??J  ?   ????           FW_ADSLOGDINT            	   nCtrlMask           ??              sMsgFmt               ??              nArg           ??                 FW_AdsLogDINT                                     ??J  ?   ????           FW_ADSLOGEVENT        
   STAMPREQ_I            ??           
   STAMPRES_I            ??           
   STAMPSIG_I            ??           
   STAMPCON_I            ??              ACCESSCNT_I            ??           	   AMSADDR_I   	                         ??              EVENT_I                      
   FW_TcEvent    ??              pTCEVENTSTREAM_I            ??              CBEVENTSTREAM_I            ??              nSTATE_I            ??              nSTATEREQ_I            ??              nSTATERES_I            ??              nSTATESIG_I            ??               nSTATECON_I            ??!              ERR_I             ??"              ERRID_I            ??#              bEVENT_SAV_I             ??$              bEVENTQUIT_SAV_I             ??%              TICKSTART_I            ??&           	      sNetId               ??              nPort           ??              bEvent            ??           
   bEventQuit            ??              stEventConfigData                      
   FW_TcEvent   ??              pEventDataAddress           ??       	    pointer    cbEventDataLength           ??	           
   bFbCleanup            ??
              tTimeout           ??                 nEventState           ??              bError            ??              nErrId           ??              bQuit            ??                       ??J  ?   ????           FW_ADSLOGLREAL            	   nCtrlMask           ??              sMsgFmt               ??              fArg                        ??                 FW_AdsLogLREAL                                     ??J  ?   ????           FW_ADSLOGSTR            	   nCtrlMask           ??              sMsgFmt               ??              sArg               ??                 FW_AdsLogSTR                                     ??J  ?   ????           FW_ADSRDWRT           STAMP_I            ??              ACCESSCNT_I            ??              BUSY_I             ??              ERR_I             ??              ERRID_I            ??              WRTRD_SAV_I             ??              PDESTADDR_I            ??              TICKSTART_I            ??           
      sNetId               ??              nPort           ??              nIdxGrp           ??              nIdxOffs           ??           
   cbWriteLen           ??           	   cbReadLen           ??           
   pWriteBuff           ??	           	   pReadBuff           ??
              bExecute            ??              tTimeout           ??                 bBusy            ??              bError            ??              nErrId           ??              cbRead           ??           count of bytes actually read             ??J  ?   ????           FW_ADSRDWRTIND           CLEAR_I             ??                 bClear            ??           	      bValid            ??              sNetId               ??              nPort           ??           	   nInvokeId           ??	              nIdxGrp           ??
              nIdxOffs           ??           	   cbReadLen           ??           
   cbWriteLen           ??           
   pWriteBuff           ??                       ??J  ?   ????           FW_ADSRDWRTRES        	   RESPOND_I             ??                 sNetId               ??              nPort           ??           	   nInvokeId           ??              nErrId           ??           	   cbReadLen           ??           	   pReadBuff           ??              bRespond            ??	                           ??J  ?   ????        
   FW_ADSREAD           STAMP_I            ??              ACCESSCNT_I            ??              BUSY_I             ??              ERR_I             ??              ERRID_I            ??           
   READ_SAV_I             ??              TICKSTART_I            ??                 sNetId               ??              nPort           ??              nIdxGrp           ??              nIdxOffs           ??           	   cbReadLen           ??           	   pReadBuff           ??              bExecute            ??	              tTimeout           ??
                 bBusy            ??              bError            ??              nErrId           ??              cbRead           ??           count of bytes actually read             ??J  ?   ????           FW_ADSREADDEVICEINFO           STAMP_I            ??              ACCESSCNT_I            ??              BUSY_I             ??              ERR_I             ??              ERRID_I            ??              RDINFO_SAV_I             ??              TICKSTART_I            ??                 sNetId               ??              nPort           ??              bExecute            ??              tTimeout           ??                 bBusy            ??	              bError            ??
              nErrId           ??              sDevName               ??              nDevVersion           ??                       ??J  ?   ????           FW_ADSREADIND           CLEAR_I             ??                 bClear            ??                 bValid            ??              sNetId               ??              nPort           ??           	   nInvokeId           ??	              nIdxGrp           ??
              nIdxOffs           ??           	   cbReadLen           ??                       ??J  ?   ????           FW_ADSREADRES        	   RESPOND_I             ??                 sNetId               ??              nPort           ??           	   nInvokeId           ??              nErrId           ??           	   cbReadLen           ??           	   pReadBuff           ??              bRespond            ??	                           ??J  ?   ????           FW_ADSREADSTATE           STAMP_I            ??              ACCESSCNT_I            ??              BUSY_I             ??              ERR_I             ??              ERRID_I            ??              RDSTATE_SAV_I             ??              TICKSTART_I            ??                 sNetId               ??              nPort           ??              bExecute            ??              tTimeout           ??                 bBusy            ??	              bError            ??
              nErrId           ??           	   nAdsState           ??           	   nDevState           ??                       ??J  ?   ????           FW_ADSWRITE           STAMP_I            ??              ACCESSCNT_I            ??              BUSY_I             ??              ERR_I             ??              ERRID_I            ??              WRITE_SAV_I             ??              TICKSTART_I            ??                 sNetId               ??              nPort           ??              nIdxGrp           ??              nIdxOffs           ??           
   cbWriteLen           ??           
   pWriteBuff           ??              bExecute            ??	              tTimeout           ??
                 bBusy            ??              bError            ??              nErrId           ??                       ??J  ?   ????           FW_ADSWRITECONTROL           STAMP_I            ??              ACCESSCNT_I            ??              BUSY_I             ??              ERR_I             ??              ERRID_I            ??              WRITE_SAV_I             ??              TICKSTART_I            ??                 sNetId               ??              nPort           ??           	   nAdsState           ??           	   nDevState           ??           
   cbWriteLen           ??           
   pWriteBuff           ??              bExecute            ??	              tTimeout           ??
                 bBusy            ??              bError            ??              nErrId           ??                       ??J  ?   ????           FW_ADSWRITEIND           CLEAR_I             ??                 bClear            ??                 bValid            ??              sNetId               ??              nPort           ??           	   nInvokeId           ??	              nIdxGrp           ??
              nIdxOffs           ??           
   cbWriteLen           ??           
   pWriteBuff           ??                       ??J  ?   ????           FW_ADSWRITERES        	   RESPOND_I             ??                 sNetId               ??              nPort           ??           	   nInvokeId           ??              nErrId           ??              bRespond            ??                           ??J  ?   ????           FW_DRAND           FirstCall_i             ??	           
   HoldRand_i            ??
              R250_Buffer_i   	  ?                        ??           
   R250_Index            ??                 nSeed           ??                 fRndNum                        ??                       ??J  ?   ????           FW_GETCPUACCOUNT                   dwCpuAccount           ??                       ??J  ?   ????           FW_GETCPUCOUNTER                
   dwCpuCntLo           ??           
   dwCpuCntHi           ??                       ??J  ?   ????           FW_GETCURTASKINDEX                   nIndex           ??                       ??J  ?   ????           FW_GETSYSTEMTIME                   dwTimeLo           ??              dwTimeHi           ??                       ??J  ?   ????           FW_GETVERSIONTCBASE               nVersionElement           ??                 FW_GetVersionTcBase                                     ??J  ?   ????           FW_LPTSIGNAL            	   nPortAddr           ??              nPinNo           ??              bOnOff            ??	                 FW_LptSignal                                      ??J  ?   ????        	   FW_MEMCMP               pBuf1           ??           First buffer    pBuf2           ??           Second buffer    cbLen           ??           Number of characters    	   FW_MemCmp                                     ??J  ?   ????        	   FW_MEMCPY               pDest           ??           New buffer    pSrc           ??           Buffer to copy from    cbLen           ??           Number of characters to copy    	   FW_MemCpy                                     ??J  ?   ????        
   FW_MEMMOVE               pDest           ??           New buffer    pSrc           ??           Buffer to copy from    cbLen           ??           Number of characters to copy    
   FW_MemMove                                     ??J  ?   ????        	   FW_MEMSET               pDest           ??           Pointer to destination 	   nFillByte           ??           Character to set    cbLen           ??           Number of characters    	   FW_MemSet                                     ??J  ?   ????           FW_PORTREAD            	   nPortAddr           ??           	   eNoOfByte               FW_NoOfByte   ??                 FW_PortRead                                     ??J  ?   ????           FW_PORTWRITE            	   nPortAddr           ??           	   eNoOfByte               FW_NoOfByte   ??              nValue           ??                 FW_PortWrite                                      ??J  ?   ????    q   C:\TWINCAT\PLC\LIB\TcSystem.lib @                                                                                Q          ADSCLEAREVENTS           fbAdsClearEvents                            FW_AdsClearEvents ` ??                 NetID            
   T_AmsNetId   ??              bClear            ??              iMode           ??              tTimeout    ?     ??                 bBusy            ??	              bErr            ??
              iErrId           ??                       8'?R  ?   ????        
   ADSLOGDINT               msgCtrlMask           ??           	   msgFmtStr               T_MaxString   ??              dintArg           ??              
   ADSLOGDINT                                     8'?R  ?    ????           ADSLOGEVENT           fbAdsLogEvent                                               FW_AdsLogEvent ` ??           	      NETID            
   T_AmsNetId   ??              PORT           ??              Event            ??           	   EventQuit            ??              EventConfigData               TcEvent   ??              EventDataAddress           ??       	    pointer    EventDataLength           ??	           	   FbCleanup            ??
              TMOUT    ?     ??              
   EventState           ??              Err            ??              ErrId           ??              Quit            ??                       8'?R  ?   ????           ADSLOGLREAL               msgCtrlMask           ??           	   msgFmtStr               T_MaxString   ??              lrealArg                        ??                 ADSLOGLREAL                                     8'?R  ?    ????        	   ADSLOGSTR               msgCtrlMask           ??           	   msgFmtStr               T_MaxString   ??              strArg               T_MaxString   ??              	   ADSLOGSTR                                     8'?R  ?    ????           ADSRDDEVINFO           fbAdsReadDeviceInfo                              FW_AdsReadDeviceInfo    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              RDINFO            ??              TMOUT    ?     ??                 BUSY            ??	              ERR            ??
              ERRID           ??              DEVNAME               ??              DEVVER           ??                       8'?R  ?   ????        
   ADSRDSTATE           fbAdsReadState                              FW_AdsReadState    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              RDSTATE            ??              TMOUT    ?     ??                 BUSY            ??	              ERR            ??
              ERRID           ??              ADSSTATE           ??              DEVSTATE           ??                       8'?R  ?   ????           ADSRDWRT        
   fbAdsRdWrt                                    FW_AdsRdWrt    ??           
      NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              WRITELEN           ??              READLEN           ??              SRCADDR           ??	              DESTADDR           ??
              WRTRD            ??              TMOUT    ?     ??                 BUSY            ??              ERR            ??              ERRID           ??                       8'?R  ?   ????        
   ADSRDWRTEX        
   fbAdsRdWrt                                    FW_AdsRdWrt    ??           
      NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              WRITELEN           ??              READLEN           ??              SRCADDR           ??	              DESTADDR           ??
              WRTRD            ??              TMOUT    ?     ??                 BUSY            ??              ERR            ??              ERRID           ??              COUNT_R           ??           count of bytes actually read             8'?R  ?   ????           ADSRDWRTIND           fbAdsRdWrtInd                         FW_AdsRdWrtInd    ??                 CLEAR            ??           	      VALID            ??              NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              INVOKEID           ??	              IDXGRP           ??
              IDXOFFS           ??              RDLENGTH           ??           	   WRTLENGTH           ??              DATAADDR           ??                       8'?R  ?    ????           ADSRDWRTRES           fbAdsRdWrtRes                      FW_AdsRdWrtRes    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              INVOKEID           ??              RESULT           ??              LEN           ??              DATAADDR           ??              RESPOND            ??	                           8'?R  ?    ????           ADSREAD        	   fbAdsRead                              
   FW_AdsRead    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              LEN           ??              DESTADDR           ??              READ            ??	              TMOUT    ?     ??
                 BUSY            ??              ERR            ??              ERRID           ??                       8'?R  ?   ????        	   ADSREADEX        	   fbAdsRead                              
   FW_AdsRead    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              LEN           ??              DESTADDR           ??              READ            ??	              TMOUT    ?     ??
                 BUSY            ??              ERR            ??              ERRID           ??              COUNT_R           ??           count of bytes actually read             8'?R  ?   ????        
   ADSREADIND           fbAdsReadInd        	               FW_AdsReadInd    ??                 CLEAR            ??                 VALID            ??              NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              INVOKEID           ??	              IDXGRP           ??
              IDXOFFS           ??              LENGTH           ??                       8'?R  ?    ????        
   ADSREADRES           fbAdsReadRes                      FW_AdsReadRes    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              INVOKEID           ??              RESULT           ??              LEN           ??              DATAADDR           ??              RESPOND            ??	                           8'?R  ?    ????           ADSWRITE        
   fbAdsWrite                                FW_AdsWrite    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              LEN           ??              SRCADDR           ??              WRITE            ??	              TMOUT    ?     ??
                 BUSY            ??              ERR            ??              ERRID           ??                       8'?R  ?   ????           ADSWRITEIND           fbAdsWriteInd        
                FW_AdsWriteInd    ??                 CLEAR            ??                 VALID            ??              NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              INVOKEID           ??	              IDXGRP           ??
              IDXOFFS           ??              LENGTH           ??              DATAADDR           ??                       8'?R  ?    ????           ADSWRITERES           fbAdsWriteRes                    FW_AdsWriteRes    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              INVOKEID           ??              RESULT           ??              RESPOND            ??                           8'?R  ?    ????        	   ADSWRTCTL           fbAdsWriteControl                                FW_AdsWriteControl    ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              ADSSTATE           ??              DEVSTATE           ??              LEN           ??              SRCADDR           ??              WRITE            ??	              TMOUT    ?     ??
                 BUSY            ??              ERR            ??              ERRID           ??                       8'?R  ?   ????           ANALYZEEXPRESSION               InputExp            ??           	   DoAnalyze            ??              	   ExpResult            ??           	   OutString               ??                       8'?R  ?    ????           ANALYZEEXPRESSIONCOMBINED           Index            ??                 InputExp            ??           	   DoAnalyze            ??              	   ExpResult            ??              OutTable   	                        ExpressionResult           ??           	   OutString               ??	                       8'?R  ?    ????           ANALYZEEXPRESSIONTABLE           Index            ??                 InputExp            ??           	   DoAnalyze            ??              	   ExpResult            ??              OutTable   	                        ExpressionResult           ??                       8'?R  ?    ????           APPENDERRORSTRING               strOld               ??              strNew               ??                 AppendErrorString                                         8'?R  ?    ????           BAVERSION_TO_DWORD               nVersion         ` ??           	   nRevision         ` ??	              nBuild         ` ??
                 BAVERSION_TO_DWORD                                     8'?R  ?    ????        
   CLEARBIT32           dwConst        ` ??                 inVal32           ??              bitNo           ??              
   CLEARBIT32                                     8'?R  ?    ????        	   CSETBIT32           dwConst        ` ??	                 inVal32           ??              bitNo           ??              bitVal            ??       &    value to which the bit should be set    	   CSETBIT32                                     8'?R  ?    ????           DRAND           fbDRand                    FW_DRand ` ??
                 Seed           ??                 Num                        ??                       8'?R  ?    ????           F_COMPAREFWVERSION           soll         ` ??              ist         ` ??                 major         ` ??           requiered major version    minor         ` ??	           requiered minor version    revision         ` ??
       )    requiered revision/service pack version    patch         ` ??       0    required patch version (reserved, default = 0 )      F_CompareFwVersion                                      8'?R  ?    ????           F_CREATEAMSNETID           idx         ` ??                 nIds               T_AmsNetIdArr   ??           Ams Net ID as array of bytes.       F_CreateAmsNetId            
   T_AmsNetId                             8'?R  ?    ????           F_CREATEIPV4ADDR           idx         ` ??                 nIds               T_IPv4AddrArr   ??       <    Internet Protocol dotted address (ipv4) as array of bytes.       F_CreateIPv4Addr            
   T_IPv4Addr                             8'?R  ?    ????           F_GETSTRUCTMEMBERALIGNMENT           tmp                ST_StructMemberAlignmentProbe ` ??                     F_GetStructMemberAlignment                                     8'?R  ?    ????           F_GETVERSIONTCSYSTEM               nVersionElement           ??                 F_GetVersionTcSystem                                     8'?R  ?    ????           F_IOPORTREAD               nAddr           ??           Port address    eSize               E_IOAccessSize   ??           Number of bytes to read       F_IOPortRead                                     8'?R  ?    ????           F_IOPORTWRITE               nAddr           ??           Port address    eSize               E_IOAccessSize   ??           Number of bytes to write    nValue           ??           Value to write       F_IOPortWrite                                      8'?R  ?    ????           F_SCANAMSNETIDS           pNetID               ` ??              b               T_AmsNetIdArr ` ??              w         ` ??	              id         ` ??
           	   Index7001                            sNetID            
   T_AmsNetID   ??       :    String containing the Ams Net ID. E.g. '127.16.17.3.1.1'       F_ScanAmsNetIds               T_AmsNetIdArr                             8'?R  ?    ????           F_SCANIPV4ADDRIDS           b               T_AmsNetIdArr ` ??           	   Index7001                            sIPv4            
   T_IPv4Addr   ??       M    String containing the Internet Protocol dotted address. E.g. '172.16.7.199'       F_ScanIPv4AddrIds               T_IPv4AddrArr                             8'?R  ?    ????           F_SPLITPATHNAME           pPath               ` ??              pSlash               ` ??              pDot               ` ??              p               ` ??              length         ` ??              	   sPathName               T_MaxString   ??                 F_SplitPathName                                sDrive               ??              sDir                T_MaxString  ??           	   sFileName                T_MaxString  ??              sExt                T_MaxString  ??	                   8'?R  ?    ????           F_TOASC           pChar               ` ??                 str    Q       Q    ??                 F_ToASC                                     8'?R  ?    ????           F_TOCHR           pChar    	                            ` ??                 c           ??                 F_ToCHR    Q       Q                              8'?R  ?    ????           FB_ADSREADWRITELIST           para                ST_AdsRdWrtListPara ` ??           	   fbTrigger                 R_TRIG ` ??              nState         ` ??              fbCall       ?    ( 	sNetID := '', nPort := 16#1234,
									nIdxGrp := GENERIC_FB_GRP_ADS,
									nIdxOffs := GENERIC_FB_ADS_RDWRTLIST,
									bExecute := FALSE,  ACCESSCNT_I := 16#0000BEC1,
									tTimeout := DEFAULT_ADS_TIMEOUT )     ??                 4                     ?         FW_AdsRdWrt ` ??           
      sNetId           ''    
   T_AmsNetID ` ??              nPort           0    	   T_AmsPort ` ??              nIdxGrp         ` ??              nIdxOffs         ` ??              cbWriteList         ` ??	           Byte size of list array 
   pWriteList                   ST_AdsReadWriteListEntry      ` ??
       !    Pointer to the first list entry 	   cbReadLen         ` ??           	   pReadBuff           0       PVOID ` ??              bExecute          ` ??              tTimeout    ?   ` ??                 bBusy          ` ??              bError          ` ??              nErrID         ` ??              cbRead         ` ??                       8'?R  ?   ????           FB_BADEVICEIOCONTROL           fbRW       O    ( PORT := AMSPORT_R3_SYSSERV, IDXGRP := SYSTEMSERVICE_BADEVAPI, IDXOFFS := 0 )              	   T_AmsPort         L                 
   ADSRDWRTEX ` ??              req                ST_AdsBaDevApiReq ` ??              state         ` ??              rtrig                 R_TRIG ` ??                 sNetID           ''    
   T_AmsNetID ` ??           Ams net id    affinity           ( lower :=0, higher := 0 )                T_U64KAFFINITY ` ??       )    Affinity mask (default  = 0 = not used) 	   nModifier         ` ??       +    Optional command modifier (0 == not used)    nIdxGrp         ` ??           Api function group    nIdxOffs         ` ??           Api function offset 
   cbWriteLen         ` ??	           Input data byte size 	   cbReadLen         ` ??
           Output data byte size 
   pWriteBuff         ` ??           Pointer to input data buffer 	   pReadBuff         ` ??           Pointer to output data buffer    bExecute          ` ??       &    Rising edge starts command execution    tTimeout    ?   ` ??                 bBusy          ` ??              bError          ` ??              nErrID         ` ??              cbRead         ` ??           Count of bytes actually read             8'?R  ?   ????           FB_BAGENGETVERSION           fbCtrl       ?    ( nModifier := 0, affinity := ( lower := 0, higher := 0 ), nIdxGrp := BADEVAPIIGRP_GENERAL, nIdxOffs := BADEVAPIIOFFS_GENERAL_VERSION )                ( lower :=0, higher := 0 )                T_U64KAFFINITY                                             FB_BaDeviceIoControl ` ??              rtrig                 R_TRIG ` ??              state         ` ??              rsp         ` ??                 sNetID           ''    
   T_AmsNetID ` ??           ams net id    bExecute          ` ??       &    rising edge starts command execution    tTimeout    ?   ` ??                 bBusy          ` ??	              bError          ` ??
              nErrID         ` ??              nVersion         ` ??           	   nRevision         ` ??              nBuild         ` ??                       8'?R  ?   ????           FB_CREATEDIR        
   fbAdsRdWrt       ]    ( nPort:= AMSPORT_R3_SYSSERV, nIdxGrp:= SYSTEMSERVICE_MKDIR, cbReadLen := 0, pReadBuff:= 0 )             '   ?                         FW_AdsRdWrt ` ??                 sNetId            
   T_AmsNetId   ??           ams net id 	   sPathName               T_MaxString   ??           max directory length = 255    ePath           PATH_GENERIC    
   E_OpenPath   ??       +    Default: Create directory at generic path    bExecute            ??       %    rising edge start command execution    tTimeout    ?     ??                 bBusy            ??
              bError            ??              nErrId           ??                       8'?R  ?   ????           FB_EOF        
   fbAdsRdWrt       `    (nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FEOF, cbWriteLen := 0, pWriteBuff := 0 )             '   ?                         FW_AdsRdWrt ` ??              iEOF         ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??           file handle    bExecute            ??           control input    tTimeout    ?     ??                 bBusy            ??	              bError            ??
              nErrId           ??              bEOF            ??                       8'?R  ?   ????           FB_FILECLOSE        
   fbAdsRdWrt       ?    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FCLOSE, cbWriteLen := 0,	cbReadLen := 0,	pWriteBuff := 0, pReadBuff := 0 )             '   y                                 FW_AdsRdWrt ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??       %    file handle obtained through 'open'    bExecute            ??           close control input    tTimeout    ?     ??                 bBusy            ??	              bError            ??
              nErrId           ??                       8'?R  ?   ????           FB_FILEDELETE        
   fbAdsRdWrt       a    (nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FDELETE, cbReadLen := 0, pReadBuff := 0 )             '   ?                         FW_AdsRdWrt ` ??              tmpOpenMode         ` ??                 sNetId            
   T_AmsNetId   ??           ams net id 	   sPathName               T_MaxString   ??           file path and name    ePath           PATH_GENERIC    
   E_OpenPath   ??           Default: Open generic file    bExecute            ??           open control input    tTimeout    ?     ??                 bBusy            ??
              bError            ??              nErrId           ??                       8'?R  ?   ????           FB_FILEGETS        
   fbAdsRdWrt       b    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FGETS, cbWriteLen := 0, pWriteBuff := 0 )             '   ~                         FW_AdsRdWrt ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??           file handle    bExecute            ??           control input    tTimeout    ?     ??                 bBusy            ??	              bError            ??
              nErrId           ??              sLine               T_MaxString   ??              bEOF            ??                       8'?R  ?   ????           FB_FILEOPEN        
   fbAdsRdWrt       @    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FOPEN )             '   x                 FW_AdsRdWrt ` ??              tmpOpenMode         ` ??              tmpHndl         ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetId            
   T_AmsNetId   ??           ams net id 	   sPathName               T_MaxString   ??           max filename length = 255    nMode           ??           open mode flags    ePath           PATH_GENERIC    
   E_OpenPath   ??           Default: Open generic file    bExecute            ??           open control input    tTimeout    ?     ??                 bBusy            ??              bError            ??              nErrId           ??              hFile           ??           file handle             8'?R  ?   ????           FB_FILEPUTS        
   fbAdsRdWrt       `    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FPUTS, cbReadLen := 0, pReadBuff := 0 )             '                            FW_AdsRdWrt ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??           file handle    sLine               T_MaxString   ??           string to write    bExecute            ??           control input    tTimeout    ?     ??                 bBusy            ??
              bError            ??              nErrId           ??                       8'?R  ?   ????           FB_FILEREAD        
   fbAdsRdWrt       b    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FREAD, cbWriteLen := 0, pWriteBuff := 0 )             '   z                         FW_AdsRdWrt ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??           file handle 	   pReadBuff           ??           buffer address for read 	   cbReadLen           ??           count of bytes for read    bExecute            ??           read control input    tTimeout    ?     ??                 bBusy            ??              bError            ??              nErrId           ??              cbRead           ??           count of bytes actually read    bEOF            ??                       8'?R  ?   ????           FB_FILERENAME        
   fbAdsRdWrt       b    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FRENAME, cbReadLen := 0, pReadBuff := 0 )             '   ?                         FW_AdsRdWrt ` ??              tmpOpenMode         ` ??           
   sBothNames   	                    T_MaxString         ` ??           = SIZEOF( T_MaxString ) * 2    nOldLen         ` ??              nNewLen         ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    sOldName               T_MaxString   ??           max filename length = 255    sNewName               T_MaxString   ??           max filename length = 255    ePath           PATH_GENERIC    
   E_OpenPath   ??           Default: generic file path   bExecute            ??           open control input    tTimeout    ?     ??                 bBusy            ??              bError            ??              nErrId           ??                       8'?R  ?   ????           FB_FILESEEK        
   fbAdsRdWrt       `    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FSEEK, cbReadLen := 0, pReadBuff := 0 )             '   |                         FW_AdsRdWrt ` ??           
   tmpSeekPos   	                       ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??	           file handle    nSeekPos           ??
           new seek pointer position    eOrigin       	    SEEK_SET       E_SeekOrigin   ??              bExecute            ??           seek control input    tTimeout    ?     ??                 bBusy            ??              bError            ??              nErrId           ??                       8'?R  ?   ????           FB_FILETELL        
   fbAdsRdWrt       b    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FTELL, cbWriteLen := 0, pWriteBuff := 0 )             '   }                         FW_AdsRdWrt ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??           file handle    bExecute            ??           control input    tTimeout    ?     ??                 bBusy            ??	              bError            ??
              nErrId           ??              nSeekPos           ??          	On error, nSEEKPOS returns -1             8'?R  ?   ????           FB_FILEWRITE        
   fbAdsRdWrt       A    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_FWRITE )             '   {                 FW_AdsRdWrt ` ??           	   fbTrigger                 R_TRIG ` ??              state         ` ??                 sNetId            
   T_AmsNetId   ??           ams net id    hFile           ??           file handle 
   pWriteBuff           ??           buffer address for write 
   cbWriteLen           ??           count of bytes for write    bExecute            ??           write control input    tTimeout    ?     ??                 bBusy            ??              bError            ??              nErrId           ??              cbWrite           ??       !    count of bytes actually written             8'?R  ?   ????           FB_PCWATCHDOG           bRetVal          ` ??              iTime         ` ??              iIdx         ` ??              iPortArr   	                 >    16#2E, 16#2E, 16#2E, 16#2F, 16#2E, 16#2F, 16#2E, 16#2F, 16#2E	      .      .      .      /      .      /      .      /      .    ` ??              iArrEn   	                 >    16#87, 16#87, 16#07, 16#08, 16#F6, 16#05, 16#30, 16#01, 16#AA	      ?      ?                  ?            0            ?    ` ??              iArrDis   	                 >    16#87, 16#87, 16#07, 16#08, 16#F6, 16#00, 16#30, 16#00, 16#AA	      ?      ?                  ?             0             ?    ` ??                 tTimeOut           ??       ;    Watchdog TimeOut Time 1s..255s, disabled if tTimeOut < 1s    bEnable            ??           Enable / Disable Watchdog       bEnabled            ??       2    TRUE: Watchdog Enabled; FALSE: Watchdog Disabled    bBusy            ??           FB still busy    bError            ??	           FB has error     nErrId           ??
           FB error ID               8'?R  ?    ????           FB_REMOVEDIR        
   fbAdsRdWrt       `    ( nPort := AMSPORT_R3_SYSSERV, nIdxGrp := SYSTEMSERVICE_RMDIR, cbReadLen := 0, pReadBuff := 0 )             '   ?                         FW_AdsRdWrt ` ??                 sNetId            
   T_AmsNetId   ??           ams net id 	   sPathName               T_MaxString   ??           max filename length = 255    ePath           PATH_GENERIC    
   E_OpenPath   ??       +    Default: Delete directory at generic path    bExecute            ??       &    rising edge starts command execution    tTimeout    ?     ??                 bBusy            ??
              bError            ??              nErrId           ??                       8'?R  ?   ????           FB_SIMPLEADSLOGEVENT           fbEvent       9    ( NETID := '', PORT := AMSPORT_EVENTLOG, TMOUT:= t#15s )             
   T_AmsNetId         n          ?:         ADSLOGEVENT ` ??              cfgEvent               TcEvent ` ??              bInit         ` ??                 SourceID           ??              EventID           ??           	   bSetEvent           ??              bQuit            ??                 ErrId           ??	              Error            ??
                       8'?R  ?   ????        	   FILECLOSE        
   fbAdsWrite                                FW_AdsWrite    ??                 NETID            
   T_AmsNetId   ??           ams net id    HFILE           ??       )    file handle obtained through 'FILEOPEN'    CLOSE            ??           close control input    TMOUT    ?     ??                 BUSY            ??	              ERR            ??
              ERRID           ??                       8'?R  ?   ????           FILEOPEN        
   fbAdsWrite                                FW_AdsWrite    ??           
   RisingEdge                 R_TRIG    ??              FallingEdge                 F_TRIG    ??                 NETID            
   T_AmsNetId   ??           ams net id 	   FPATHNAME               T_MaxString   ??       #    default max filename length = 255    OPENMODE           ??           open mode flags    OPEN            ??           open control input    TMOUT    ?     ??                 BUSY            ??
              ERR            ??              ERRID           ??              HFILE           ??           file handle             8'?R  ?   ????           FILEREAD        	   fbAdsRead                              
   FW_AdsRead    ??                 NETID            
   T_AmsNetId   ??           ams net id    HFILE           ??           file handle    BUFADDR           ??           buffer address for read    COUNT           ??           count of bytes for read    READ            ??           read control input    TMOUT    ?     ??                 BUSY            ??              ERR            ??              ERRID           ??              COUNT_R           ??           count of bytes actually read             8'?R  ?   ????           FILESEEK        
   fbAdsWrite                                FW_AdsWrite    ??                 NETID            
   T_AmsNetId   ??           ams net id    HFILE           ??           file handle    SEEKPOS           ??           new seek pointer position    SEEK            ??           seek control input    TMOUT    ?     ??                 BUSY            ??
              ERR            ??              ERRID           ??                       8'?R  ?   ????        	   FILEWRITE        
   fbAdsWrite                                FW_AdsWrite    ??           
   RisingEdge                 R_TRIG    ??              FallingEdge                 F_TRIG    ??              tmpCount            ??                 NETID            
   T_AmsNetId   ??           ams net id    HFILE           ??           file handle    BUFADDR           ??           buffer address for write    COUNT           ??           count of bytes for write    WRITE            ??           write control input    TMOUT    ?     ??                 BUSY            ??              ERR            ??              ERRID           ??              COUNT_W           ??       !    count of bytes actually written             8'?R  ?   ????           FW_CALLGENERICFB           fbCall       w    ( 	sNetID := '', nPort := 16#1234,
								bExecute := FALSE, tTimeout := T#0s,
								ACCESSCNT_I := 16#0000BEC1 )     ??                 4                          FW_AdsRdWrt ` ??                 funGrp         ` ??       #    Function block group (identifier)    funNum         ` ??       $    Function block number (identifier)    pWrite         ` ??       +    Byte length of output parameter structure    cbWrite         ` ??       *    Byte length of input parameter structure    pRead         ` ??	           Points ot output buffer    cbRead         ` ??
           Points to input buffer       nErrID         ` ??           0 => no error, <> 0 => error
   cbReturned         ` ??       ,    Number of successfully returned data bytes             8'?R  ?    ????           FW_CALLGENERICFUN           fbCall       y    ( 	sNetID := '', nPort := 16#1234,
									bExecute := FALSE, tTimeout := T#0s,
									ACCESSCNT_I := 16#0000BEC2 )     ¾                 4                          FW_AdsRdWrt ` ??           don't use it!        funGrp         ` ??           Function group (identifier)    funNum         ` ??       $    Function block number (identifier)    pWrite         ` ??       +    Byte length of output parameter structure    cbWrite         ` ??	       *    Byte length of input parameter structure    pRead         ` ??
           Points ot output buffer    cbRead         ` ??           Points to input buffer    pcbReturned               ` ??       ,    Number of successfully returned data bytes       FW_CallGenericFun                                     8'?R  ?    ????           GETBIT32           dwConst        ` ??                 inVal32           ??              bitNo           ??                 GETBIT32                                      8'?R  ?    ????           GETCPUACCOUNT           fbGetCpuAccount               FW_GetCpuAccount ` ??                     cpuAccountDW           ??                       8'?R  ?    ????           GETCPUCOUNTER           fbGetCpuCounter                FW_GetCpuCounter ` ??                  
   cpuCntLoDW           ??           
   cpuCntHiDW           ??                       8'?R  ?    ????           GETCURTASKINDEX           fbGetCurTaskIndex               FW_GetCurTaskIndex ` ??                     index           ??           Task index [1..4]             8'?R  ?    ????           GETSYSTEMTIME           fbGetSystemTime                FW_GetSystemTime ` ??                     timeLoDW           ??              timeHiDW           ??                       8'?R  ?    ????           GETTASKTIME           out   	                       ` ??	           
   cbReturned         ` ??
                     timeLoDW           ??              timeHiDW           ??                       8'?R  ?    ????        	   LPTSIGNAL               PortAddr           ??              PinNo           ??              OnOff            ??	              	   LPTSIGNAL                                      8'?R  ?    ????           MEMCMP               pBuf1           ??           First buffer    pBuf2           ??           Second buffer    n           ??           Number of characters       MEMCMP                                     8'?R  ?    ????           MEMCPY               destAddr           ??           New buffer    srcAddr           ??           Buffer to copy from    n           ??           Number of characters to copy       MEMCPY                                     8'?R  ?    ????           MEMMOVE               destAddr           ??           New buffer    srcAddr           ??           Buffer to copy from    n           ??           Number of characters to copy       MEMMOVE                                     8'?R  ?    ????           MEMSET               destAddr           ??           Pointer to destination    fillByte           ??           Character to set    n           ??           Number of characters       MEMSET                                     8'?R  ?    ????           ROL32               inVal32           ??              n           ??                 ROL32                                     8'?R  ?    ????           ROR32               inVal32           ??              n           ??                 ROR32                                     8'?R  ?    ????           SETBIT32           dwConst        ` ??                 inVal32           ??              bitNo           ??                 SETBIT32                                     8'?R  ?    ????           SFCACTIONCONTROL     
      S_FF                 RS    ??              L_TMR                    TON    ??              D_TMR                    TON    ??              P_TRIG                 R_TRIG    ??              SD_TMR                    TON    ??              SD_FF                 RS    ??              DS_FF                 RS    ??              DS_TMR                    TON    ??              SL_FF                 RS    ??              SL_TMR                    TON    ??           
      N            ??              R0            ??              S0            ??              L            ??              D            ??              P            ??              SD            ??	              DS            ??
              SL            ??              T           ??                 Q            ??                       8'?R  ?    ????           SHL32               inVal32           ??              n           ??                 SHL32                                     8'?R  ?    ????           SHR32               inVal32           ??              n           ??                 SHR32                                     8'?R  ?    ????    s   C:\TWINCAT\PLC\LIB\TcBaseMath.lib @                                                                                          FW_FLOOR               lr_in                        ??                 FW_Floor                                                  |)A  ?   ????           FW_LREALFRAC               lr_in                        ??                 FW_LrealFrac                                                  |)A  ?   ????           FW_LREALMODP               lr_val                        ??              lr_mod                        ??                 FW_LrealModP                                                  |)A  ?   ????           FW_LREALTRUNC               lr_in                        ??                 FW_LrealTrunc                                                  |)A  ?   ????    o   C:\TWINCAT\PLC\LIB\TcMath.lib @                                                                                          F_GETVERSIONTCMATH               nVersionElement           ??                 F_GetVersionTcMath                                     r?RA  ?    ????           FLOOR               lr_in                        ??                 FLOOR                                                  r?RA  ?    ????           FRAC               lr_in                        ??                 FRAC                                                  r?RA  ?    ????           LMOD               lr_Value                        ??              lr_Arg                        ??                 LMOD                                                  r?RA  ?    ????           LTRUNC               lr_in                        ??                 LTRUNC                                                  r?RA  ?    ????           MODABS               lr_Value                        ??              lr_Arg                        ??                 MODABS                                                  r?RA  ?    ????           MODTURNS           lr_Tmp                         ??                 lr_Value                        ??              lr_Arg                        ??                 MODTURNS                                     r?RA  ?    ????    n   C:\TWINCAT\PLC\LIB\TcMC2.lib @                                                                                a          _F_AXISSTATE                   _F_AxisState               MC_AxisStates                       NcToPlc                                        NCTOPLC_AXIS_REF` ??              PlcToNc                                   PLCTONC_AXIS_REF` ??                   6??S  ?    ????           _F_GETINDEXGROUP        
   IndexGroup         ` ??              IndexOffset         ` ??	              nType               _E_ParameterType ` ??
              lrValue                      ` ??              
   ParaNumber         ` ??              Mode         ` ??           read/write       _F_GetIndexGroup                    _ST_ParaStruct                             6??S  ?    ????           _F_NCCYCLECOUNTERUPDATED           pData                   _InternalAxisRefData         ??	              NcCycleCounter            ??
                 LastNcCycleCounter           ??                 _F_NcCycleCounterUpdated                                Axis                AXIS_REF  ??                   6??S  ?    ????           _F_READSTATUS           GetTaskIndex                GETCURTASKINDEX ` ??                 ForceTaskIndexUpdate          ` ??       `    force update of the taskindex with GETCURTASKINDEX - otherwise update only once - 20100416 KSt       _F_ReadStatus        1                                                       ST_AxisStatus                       NcToPlc                                        NCTOPLC_AXIS_REF` ??              PlcToNc                                   PLCTONC_AXIS_REF` ??           
   LastStatus         1                                                       ST_AxisStatus` ??	                   6??S  ?    ????           _F_TCMC_DWORD_TO_HEXSTR           str1             ` ??              pstr               ` ??	              i         ` ??
              digit         ` ??                 in         ` ??                 _F_TcMC_DWORD_TO_HEXSTR                                         6??S  ?    ????           _F_TCMC_ROUND           n                      ` ??                 value                      ` ??              digits         ` ??                 _F_TcMC_Round                                                  6??S  ?    ????           _F_UPDATENCCYCLECOUNTER                   _F_UpdateNcCycleCounter                               Axis                AXIS_REF  ??                   6??S  ?    ????           _FB_MOVEUNIVERSALGENERIC           iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??               sStartRequest                      !   _ST_TcNC_UnversalAxisStartRequest    ??!              sStartResponse             "   _ST_TcNC_UnversalAxisStartResponse    ??"              fbAdsReadWrite                          
   ADSRDWRTEX    ??#              ContinousMode             ??$           
   InVelocity             ??%              DiffCycleCounter            ??&       5    last recent PLC cycle counter difference while Busy    EmptyStartResponse             "   _ST_TcNC_UnversalAxisStartResponse ` ??*              COUNT_R         ` ??+              CounterCmdNoZero         ` ??,              CounterCmdNotStarted         ` ??-           	   DiffCmdNo         ` ??.              NcCycleCounter         ` ??/              LastNcCycleCounter         ` ??0              NcMappingCounter         ` ??1              NcCycleCounterAvailable          ` ??2              NcCycleCounterNotAvailable          ` ??3              NcCyclicFeedbackExpected          ` ??4              PlcDebugCode         ` ??5       a    additional information for debugging purposes will be added to debug messages (ActErrorMessage)    AxisIsSlave          ` ??6              GetTaskIndex                GETCURTASKINDEX ` ??7              CycleCounter         ` ??8       *    last recent PLC cycle counter while Busy 	   fbTimeOut                    TON ` ??9              fbStopMonitoringTimeOut                    TON ` ??:              fbTimeOutMappingCounter                    TON ` ??;              fbOnTrigger                 R_TRIG ` ??<              sTempMsg             ` ??=              OpMode                  _ST_TcNc_OperationModes ` ??@                 Execute            ??          	StartType		: 	UDINT; 	   StartType               _E_TCNC_StartPosType   ??	       #    20110511 KSt type changed for Tc3    Position                        ??
              Velocity                        ??              Acceleration                        ??              Deceleration                        ??              Jerk                        ??       6   	Direction			:	MC_Direction := MC_Positive_Direction;	
   BufferMode               MC_BufferMode   ??              Options                  ST_MoveOptions   ??              Reset            ??           for internal use only    GotoRunState            ??           for internal use only       Done            ??       :    Same meaning as InVelocity for continous motion commands    Busy            ??              Active            ??              CommandAborted            ??              Error            ??              ErrorID           ??              CmdNo           ??              ADSbusy            ??                 Axis                AXIS_REF  ??              LastExecutionResult                   _ST_FunctionBlockResults  ??                   6??S  ?    ????           _FB_PHASINGGENERIC           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??          	fbAdsReadWrite	: 	ADSRDWRTEX;
   fbAdsWrite                          ADSWRITE    ??!           
   sNcPhasing                           _ST_TcNC_PhasingRequest    ??"              PhasingStarted             ??#           	   fbTrigger                 R_TRIG ` ??'           	   fbTimeOut                    TON ` ??(           
      Execute            ??           B 	   StartType               _E_TcNC_StartPosType   ??	           command type start, stop etc, 
   PhaseShift                        ??
           B    PhasingType               E_PhasingType   ??       1    type of phase shift value, absolute or relative    Velocity                        ??           E    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options               ST_PhasingOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E    PhaseShiftFeedback                        ??           E       Master                AXIS_REF  ??              Slave                AXIS_REF  ??                   6??S  ?    ????        !   _FB_POSITIONCORRECTIONTABLELOOKUP           nState            ??              dwSize            ??              eActiveDirection           WorkDirectionNone       E_WorkDirection    ??           	   bExecCalc             ??           
   stElement1             #   ST_PositionCompensationTableElement    ??           
   stElement2             #   ST_PositionCompensationTableElement    ??            
   nLeftIndex            ??!       #    Index starting from 0 up to (N-1)    fSetPos                         ??"       A    position setpoint (absolute linear position or modulo position) 	   pTmpTable              #   ST_PositionCompensationTableElement         ??$       0    helper variable: pointer to table of this type    fTmpStep                         ??%           helper variable    fTmpA                         ??&           helper variable    fTmpB                         ??'           helper variable 
   fTmpDelta1                         ??(           helper variable 
   fTmpDelta2                         ??)           helper variable    ERRORCODE_PARAMSIZE    CK     ??,              ERRORCODE_PARAMPOS    DK     ??-       +   	ERRORCODE_TCNCNOTIMPL	: UDINT := 16#4B45;       Enable            ??       )    rising edge triggers initialize routine    pTable              #   ST_PositionCompensationTableElement        ??           pointer to table of this type 	   TableSize           ??       +    size of data in bytes related to 'pTable'    TableParameter                %   ST_PositionCompensationTableParameter   ??       1    position compensation table parameter structure       Compensation                        ??              Error            ??              ErrorID           ??              Active            ??                 Axis                Axis_Ref  ??                   6??S  ?    ????           _FB_READWRITEPARAMETER           iState           STATE_INITIALIZATION       _E_TcMC_STATES ` ??           	   fbAdsRead                          ADSREAD ` ??           
   fbAdsWrite                          ADSWRITE ` ??              dwValue         ` ??              lrValue   	  
                                 ` ??              NcBoolValue         ` ??              bStarted          ` ??!              stParaStruct                    _ST_ParaStruct ` ??"              n         ` ??#              i         ` ??$           	   ParaLREAL        ` ??(          	ParaBOOL		:	INT := 3;      Enable          ` ??              ParameterNumber               MC_AxisParameter ` ??              Mode         ` ??	           read/write    ParameterType         ` ??
           bool/ not  bool       Done          ` ??              Busy          ` ??              Error          ` ??              ErrorID         ` ??              ADSbusy          ` ??                 Axis                AXIS_REF` ??           
   ValueLreal                      ` ??           
   ValueDword         ` ??           	   ValueBool          ` ??                   6??S  ?    ????           _FBAXIS_REF        	   _internal                  _InternalAxisRefData ` ??              Storage   	                       ` ??       $    universal storage for internal use       PlcToNc                                  PLCTONC_AXIS_REF   ??                 NcToPlc                                       NCTOPLC_AXIS_REF  ??              ADS                 ST_AdsAddress   ??              Status        1                                                       ST_AxisStatus   ??                       6??S  ?   ????           _MC_HALTPHASING           PhasingGeneric                                         _FB_PhasingGeneric    ??                 Execute            ??           B    Deceleration                        ??           E    Jerk                        ??	           E 
   BufferMode               MC_BufferMode   ??
           E    Options               ST_PhasingOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Master                AXIS_REF  ??              Slave                AXIS_REF  ??                   6??S  ?    ????           _MC_PHASINGABSOLUTE           PhasingGeneric                                         _FB_PhasingGeneric    ??$                 Execute            ??           B 
   PhaseShift                        ??           B    Velocity                        ??           E    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options               ST_PhasingOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??            E    AbsolutePhaseShift                        ??!           E       Master                AXIS_REF  ??              Slave                AXIS_REF  ??                   6??S  ?    ????           _MC_PHASINGRELATIVE           PhasingGeneric                                         _FB_PhasingGeneric    ??&                 Execute            ??           B 
   PhaseShift                        ??           B    Velocity                        ??           E    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options               ST_PhasingOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??            E    Error            ??!           B    ErrorID           ??"           E    CoveredPhaseShift                        ??#           E       Master                AXIS_REF  ??              Slave                AXIS_REF  ??                   6??S  ?    ????           _TCMC_ADSRDWRT        
   fbAdsRdWrt                                    FW_AdsRdWrt    ??              NcCycleCounter            ??              NcCycleCounterAdsEnd            ??              UpdateCounter            ??               NoUpdateCounter            ??!           used for timeout detection    state            ??"           
      NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              WRITELEN           ??              READLEN           ??              SRCADDR           ??              DESTADDR           ??              WRTRD            ??              TMOUT    ?     ??                 BUSY            ??              ERR            ??              ERRID           ??              COUNT_R           ??           count of bytes actually read       Axis                AXIS_REF  ??                   6??S  ?   ????           _TCMC_ADSREAD        	   fbAdsRead                              
   FW_AdsRead    ??              NcCycleCounter            ??              NcCycleCounterAdsEnd            ??              UpdateCounter            ??              NoUpdateCounter            ??           used for timeout detection    state            ??                  NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              LEN           ??              DESTADDR           ??              READ            ??              TMOUT    ?     ??                 BUSY            ??              ERR            ??              ERRID           ??              COUNT_R           ??           count of bytes actually read       Axis                AXIS_REF  ??                   6??S  ?   ????           _TCMC_ADSWRITE        
   fbAdsWrite                                FW_AdsWrite    ??              NcCycleCounter            ??              NcCycleCounterAdsEnd            ??              UpdateCounter            ??              NoUpdateCounter            ??           used for timeout detection    state            ??                 NETID            
   T_AmsNetId   ??              PORT            	   T_AmsPort   ??              IDXGRP           ??              IDXOFFS           ??              LEN           ??              SRCADDR           ??              WRITE            ??              TMOUT    ?     ??                 BUSY            ??              ERR            ??              ERRID           ??                 Axis                AXIS_REF  ??                   6??S  ?   ????           _TCMCGLOBAL     	      Axis                 _ST_NCADS_Axis   ??       A    IDXGRP and IDXOFFS constants of axis parameter/status/functions    Table                _ST_NCADS_Table   ??       B    IDXGRP and IDXOFFS constants of table parameter/status/functions     NCPORT_TCNCCAMMING_TABLEFUNCTION    ?     ??       	    timeout    tTargetPosTimeOut    p     ??       T    20050128 KSt - changed from 5 sec to 6 sec to be later than a NC PEH error (5 sec)    tADSTimeOut    ?     ??              tStopMonitoringTimeOut    d      ??       C    20111208 KSt - new for stop monitoring (axis in standstill window    NCTOPLC_FEEDBACK_MAXWAITCYCLES    
      ??       \    maximum number of PLC cycles to wait for a cyclic feedback in NcToPlc after an ADS command    fbADSRDDEVINFO        
                ADSRDDEVINFO    ??              DeviceVersion            ??                  NCPORT_TCMC    ?     ??       (    20110511 type changed from INT to UINT    NCPORT_TCMC_COUPLING    ?     ??       6    used with all axis coupling commands - 12.7.2006 KSt    NCNETID_TCMC           ''    
   T_AmsNetId   ??              NCPORT_TCMC_CAM    ?     ??       (    20110511 type changed from INT to UINT    NCNETID_TCMC_CAM           ''    
   T_AmsNetId   ??                 NcDeviceInfoTcMainVersion           ??              NcDeviceInfoTcSubVersion           ??              NcDeviceInfoNcDriverVersion           ??              NcDeviceInfoNcVersion           ??              NcDeviceInfoNcName               ??                       6??S  ?   ????           F_AXISCAMDATAQUEUED            
   StateDWord           ??
                 F_AxisCamDataQueued                                      6??S  ?    ????           F_AXISCAMSCALINGPENDING            
   StateDWord           ??                 F_AxisCamScalingPending                                      6??S  ?    ????           F_AXISCAMTABLEQUEUED            
   StateDWord           ??
                 F_AxisCamTableQueued                                      6??S  ?    ????           F_AXISCONTROLLOOPCLOSED            
   StateDWord           ??                 F_AxisControlLoopClosed                                      6??S  ?    ????           F_AXISEXTERNALLATCHVALID            
   StateDWord           ??                 F_AxisExternalLatchValid                                      6??S  ?    ????           F_AXISGOTNEWTARGETPOSITION            
   StateDWord           ??
                 F_AxisGotNewTargetPosition                                      6??S  ?    ????           F_AXISHASBEENSTOPPED            
   StateDWord           ??                 F_AxisHasBeenStopped                                      6??S  ?    ????           F_AXISHASEXTSETPOINTGEN            
   StateDWord           ??                 F_AxisHasExtSetPointGen                                      6??S  ?    ????           F_AXISHASJOB            
   StateDWord           ??                 F_AxisHasJob                                      6??S  ?    ????           F_AXISINERRORSTATE            
   StateDWord           ??                 F_AxisInErrorState                                      6??S  ?    ????           F_AXISINPOSITIONWINDOW            
   StateDWord           ??                 F_AxisInPositionWindow                                      6??S  ?    ????           F_AXISINPROTECTEDMODE            
   StateDWord           ??                 F_AxisInProtectedMode                                      6??S  ?    ????           F_AXISINPTPMODE            
   StateDWord           ??
                 F_AxisInPTPmode                                      6??S  ?    ????           F_AXISIODATAISINVALID            
   StateDWord           ??                 F_AxisIoDataIsInvalid                                      6??S  ?    ????           F_AXISISATTARGETPOSITION            
   StateDWord           ??                 F_AxisIsAtTargetPosition                                      6??S  ?    ????           F_AXISISCALIBRATED            
   StateDWord           ??                 F_AxisIsCalibrated                                      6??S  ?    ????           F_AXISISCALIBRATING            
   StateDWord           ??                 F_AxisIsCalibrating                                      6??S  ?    ????           F_AXISISCOMPENSATING            
   StateDWord           ??                 F_AxisIsCompensating                                      6??S  ?    ????           F_AXISISCOUPLED               nCoupleState           ??                 F_AxisIsCoupled                                      6??S  ?    ????           F_AXISISMOVING            
   StateDWord           ??                 F_AxisIsMoving                                      6??S  ?    ????           F_AXISISMOVINGBACKWARDS            
   StateDWord           ??                 F_AxisIsMovingBackwards                                      6??S  ?    ????           F_AXISISMOVINGENDLESS            
   StateDWord           ??                 F_AxisIsMovingEndless                                      6??S  ?    ????           F_AXISISMOVINGFORWARD            
   StateDWord           ??                 F_AxisIsMovingForward                                      6??S  ?    ????           F_AXISISNOTMOVING            
   StateDWord           ??                 F_AxisIsNotMoving                                      6??S  ?    ????           F_AXISISREADY            
   StateDWord           ??                 F_AxisIsReady                                      6??S  ?    ????           F_AXISJOBPENDING            
   StateDWord           ??
                 F_AxisJobPending                                      6??S  ?    ????           F_AXISMOTIONCOMMANDSLOCKED            
   StateDWord           ??                 F_AxisMotionCommandsLocked                                      6??S  ?    ????           F_AXISPHASINGACTIVE            
   StateDWord           ??                 F_AxisPhasingActive                                      6??S  ?    ????           F_AXISREACHEDCONSTANTVELOCITY            
   StateDWord           ??                 F_AxisReachedConstantVelocity                                      6??S  ?    ????           F_GETVERSION_TCMC2               nVersionElement           ??                 F_GetVersion_TcMC2                                     6??S  ?    ????           MC_ABORTSUPERPOSITION           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              fbOnTrigger                 R_TRIG    ??           
   fbADSwrite                          ADSWRITE    ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??                 Execute            ??           Starts touch probe recording       Done            ??           move completed    Busy            ??       "    function block is currently Busy    Error            ??       6    Signals that error has occured within Function Block    ErrorID           ??	           Error identification       Axis                AXIS_REF  ??       f    Identifies the axis of which the position should be recorded at a defined event at the trigger input         6??S  ?    ????           MC_ABORTTRIGGER           ADSbusy             ??           
   fbADSwrite                          ADSWRITE    ??              OLDADSINTERFACE         ` ??       A    temporary flag to test old and new NC ADS touch probe interface    LatchID         ` ??              fbOnTrigger                 R_TRIG ` ??                 Execute            ??
           B       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B    TriggerInput                       TRIGGER_REF  ??           E         6??S  ?    ????           MC_ABORTTRIGGER_V2_00           ADSbusy             ??           
   fbADSwrite                          ADSWRITE    ??              i            ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??              LatchID         ` ??              fbOnTrigger                 R_TRIG ` ??              stTouchProbeDeactivation                 _ST_TcNc_TouchProbeDeactivation ` ??                 Execute            ??
           B       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B    TriggerInput                       TRIGGER_REF  ??           E         6??S  ?    ????           MC_BACKLASHCOMPENSATION           InternalEnable             ??&            trick for internal FB handling    state            ??'              GetThisTaskIndex                GETCURTASKINDEX    ??(           	   CycleTime                         ??)           task cycle time [s]    fbCalcBacklashCorrection                                    !   _FB_PositionCorrectionTableLookup    ??*       4    based on old 'FB_PositionCompensation' in TcNc.lib    fbFeedBacklashCorrection                                    MC_PositionCorrectionLimiter    ??+       N    s.TcMC2.lib (original based on old 'FB_WritePositionCorrection' in TcNc.lib)    CalcBacklashCorrOut                    ST_McOutputs    ??,              FeedBacklashCorrOut                    ST_McOutputs    ??-              ReadParameter                              MC_ReadParameter    ??.           	   iBacklash                         ??/              InternalAcceleration                         ??0       E    input of FB 'MC_PositionCorrectionLimiter': 'Acceleration' [mm/s^2]    InternalBacklashValue                         ??1       M    output of FB 'MC_PositionCorrectionLimiter': 'PositionCorrectionValue' [mm]    InternalLimitingActive             ??2       ?    output of FB 'MC_PositionCorrectionLimiter': 'Limiting' [0/1]    stPosCompParameter       w   
								( MinPosition := -1.0E+12, MaxPosition :=1.0E+12, NoOfTableElements :=2, Direction := WorkDirectionNegative )       ??m?   -1.0E+12    ??mB   1.0E+12            WorkDirectionBoth       E_WorkDirection         %   ST_PositionCompensationTableParameter    ??4              stPosCompTable   	                   #   ST_PositionCompensationTableElement   o   
								( Position := -1.0E+12,	Compensation := 0.0 ),
								( Position := +1.0E+12,	Compensation := 0.0 )                 ??m?   -1.0E+12            0.0              ??mB   1.0E+12            0.0    ??7                 Enable            ??       *    switch to activate backlash compensation    Backlash    3t?<{?   1.0E3073t?<{?   ??       ~    signed backlash value [mm] (when using default value the internal nc backlash value will be read by ADS and used in this FB)    CompensationInPositiveDirection            ??       @    compensation is just working in the selected working direction    Ramp                        ??       ?    velocity limit for feeded backlash compensation (constant velocity and linear position sub profile for backlash compensation) [mm/s] (default:=0.0)    DisableMode               E_DisableMode   ??       S    disable mode defines whow to react in case of disabling: (0)=HOLD, (1)=RESET, ...    Options               ST_BacklashCompensationOptions   ??       $    optional parameters (NOT USED YET)       Enabled            ??              Busy            ??              Error            ??               ErrorID           ??!              CurrentBacklash                        ??"       $    current actual backlash value [mm]    Limiting            ??#       >    function block is currently limiting the Backlash Correction       Axis                Axis_Ref  ??                   6??S  ?   ????           MC_EXTSETPOINTGENDISABLE           LastExecutionResult                  _ST_FunctionBlockResults    ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??              ADSbusy             ??           
   fbADSwrite                          ADSWRITE    ??              fbOnTrigger                 R_TRIG ` ??              TimerStateFeedback                    TON ` ??                 Execute            ??                 Done            ??	              Busy            ??
              Error            ??              ErrorID           ??              Enabled            ??                 Axis                AXIS_REF  ??                   6??S  ?    ????           MC_EXTSETPOINTGENENABLE           LastExecutionResult                  _ST_FunctionBlockResults    ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??              ADSbusy             ??              sExtSetPointGen                _ST_TcNC_SetPos    ??           
   fbADSwrite                          ADSWRITE    ??              fbOnTrigger                 R_TRIG ` ??              TimerStateFeedback                    TON ` ??                 Execute            ??              Position                        ??              PositionType               E_PositionType   ??                 Done            ??              Busy            ??              Error            ??              ErrorID           ??              Enabled            ??                 Axis                AXIS_REF  ??                   6??S  ?    ????           MC_EXTSETPOINTGENFEED           GetTaskIndex                GETCURTASKINDEX    ??                 Position                        ??              Velocity                        ??              Acceleration                        ??           	   Direction           ??	                 MC_ExtSetPointGenFeed                                Axis                AXIS_REF  ??                   6??S  ?    ????        	   MC_GEARIN           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??           
   fbAdsWrite                          ADSWRITE    ??              sCouple                      _ST_TcNC_CoupleSlave    ??               fbOptGearInDyn                                         MC_GearInDyn    ??!              fbOnTrigger                 R_TRIG ` ??%              TimerStateFeedback                    TON ` ??&                 Execute            ??           B    RatioNumerator          ??   1      ??   ??
       %    changed from INT (PLCopen) to LREAL    RatioDenominator          ??           	MasterValueSource :	MC_SOURCE;    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options               ST_GearInOptions   ??           V       InGear            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Master                AXIS_REF  ??           B    Slave                AXIS_REF  ??           B         6??S  ?    ????           MC_GEARINDYN           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??               iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??!           	   iSubState            ??"           
   fbAdsWrite                          ADSWRITE    ??#           	   fbAdsRead                          ADSREAD    ??$              sCouple                      _ST_TcNC_CoupleSlave    ??%              v_max                         ??&              pa_limit                         ??'           	   WasInGear             ??(              iAcceleration                      ` ??,              TimerStateFeedback                    TON ` ??-                 Enable            ??           	   GearRatio          ??   1.0      ??   ??              Acceleration                        ??              Deceleration                        ??       
    not used    Jerk                        ??       
    not used 
   BufferMode               MC_BufferMode   ??           E    Options               ST_GearInDynOptions   ??           V       InGear            ??              Busy            ??              Active            ??              CommandAborted            ??              Error            ??              ErrorID           ??                 Master                AXIS_REF  ??
              Slave                AXIS_REF  ??                   6??S  ?    ????           MC_GEARINMULTIMASTER           ADSbusy             ??%              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??&           
   fbAdsWrite                          ADSWRITE    ??'              sCouple                           _ST_TcNC_CoupleSlaveMultiMaster    ??(           	   sCoupleV2                                  _ST_TcNC_CoupleSlaveMultiMaster2    ??)              LastExecutionResult                  _ST_FunctionBlockResults    ??*           	   IsCoupled             ??+              TimerStateFeedback                    TON ` ??/              iAdvancedSlaveDynamics          ` ??0              iAcceleration                      ` ??1              iDeceleration                      ` ??2           just used in version 2 mode    iJerk                      ` ??3           just used in version 2 mode 	   WasInGear          ` ??4              ParameterChanged          ` ??5           
      Enable            ??           
   GearRatio1          ??   1.0      ??   ??           
   GearRatio2          ??   1.0      ??   ??           
   GearRatio3          ??   1.0      ??   ??           
   GearRatio4          ??   1.0      ??   ??              Acceleration                        ??              Deceleration                        ??       5    just used in version 2 mode (AdvancedSlaveDynamics)    Jerk                        ??       5    just used in version 2 mode (AdvancedSlaveDynamics) 
   BufferMode               MC_BufferMode   ??           E    Options                ST_GearInMultiMasterOptions   ??           V       InGear            ??              Busy            ??              Active            ??              CommandAborted            ??               Error            ??!              ErrorID           ??"                 Master1                AXIS_REF  ??
              Master2                AXIS_REF  ??              Master3                AXIS_REF  ??              Master4                AXIS_REF  ??              Slave                AXIS_REF  ??                   6??S  ?    ????        
   MC_GEAROUT           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??           
   fbAdsWrite                          ADSWRITE    ??              fbOnTrigger                 R_TRIG ` ??              TimerStateFeedback                    TON ` ??                 Execute            ??           B    Options               ST_GearOutOptions   ??           V       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Slave                AXIS_REF  ??	           B         6??S  ?    ????           MC_HALT           LastExecutionResult                  _ST_FunctionBlockResults    ??$              ADSbusy             ??%              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??&              CmdNo            ??'                 Execute            ??           B    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??            B    ErrorID           ??!           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_HOME           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??            	   iSubState            ??!              fbAdsWriteCmd                          ADSWRITE    ??"              fbAdsWriteRefPos                          ADSWRITE    ??#              fbAdsReadRefPos                          ADSREAD    ??$              ReferenceFlagValue            ??%              fbSetPosition                            MC_SetPosition    ??&              NcHomePosition                         ??'           	   fbTrigger                 R_TRIG ` ??+           	   fbTimeOut                    TON ` ??,                 Execute            ??           B    Position    3t?<{?   1.0E3073t?<{?   ??           
   HomingMode               MC_HomingMode   ??           V 
   BufferMode               MC_BufferMode   ??           E    Options               ST_HomingOptions   ??           V    bCalibrationCam            ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?   ????           MC_JOG           state            ??              LastExecutionResult                  _ST_FunctionBlockResults    ??           
   StateDWord             ??              ExecuteMoveVelocity          ` ??               MoveVelocity                                 MC_MoveVelocity ` ??!              MoveVelocityOut                    ST_McOutputs ` ??"           	   Direction               MC_Direction ` ??#              ExecuteHalt          ` ??%              Halt                              MC_Halt ` ??&              HaltOut                    ST_McOutputs ` ??'              ExecuteMoveAbsolute          ` ??)              MoveAbsolute                                 MC_MoveAbsolute ` ??*              MoveAbsoluteOut                    ST_McOutputs ` ??+              ExecuteMoveRelative          ` ??-              MoveRelative                                 MC_MoveRelative ` ??.              MoveRelativeOut                    ST_McOutputs ` ??/              JogMove        1                                                       _FB_MoveUniversalGeneric ` ??1              LastJogMoveResult                  _ST_FunctionBlockResults ` ??2              ExecuteJogMove          ` ??3           	   StartType               _E_TCNC_StartPosType ` ??4           
   JogMoveOut                    ST_McOutputs ` ??5              JogEnd          ` ??7              TargetPosition                      ` ??8              modulo                      ` ??9              
   JogForward            ??              JogBackwards            ??              Mode            	   E_JogMode   ??	              Position                        ??
              Velocity                        ??              Acceleration                        ??              Deceleration                        ??              Jerk                        ??          	BufferMode		:	MC_BufferMode;      Done            ??              Busy            ??              Active            ??              CommandAborted            ??              Error            ??              ErrorID           ??                 Axis                AXIS_REF  ??                   6??S  ?    ????           MC_MOVEABSOLUTE           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??              CmdNo            ??                 Execute            ??           B    Position                        ??	           B    Velocity                        ??
           E    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_MOVEADDITIVE           LastExecutionResult                  _ST_FunctionBlockResults    ??%              ADSbusy             ??&              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??'              CmdNo            ??(                 Execute            ??           B    Distance                        ??           B    Velocity                        ??           E    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??            E    Error            ??!           B    ErrorID           ??"           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_MOVECONTINUOUSABSOLUTE           LastExecutionResult                  _ST_FunctionBlockResults    ??+              ADSbusy             ??,              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??-              CmdNo            ??.           	      Execute            ??           B    Position                        ??           B    Velocity                        ??           B    EndVelocity                        ??           B    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??                  InEndVelocity            ??#           B    Busy            ??$           E    Active            ??%           E    CommandAborted            ??&           E    Error            ??'           B    ErrorID           ??(           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_MOVECONTINUOUSRELATIVE           LastExecutionResult                  _ST_FunctionBlockResults    ??*              ADSbusy             ??+              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??,              CmdNo            ??-           	      Execute            ??           B    Distance                        ??           B    Velocity                        ??           B    EndVelocity                        ??           B    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??                 InEndVelocity            ??"           B    Busy            ??#           E    Active            ??$           E    CommandAborted            ??%           E    Error            ??&           B    ErrorID           ??'           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_MOVEMODULO           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??          	StartType: UDINT;	   StartType               _E_TcNC_StartPosType    ??       #    20110511 KSt type changed for TC3    CmdNo            ??              TriggerExecute                 R_TRIG ` ??#           	      Execute            ??              Position                        ??              Velocity                        ??	              Acceleration                        ??
              Deceleration                        ??              Jerk                        ??           	   Direction               MC_Direction   ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??                 Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??                   6??S  ?    ????           MC_MOVERELATIVE           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??              CmdNo            ??                 Execute            ??	           B    Distance                        ??           B    Velocity                        ??           E    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_MOVESUPERIMPOSED     
      LastExecutionResult                  _ST_FunctionBlockResults    ??1              ADSbusy             ??2              CompensationStarted             ??3              AxisHasJobAtStartOfCompensation             ??4       ,    HasJob flag when starting the compensation !   AxisIsMovingAtStartOfCompensation             ??5       ,    Moving flag when starting the compensation    iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??6              fbAdsReadWrite                          
   ADSRDWRTEX    ??7              sNcCompensation                      _ST_TcNC_Compensation2    ??8           	   fbTrigger                 R_TRIG ` ??<           	   fbTimeOut                    TON ` ??=           
      Execute            ??           B    Mode               E_SuperpositionMode   ??           V    Distance                        ??           B    VelocityDiff                        ??           E    Acceleration                        ??           E    Deceleration                        ??           E    Jerk                        ??           E    VelocityProcess                        ??           V    Length                        ??           V    Options               ST_SuperpositionOptions   ??           V       Done            ??!           B    Busy            ??"           E    Active            ??#           E    CommandAborted            ??$           E    Error            ??%           B    ErrorID           ??&           E    Warning            ??(           V 	   WarningId           ??)           V    ActualVelocityDiff                        ??*           V    ActualDistance                        ??+           V    ActualLength                        ??,           V    ActualAcceleration                        ??-           V    ActualDeceleration                        ??.           V       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_MOVEVELOCITY           LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??              MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??              CmdNo            ??                 Execute            ??           B    Velocity                        ??	           E    Acceleration                        ??
           E    Deceleration                        ??           E    Jerk                        ??           E 	   Direction           MC_Positive_Direction       MC_Direction   ??           E 
   BufferMode               MC_BufferMode   ??           E    Options                  ST_MoveOptions   ??           V    
   InVelocity            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_OVERRIDEFILTER           bFirstCycle            ??              bThresholdActive            ??              InternalOverrideValueRaw            ??              LastOverrideValueRaw    ??      ??              OverrideRange            ??              fbTimer                    TON ` ??                 OverrideValueRaw           ??           parameter    LowerOverrideThreshold           ??           0...32767 digits    UpperOverrideThreshold    ?     ??           0...32767 digits    OverrideSteps    ?      ??           200 steps => 0.5 percent    OverrideRecoveryTime    ?      ??	           150 ms       OverrideValueFiltered           ??           0...1000000 counts    OverridePercentFiltered                        ??           0...100 %    Error            ??              ErrorID           ??                       6??S  ?    ????           MC_POSITIONCORRECTIONLIMITER           GetThisTaskIndex                GETCURTASKINDEX    ??(           	   CycleTime                         ??)              MaxDeltaVelocity                         ??*              MaxDeltaPosition                         ??+              DeltaCorrection                         ??,              InitialDeltaCorrection                         ??-              EndOfEnablePhase             ??.              iCorrectionMode               E_AxisPositionCorrectionMode    ??/              state            ??0              NumberOfCycles            ??1              DeltaCorrectionPerCycle                         ??2              LastPositionCorrectionValue                         ??3                 Enable            ??              PositionCorrectionValue                        ??              CorrectionMode               E_AxisPositionCorrectionMode   ??              Acceleration                        ??              CorrectionLength                        ??       8    optional length - comparable to 'superposition length'       Busy            ??"              Error            ??#              ErrorID           ??$              Limiting            ??%       >    function block is currently limiting the Position Correction       Axis                AXIS_REF  ??                   6??S  ?    ????           MC_POWER           EnableTimeout                    TON ` ??              EnableOffOnDelay                   TP ` ??          	iOverride: DINT;	   iOverride         ` ??        $    20110511 KSt type adaption for TC3       Enable            ??           B    Enable_Positive            ??           E    Enable_Negative            ??           E    Override          Y@   100.0      Y@   ??       )    in percent - Beckhoff proprietary input 
   BufferMode               MC_BufferMode   ??           V       Status            ??           B    Busy            ??           V    Active            ??           V    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??
           B         6??S  ?    ????           MC_POWERSTEPPER     $      fbPower                            MC_Power    ??           	   ErrorCode            ??              fbWriteErrCode                          ADSWRITE    ??           	   nRefState            ??              fbWriteNonRef                          ADSWRITE    ??              fbReadParams                          ADSREAD    ??              fbWriteInstOvr                          ADSWRITE    ??              bAdsInitDone             ??           	   bOverTemp             ??           
   rtOverTemp                 R_TRIG    ??               bUnderVoltage             ??!              rtUnderVoltage                 R_TRIG    ??"           
   bOpenLoopA             ??#              rtOpenLoopA                 R_TRIG    ??$           
   bOpenLoopB             ??%              rtOpenLoopB                 R_TRIG    ??&              bOverCurrentA             ??'              rtOverCurrentA                 R_TRIG    ??(              bOverCurrentB             ??)              rtOverCurrentB                 R_TRIG    ??*              rtStallError                 R_TRIG    ??+           	   bOldState             ??,              bLagFilterInit             ??.              tonLagFilter                    TON    ??/              tonNoLagFilter                    TON    ??0              wState2            ??2              nAngle            ??3           	   nOldAngle            ??4           	   AngleDiff            ??5              bInErrorState             ??6           	   nParamIdx            ??8              bParamEnabled             ??9           	   fLagLimit                         ??:           
   fLagFilter                         ??;              nTries            ??<              tTimeOut                    TON    ??=                 Enable            ??              Enable_Positive            ??              Enable_Negative            ??              Override          Y@   100.0      Y@   ??	           in percent    DestallParams                    ST_PowerStepperStruct   ??
           	   KL_Status           ??           
   KL_Status2           ??                 Status            ??              Error            ??              ErrorID           ??              Stalled            ??           
   StallError            ??                 Axis                AXIS_REF  ??                   6??S  ?    ????           MC_READACTUALPOSITION               Enable            ??           B       Valid            ??
           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E    Position                        ??           B       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_READACTUALVELOCITY               Enable            ??           B       Valid            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E    ActualVelocity                        ??           B       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_READAPPLICATIONREQUEST           TriggerExecute                 R_TRIG    ??              state           STATE_INITIALIZATION       _E_TcMC_STATES    ??           	   fbAdsRead                          ADSREAD    ??                 Execute            ??                 Done            ??              Busy            ??              Error            ??              ErrorID           ??              Request            ??           application request bit [0/1]    RequestType           ??           application request TYPE/ID       ApplicationRequest                    ST_NcApplicationRequest  ??              Axis                AXIS_REF  ??                   6??S  ?    ????           MC_READAXISCOMPONENTS           TriggerExecute                 R_TRIG    ??              state           STATE_INITIALIZATION       _E_TcMC_STATES    ??           	   fbAdsRead                          ADSREAD    ??                 Execute            ??                 Done            ??
              Busy            ??              Error            ??              ErrorID           ??                 AxisComponents                  ST_AxisComponents  ??              Axis                AXIS_REF  ??                   6??S  ?    ????           MC_READAXISERROR               Enable            ??
           B       Valid            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           B    AxisErrorID           ??           B       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_READBOOLPARAMETER           ADSbusy             ??              fbReadWriteParameter                                      _FB_ReadWriteParameter    ??           	   nParatype               _E_ParameterType ` ??              dwValue         ` ??              lrValue                      ` ??              bStarted          ` ??           	   fbTrigger                 R_TRIG ` ??                 Enable            ??           B    ParameterNumber           ??           B    ReadMode           ReadMode_Once    
   E_ReadMode   ??	           Beckhoff proprietary input       Valid            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E    Value            ??           B       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_READDRIVEADDRESS           state           STATE_INITIALIZATION       _E_TcMC_STATES    ??              TriggerExecute                 R_TRIG    ??           	   fbAdsRead                          ADSREAD    ??           
   readBuffer   	  ?                        ??       H    2013-04-03 KSt - new data structure - size changed from 10 to 64 bytes    i         ` ??              pDword               ` ??                 Execute            ??                 Done            ??
              Busy            ??              Error            ??              ErrorID           ??              DriveAddress                          ST_DriveAddress   ??                 Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_READPARAMETER           ADSbusy             ??              fbReadWriteParameter                                      _FB_ReadWriteParameter    ??           	   nParatype               _E_ParameterType ` ??              dwValue         ` ??              bValue          ` ??              bStarted          ` ??           	   fbTrigger                 R_TRIG ` ??                 Enable            ??           B    ParameterNumber           ??           B    ReadMode           ReadMode_Once    
   E_ReadMode   ??	           Beckhoff proprietary input       Valid            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E    Value                        ??           B       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_READPARAMETERSET           TriggerExecute                 R_TRIG    ??              state           STATE_INITIALIZATION       _E_TcMC_STATES    ??           	   fbAdsRead                          ADSREAD    ??              SizeofPayloadData            ??                 Execute            ??                 Done            ??              Busy            ??              Error            ??              ErrorID           ??              	   Parameter         E                                                                           ST_AxisParameterSet  ??              Axis                AXIS_REF  ??                   6??S  ?    ????           MC_READSTATUS               Enable            ??           B       Valid            ??
           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E 	   ErrorStop            ??           B    Disabled            ??           B    Stopping            ??           B 
   StandStill            ??           B    DiscreteMotion            ??           B    ContinuousMotion            ??           B    SynchronizedMotion            ??           E    Homing            ??           E    ConstantVelocity            ??           V    Accelerating            ??           V    Decelerating            ??           V    Status        1                                                       ST_AxisStatus   ??           V       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_READSTOPINFO           TriggerExecute                 R_TRIG    ??              state           STATE_INITIALIZATION       _E_TcMC_STATES    ??              stStopInfoRequest                _ST_TcNC_StopInfoRequest    ??              stStopInfoResponse                _ST_TcNC_StopInfoResponse    ??              fbAdsReadWrite                            ADSRDWRT    ??                 Execute            ??           B    Deceleration                        ??           E    Jerk                        ??           E       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E    StopDistance                        ??       $    distance required to stop the axis    StopTime                        ??            time required to stop the axis       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_RESET           ADSbusy             ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??              LastExecutionResult                  _ST_FunctionBlockResults    ??           
   fbAdsWrite                                _TcMC_ADSWRITE ` ??           2010-05-31 KSt    fbOnTrigger                 R_TRIG ` ??                 Execute            ??           B       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           B       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_SETACCEPTBLOCKEDDRIVESIGNAL               Enable            ??                 MC_SetAcceptBlockedDriveSignal                                Axis                AXIS_REF  ??                   6??S  ?    ????           MC_SETENCODERSCALINGFACTOR           ADSbusy             ??              stSetEncoderSaclingFactor                  _ST_TcNC_SetEncoderSaclingFactor    ??               iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??!           
   fbAdsWrite                          ADSWRITE    ??"              fbOnTrigger                 R_TRIG ` ??&                 Execute            ??	           B    ScalingFactor                        ??
           B    Mode               E_SetScalingFactorMode   ??           E    Options                ST_SetEncoderScalingOptions   ??           V       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_SETOVERRIDE               Enable            ??           B 	   VelFactor          ??   1.0      ??   ??           1.0 = 100% 	   AccFactor          ??   1.0      ??   ??           1.0 = 100% 
   JerkFactor          ??   1.0      ??   ??           1.0 = 100%       Enabled            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??                   6??S  ?    ????           MC_SETPOSITION           ADSbusy             ??)              stSetPos                   _ST_TcNC_SetPosOnTheFly    ??*              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??+           
   fbAdsWrite                          ADSWRITE    ??,              fbOnTrigger                 R_TRIG ` ??0                 Execute            ??           B    Position                        ??           B    Mode            ??           E    Options                 ST_SetPositionOptions   ??           V       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_STOP     
      LastExecutionResult                  _ST_FunctionBlockResults    ??              ADSbusy             ??               MoveGeneric        1                                                       _FB_MoveUniversalGeneric    ??!              ReleaseLock                          ADSWRITE    ??"              CmdNo            ??#              AxisMotionCommandsLocked             ??$              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??%           
   NoJobTimer                    TON    ??&              fbOnTrigger                 R_TRIG ` ??*              CounterMotionCommandsLocked         ` ??+                 Execute            ??           B    Deceleration                        ??           E    Jerk                        ??           E    Options                  ST_MoveOptions   ??           V       Done            ??           B    Busy            ??           E    Active            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????        !   MC_TABLEBASEDPOSITIONCOMPENSATION           InternalEnable             ??#            trick for internal FB handling    state            ??$              GetThisTaskIndex                GETCURTASKINDEX    ??%           	   CycleTime                         ??&           task cycle time [s]    fbCalcPositionCorrection                                    !   _FB_PositionCorrectionTableLookup    ??'       4    based on old 'FB_PositionCompensation' in TcNc.lib    fbFeedPositionCorrection                                    MC_PositionCorrectionLimiter    ??(       N    s.TcMC2.lib (original based on old 'FB_WritePositionCorrection' in TcNc.lib)    CalcPosCorrOut                    ST_McOutputs    ??)              FeedPosCorrOut                    ST_McOutputs    ??*              InternalAcceleration                         ??+       E    input of FB 'MC_PositionCorrectionLimiter': 'Acceleration' [mm/s^2]    InternalCorrectionValue                         ??,       M    output of FB 'MC_PositionCorrectionLimiter': 'PositionCorrectionValue' [mm]    InternalLimitingActive             ??-       ?    output of FB 'MC_PositionCorrectionLimiter': 'Limiting' [0/1]       Enable            ??       )    rising edge triggers initialize routine    pTable              #   ST_PositionCompensationTableElement        ??       R    pointer to equidistant table with strictly monotonous increasing position values 	   TableSize           ??       +    size of data in bytes related to 'pTable'    TableParameter                %   ST_PositionCompensationTableParameter   ??       1    position compensation table parameter structure    Ramp                        ??       ?    velocity limit for feeded position compensation (constant velocity and linear position sub profile for position compensation) [mm/s] (default:=0.0)    DisableMode               E_DisableMode   ??       S    disable mode defines whow to react in case of disabling: (0)=HOLD, (1)=RESET, ...    Options               ST_PositionCompensationOptions   ??       $    optional parameters (NOT USED YET)       Enabled            ??              Busy            ??              Error            ??              ErrorID           ??              CurrentCorrection                        ??       /    current actual position correction value [mm]    Limiting            ??        >    function block is currently limiting the Position Correction       Axis                Axis_Ref  ??                   6??S  ?    ????           MC_TOUCHPROBE           ADSbusy             ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??              TouchProbeValid   	                         ??            valid state of probes 1..4    TouchProbeActive   	                         ??!       !    activation state of probes 1..4    TouchProbeValue   	                                      ??"       $    last recorded value of probes 1..4    TouchProbeModuloValue   	                                      ??#       +    last recorded modulo value of probes 1..4    TouchProbeInactiveCounter   	                         ??$       O    number of cycles where the probes 1..4 where inactive (activation monitoring)    OLDADSINTERFACE         ` ??(       A    temporary flag to test old and new NC ADS touch probe interface 
   fbADSwrite                          ADSWRITE ` ??+              fbAdsReadValid                          ADSREAD ` ??,              fbAdsReadValue                          ADSREAD ` ??-              fbAdsReadState                          ADSREAD ` ??.              fbAdsReadModulo                          ADSREAD ` ??/              TimerAdsReadState                    TON ` ??0              RtrigPlcEvent                 R_TRIG ` ??1              FtrigPlcEvent                 F_TRIG ` ??2              fbOnTrigger                 R_TRIG ` ??3              LatchID         ` ??4              Restart          ` ??5       H    restart probe sequence when the trigger was outside the defined window    InWindow          ` ??6       *    probe value is inside the defined window    ModuloFactor                      ` ??7       &    axis' modulo factor read from the NC    InitDone          ` ??8       %    initialization on start-up finished    IndexOffset         ` ??9              i         ` ??:                 Execute            ??           B 
   WindowOnly            ??           E    FirstPosition                        ??           E    LastPosition                        ??           E       Done            ??           B    Busy            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E    RecordedPosition                        ??           B       Axis                AXIS_REF  ??           B    TriggerInput                       TRIGGER_REF  ??           B         6??S  ?    ????           MC_TOUCHPROBE_V2_00            ADSbusy             ??              iState           STATE_INITIALIZATION       _E_TcMC_STATES    ??              ExternalLatchValid             ??              TouchProbeValid   	                         ??           valid state of probes 1..4    TouchProbeActive   	                         ??       !    activation state of probes 1..4    TouchProbeValue   	                                      ??       $    last recorded value of probes 1..4    TouchProbeModuloValue   	                                      ??       +    last recorded modulo value of probes 1..4    TouchProbeCounter   	                         ??       ,    last recorded value counter of probes 1..4    TouchProbeInactiveCounter   	                         ??       O    number of cycles where the probes 1..4 where inactive (activation monitoring)    stTouchProbeActivation                   _ST_TcNc_TouchProbeActivation    ??               stTouchProbeStatusRequest                 _ST_TcNc_TouchProbeStatusRequest    ??!              stTouchProbeStatusResponse                      !   _ST_TcNc_TouchProbeStatusResponse    ??"              stTouchProbeDeactivation                 _ST_TcNc_TouchProbeDeactivation    ??#              LastTouchProbeValue   	                                   ` ??'       $    last recorded value of probes 1..4    LastTouchProbeCounter   	                      ` ??(       ,    last recorded value counter of probes 1..4 
   fbADSwrite                          ADSWRITE ` ??)              fbAdsReadValid                          ADSREAD ` ??*              fbAdsReadValue                          ADSREAD ` ??+              fbAdsReadState                          ADSREAD ` ??,              fbAdsReadModulo                          ADSREAD ` ??-              fbAdsReadLatchStatus                          
   ADSRDWRTEX ` ??.              TimerAdsReadState                    TON ` ??/              RtrigPlcEvent                 R_TRIG ` ??0              FtrigPlcEvent                 F_TRIG ` ??1              fbOnTrigger                 R_TRIG ` ??2              Restart          ` ??3       H    restart probe sequence when the trigger was outside the defined window    InWindow          ` ??4       *    probe value is inside the defined window    ModuloFactor                      ` ??5       &    axis' modulo factor read from the NC    InitDone          ` ??6       %    initialization on start-up finished    IndexOffset         ` ??7              iTriggerInput                      TRIGGER_REF ` ??8           B    i         ` ??9                 Execute            ??           B 
   WindowOnly            ??	           E    FirstPosition                        ??
           E    LastPosition                        ??           E       Done            ??           B    Busy            ??           E    CommandAborted            ??           E    Error            ??           B    ErrorID           ??           E    RecordedPosition                        ??           B    RecordedData                  MC_TouchProbeRecordedData   ??           V       Axis                AXIS_REF  ??           B    TriggerInput                       TRIGGER_REF  ??           B         6??S  ?    ????           MC_WRITEBOOLPARAMETER           ADSbusy             ??              fbReadWriteParameter                                      _FB_ReadWriteParameter    ??           	   nParatype               _E_ParameterType ` ??              dwValue         ` ??              lrValue                      ` ??           	   fbTrigger                 R_TRIG ` ??                 Execute            ??           B    ParameterNumber           ??           B    Value            ??	           B       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????           MC_WRITEPARAMETER           ADSbusy             ??              fbReadWriteParameter                                      _FB_ReadWriteParameter    ??           	   nParatype               _E_ParameterType ` ??              dwValue         ` ??              bValue          ` ??           	   fbTrigger                 R_TRIG ` ??                 Execute            ??           B    ParameterNumber           ??           B    Value                        ??	           B       Done            ??           B    Busy            ??           E    Error            ??           B    ErrorID           ??           E       Axis                AXIS_REF  ??           B         6??S  ?    ????    q   C:\TWINCAT\PLC\LIB\STANDARD.LIB @                                                                                          CONCAT               STR1               ??              STR2               ??                 CONCAT                                         ??66  ?   ????           CTD           M             ??           Variable for CD Edge Detection      CD            ??           Count Down on rising edge    LOAD            ??           Load Start Value    PV           ??           Start Value       Q            ??           Counter reached 0    CV           ??           Current Counter Value             ??66  ?   ????           CTU           M             ??            Variable for CU Edge Detection       CU            ??       
    Count Up    RESET            ??           Reset Counter to 0    PV           ??           Counter Limit       Q            ??           Counter reached the Limit    CV           ??           Current Counter Value             ??66  ?   ????           CTUD           MU             ??            Variable for CU Edge Detection    MD             ??            Variable for CD Edge Detection       CU            ??	       
    Count Up    CD            ??
           Count Down    RESET            ??           Reset Counter to Null    LOAD            ??           Load Start Value    PV           ??           Start Value / Counter Limit       QU            ??           Counter reached Limit    QD            ??           Counter reached Null    CV           ??           Current Counter Value             ??66  ?   ????           DELETE               STR               ??              LEN           ??              POS           ??                 DELETE                                         ??66  ?   ????           F_TRIG           M             ??
                 CLK            ??           Signal to detect       Q            ??           Edge detected             ??66  ?   ????           FIND               STR1               ??              STR2               ??                 FIND                                     ??66  ?   ????           INSERT               STR1               ??              STR2               ??              POS           ??                 INSERT                                         ??66  ?   ????           LEFT               STR               ??              SIZE           ??                 LEFT                                         ??66  ?   ????           LEN               STR               ??                 LEN                                     ??66  ?   ????           MID               STR               ??              LEN           ??              POS           ??                 MID                                         ??66  ?   ????           R_TRIG           M             ??
                 CLK            ??           Signal to detect       Q            ??           Edge detected             ??66  ?   ????           REPLACE               STR1               ??              STR2               ??              L           ??              P           ??                 REPLACE                                         ??66  ?   ????           RIGHT               STR               ??              SIZE           ??                 RIGHT                                         ??66  ?   ????           RS               SET            ??              RESET1            ??                 Q1            ??
                       ??66  ?   ????           SEMA           X             ??                 CLAIM            ??	              RELEASE            ??
                 BUSY            ??                       ??66  ?   ????           SR               SET1            ??              RESET            ??                 Q1            ??	                       ??66  ?   ????           TOF           M             ??           internal variable 	   StartTime            ??           internal variable       IN            ??       ?    starts timer with falling edge, resets timer with rising edge    PT           ??           time to pass, before Q is set       Q            ??	       2    is FALSE, PT seconds after IN had a falling edge    ET           ??
           elapsed time             ??66  ?   ????           TON           M             ??           internal variable 	   StartTime            ??           internal variable       IN            ??       ?    starts timer with rising edge, resets timer with falling edge    PT           ??           time to pass, before Q is set       Q            ??	       0    is TRUE, PT seconds after IN had a rising edge    ET           ??
           elapsed time             ??66  ?   ????           TP        	   StartTime            ??           internal variable       IN            ??       !    Trigger for Start of the Signal    PT           ??       '    The length of the High-Signal in 10ms       Q            ??	           The pulse    ET           ??
       &    The current phase of the High-Signal             ??66  ?   ????    ?   c:\documents and settings\administrator\desktop\hobbit\programm\libs\TcpIp.lib @                                                                                          F_GETVERSIONTCPIP               nVersionElement           ??                 F_GetVersionTcpIp                                     ??8U  ?    ????           FB_SOCKETACCEPT        
   fbAdsRdWrt       _    ( PORT := AMSPORT_TCPIPSRV, IDXGRP := TCPADS_IGR_CONLIST, IDXOFFS := TCPADSCONLST_IOF_ACCEPT )              	   T_AmsPort           ?                ADSRDWRT ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              response                ST_TcIpConnSvrResponse ` ??              request                ST_SockAddr ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system	   hListener              	   T_HSOCKET   ??       x    Listener handle identifying a socket that has been placed in a listening state with the FB_SocketListen function block    bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.    	   bAccepted            ??
       ;    TRUE = new connection is made. FALSE = no new connection.    bBusy            ??              bError            ??              nErrId           ??              hSocket              	   T_HSOCKET   ??       V   This returned value is a handle for the socket on which the actual connection is made.            ??8U  ?   ????           FB_SOCKETCLOSE        
   fbAdsWrite       V    ( PORT := AMSPORT_TCPIPSRV, IDXGRP := TCPADS_IGR_CLOSEBYHDL, SRCADDR := 0, LEN := 0 )              	   T_AmsPort           ?                   ADSWRITE ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       4    Local or remote client or listener socket to close.   bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????           FB_SOCKETCLOSEALL        
   fbAdsWrite       y    ( PORT := AMSPORT_TCPIPSRV, IDXGRP := TCPADS_IGR_CONLIST, IDXOFFS := TCPADSCONLST_IOF_CLOSEALL, SRCADDR := 0, LEN := 0 )              	   T_AmsPort           ?                      ADSWRITE ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??	              bError            ??
              nErrId           ??                       ??8U  ?   ????           FB_SOCKETCONNECT        
   fbAdsRdWrt       _    ( PORT := AMSPORT_TCPIPSRV, IDXGRP :=TCPADS_IGR_CONLIST, IDXOFFS := TCPADSCONLST_IOF_CONNECT )              	   T_AmsPort           ?                ADSRDWRT ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              request                ST_SockAddr ` ??              response                ST_TcIpConnSvrResponse ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   sRemoteHost               ??       X    Remote (server) address. String containing an (Ipv4) Internet Protocol dotted address.    nRemotePort           ??       .    Remote (server) Internet Protocol (IP) port.    bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ȯ     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??              hSocket              	   T_HSOCKET   ??       V   This returned value is a handle for the socket on which the actual connection is made.            ??8U  ?   ????           FB_SOCKETLISTEN        
   fbAdsRdWrt       ]    ( PORT := AMSPORT_TCPIPSRV, IDXGRP :=TCPADS_IGR_CONLIST, IDXOFFS :=TCPADSCONLST_IOF_LISTEN )              	   T_AmsPort           ?                ADSRDWRT ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              request                ST_SockAddr ` ??              response                ST_TcIpConnSvrResponse ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system
   sLocalHost               ??       W    Local (server) address. String containing an (Ipv4) Internet Protocol dotted address. 
   nLocalPort           ??       -    Local (server) Internet Protocol (IP) port.    bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??	       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??           	   hListener              	   T_HSOCKET   ??       _   This returned value is a handle for the listener socket on which the actual connection is made.            ??8U  ?   ????           FB_SOCKETRECEIVE           fbAdsReadEx       <    ( PORT := AMSPORT_TCPIPSRV, IDXGRP :=TCPADS_IGR_RECVBYHDL )              	   T_AmsPort           ?         	   ADSREADEX ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       ?    Handle for the socket on which the actual connection is made.    cbLen           ??       3    Contains the max. number of bytes to be received.    pDest           ??       ;    Contains the address of the buffer for the received data.    bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??	       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??           	   nRecBytes           ??       2    Contains the number of bytes currently received.             ??8U  ?   ????           FB_SOCKETSEND        
   fbAdsWrite       >    ( PORT :=  AMSPORT_TCPIPSRV, IDXGRP := TCPADS_IGR_SENDBYHDL )              	   T_AmsPort           ?           ADSWRITE ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       ?    Handle for the socket on which the actual connection is made.    cbLen           ??       *    Contains the number of bytes to be send.    pSrc           ??       D    Contains the address of the buffer containing the data to be send.    bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??	       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????           FB_SOCKETUDPADDMULTICASTADDRESS        
   fbAdsWrite       E    ( PORT := AMSPORT_TCPIPSRV, IDXGRP :=TCPADS_IGR_MULTICAST_ADDBYHDL )              	   T_AmsPort           ?           ADSWRITE ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       )    udp socket to add multicast address  to.   sMulticastAddr               ??          Multicast address to add   bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????           FB_SOCKETUDPCREATE        
   fbAdsRdWrt       ^    ( PORT := AMSPORT_TCPIPSRV, IDXGRP :=TCPADS_IGR_CONLIST, IDXOFFS :=TCPADSCONLST_IOF_UDPBIND )              	   T_AmsPort           ?                ADSRDWRT ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              request                ST_SockAddr ` ??              response                ST_TcIpConnSvrResponse ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system
   sLocalHost               ??       N    Local address. String containing an (Ipv4) Internet Protocol dotted address. 
   nLocalPort           ??	       $    Local Internet Protocol (IP) port.    bExecute            ??
       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??              hSocket              	   T_HSOCKET   ??       ?   This returned value is a handle for the bind (reserved) socket.            ??8U  ?   ????            FB_SOCKETUDPDROPMULTICASTADDRESS        
   fbAdsWrite       F    ( PORT := AMSPORT_TCPIPSRV, IDXGRP :=TCPADS_IGR_MULTICAST_DROPBYHDL )              	   T_AmsPort         	  ?           ADSWRITE ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       .    udp socket to remove multicast address  from.   sMulticastAddr               ??          Multicast address to remove   bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????           FB_SOCKETUDPRECEIVEFROM           fbAdsReadEx       @    ( PORT := AMSPORT_TCPIPSRV, IDXGRP :=TCPADS_IGR_RECVFROMBYHDL )              	   T_AmsPort           ?         	   ADSREADEX ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              buffer                ST_TcIpConnSvrUdpBuffer ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       ?    Handle for the socket on which the actual connection is made.    cbLen           ??       3    Contains the max. number of bytes to be received.    pDest           ??       ;    Contains the address of the buffer for the received data.    bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??	       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??              sRemoteHost               ??       p    Remote address from which the data was received. String containing an (Ipv4) Internet Protocol dotted address.    nRemotePort           ??       G    Remote Internet Protocol (IP) port  from which the data was received. 	   nRecBytes           ??       2    Contains the number of bytes currently received.             ??8U  ?   ????           FB_SOCKETUDPSENDTO        
   fbAdsWrite       @    ( PORT :=  AMSPORT_TCPIPSRV, IDXGRP := TCPADS_IGR_SENDTOBYHDL )              	   T_AmsPort           ?           ADSWRITE ` ??              fbRTrig                 R_TRIG ` ??              nStep         ` ??              buffer                ST_TcIpConnSvrUdpBuffer ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpserver.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       ?    Handle for the socket on which the actual connection is made.    sRemoteHost               ??       d    Remote address of the target socket. String containing an (Ipv4) Internet Protocol dotted address.    nRemotePort           ??       :    Remote Internet Protocol (IP) port of the target socket.    cbLen           ??       *    Contains the number of bytes to be send.    pSrc           ??	       D    Contains the address of the buffer containing the data to be send.    bExecute            ??
       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????    ?   c:\documents and settings\administrator\desktop\hobbit\programm\libs\TcSocketHelper.lib @                                                                                          F_CREATESERVERHND            	   sSrvNetID           ''    
   T_AmsNetID   ??       Z    The network address of the TcpIpServer.exe. If empty string=>server runs on local system 
   sLocalHost               ??	       W    Local (server) address. String containing an (Ipv4) Internet Protocol dotted address. 
   nLocalPort           ??
       -    Local (server) Internet Protocol (IP) port.    nMode          ??           Listen mode flags    bEnable           ??       -    TRUE opens and FALSE closes listener socket       F_CreateServerHnd                                hServer                           	   T_HSERVER  ??           TCP/IP plc server handle         ??8U  ?   ????           F_GETVERSIONTCSOCKETHELPER               nVersionElement           ??                 F_GetVersionTcSocketHelper                                     ??8U  ?    ????           FB_CLIENTSERVERCONNECTION           eStep               E_ConnEstablishState ` ??           Internal state 	   fbConnect                            FB_SocketConnect ` ??              fbClose        
                FB_SocketClose ` ??              timer           ( PT := T#0s )                TON ` ??       [    This timer specifies when open is retried. At the first time open is retried immediatelly.   fallingEdge                 F_TRIG ` ??           
   bConnected          ` ??           Internal flag    sHSocket    Q       Q  ` ??              	   sSrvNetID           ''    
   T_AmsNetID   ??       Z    The network address of the TcpIpServer.exe. If empty string=>server runs on local system    nMode           ??           OR CONNECT_MODE_ENABLEDBG    sRemoteHost               ??       X    Remote (server) address. String containing an (Ipv4) Internet Protocol dotted address.    nRemotePort           ??       .    Remote (server) Internet Protocol (IP) port.    bEnable            ??       4    TRUE = connect, FALSE = disconnect or don't connect
   tReconnect    ȯ     ??	       /    This timer specifies when connect is retried.       bBusy            ??              bError            ??              nErrId           ??              hSocket              	   T_HSOCKET   ??           Socket handle    eState           eSOCKET_DISCONNECTED       E_SocketConnectionState   ??           Connection state             ??8U  ?    ????           FB_SERVERCLIENTCONNECTION     	   
   risingEdge                 R_TRIG ` ??              fallingEdge                 F_TRIG ` ??              timer                    TON ` ??       -    This timer specifies when accept is retried    timer2                    TON ` ??       -    This timer specifies when listen is retried 
   bConnected          ` ??       7    Internal flag, TRUE = Connection successfull accepted 
   bListening          ` ??       T    Internal flat, TRUE = Listener socket is opened, FALSE = Listener socket is closed    eStep               E_ConnEstablishState ` ??           Internal state    pLocked         ` ??              sHSocket    Q       Q  ` ??                 eMode           eACCEPT_ALL       E_SocketAcceptMode   ??           Connection accept flags    sRemoteHost               ??       X    Remote (client) address. String containing an (Ipv4) Internet Protocol dotted address.    nRemotePort           ??	       .    Remote (client) Internet Protocol (IP) port.    bEnable            ??
       $    TRUE = connect, FALSE = disconnect.
   tReconnect    ?     ??       .    This timer specifies when accept is retried.       bBusy            ??              bError            ??              nErrID           ??              hSocket              	   T_HSOCKET   ??           Socket handle    eState           eSOCKET_DISCONNECTED       E_SocketConnectionState   ??           Connection state       hServer                           	   T_HSERVER  ??           TCP/IP plc server handle         ??8U  ?    ????           FB_SOCKETLISTENEX           nStep         ` ??           
   fbAdsRdWrt                            ADSRDWRT ` ??           
   RisingEdge                 R_TRIG ` ??              request                ST_SockAddr ` ??              response                ST_TcIpConnSvrResponse ` ??           
   fbCloseAll        	               FB_SocketCloseAll ` ??              TCPADSCONLST_IOF_GETHNDLBYADDR        ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcpIpServer.exe. If empty string=>server runs on local system
   sLocalHost               ??       W    Local (server) address. String containing an (Ipv4) Internet Protocol dotted address. 
   nLocalPort           ??       -    Local (server) Internet Protocol (IP) port.    nMode          ??              bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??           	   hListener              	   T_HSOCKET   ??       _   This returned value is a handle for the listener socket on which the actual connection is made.            ??8U  ?   ????           FB_SOCKETRECEIVEEX        	   fbReceive                           FB_SocketReceive ` ??           
   RisingEdge                 R_TRIG ` ??              timer                    FB_ThrottleTimer ` ??              nStep         ` ??              	   sSrvNetId           ''    
   T_AmsNetId   ??       Y    The network address of the TcIpConnSvr.exe. If empty string=>server runs on local system   hSocket              	   T_HSOCKET   ??       ?    Handle for the socket on which the actual connection is made.    cbLen           ??       3    Contains the max. number of bytes to be received.    pDest           ??       ;    Contains the address OF the buffer FOR the received data.    bExecute            ??       F    FUNCTION block execution is triggered BY a rising edge AT this input.   tTimeout    ?     ??	       3    States the time before the function is cancelled.    throttleTimes       f    T#0s, T#10ms, T#20ms, T#40ms, T#60ms, T#80ms, T#100ms, T#200ms, T#400ms, T#600ms, T#800ms, T#1s, T#2s       T_ThrottleTimes   ??
                 bBusy            ??              bError            ??              nErrId           ??           	   nRecBytes           ??       2    Contains the number of bytes currently received.             ??8U  ?   ????           FB_THROTTLETIMER           timer                    TON ` ??              selector         ` ??                 bIn            ??       ?    Starts timer with rising edge, resets timer with falling edge    tT               T_ThrottleTimes   ??                 bOut            ??              tElapsed           ??	                       ??8U  ?    ????           HSOCKET_TO_STRING               hSocket              	   T_HSOCKET   ??                 HSOCKET_TO_STRING    Q       Q                              ??8U  ?    ????    ?   c:\documents and settings\administrator\desktop\hobbit\programm\libs\Hella.Automation.Tools-00.lib @                                                                                          F_BITARR_TO_USINT           byValue            ??                 bBitArr   	                          ??          Eingabewert      F_BitArr_To_USINT                                     ??8U  ?    ????           F_DEG_TO_RAD               angle                        ??                 F_DEG_TO_RAD                                                  ??8U  ?    ????           F_INBORDERS_INT               iVar           ??          Eingabewert   iMin           ??          Minimalwert   iMax           ??          Maximalwert      F_InBorders_INT                                      ??8U  ?    ????           F_INBORDERS_REAL               rVar            ??          Eingabewert   rMin            ??          Minimalwert   rMax            ??          Maximalwert      F_InBorders_REAL                                      ??8U  ?    ????           F_INCH_TO_MM               rIN            ??       
   Inch Input      F_Inch_To_mm                                      ??8U  ?    ????           F_LINFKT               X            ??          Eingangswert   X1            ??          Minimaler Eingangswert   X2            ??          Maximaler Eingangswert   Y1            ??          Minimaler Ausgangswert   Y2            ??          Maximaler Ausgangswert      F_LinFkt                                      ??8U  ?    ????           F_MM_TO_INCH               rIN            ??          mm Input      F_mm_To_Inch                                      ??8U  ?    ????           F_RAD_TO_DEG               angle                        ??                 F_RAD_TO_DEG                                                  ??8U  ?    ????           F_RESULTRESET                   F_ResultReset                                result                	   ST_Result  ??                   ??8U  ?    ????           F_ROUNDREAL           iValue            ??           
   rDigIntern             ??                 rVar            ??          Eingabewert   iDigits           ??          Kommastellen      F_RoundREAL                                      ??8U  ?    ????           F_SPLITSTRING           _pos            ??
              _i            ??              _pt     Q       Q          ??              _length            ??                 p           ??           pointer to target array    code               T_MaxString   ??           	   delimiter    Q      /Q    ??              tarElCnt           ??           count of items in array    sizeLine           ??       4    size of each string in array. normally 80 character      F_SplitString               	   ST_Result                             ??8U  ?    ????           F_USINT_TO_BITARR           byValue            ??           	   Index7001                            nIn           ??          Eingabewert 0- 255      F_USINT_To_BitArr   	                                                    ??8U  ?    ????           FB_BLINK           TON1                    TON    ??
              TOF1                    TOF    ??                 bIn            ??          Freigabe Blinker   tTime    ?     ??          Blink Interval      bOut            ??                       ??8U  ?    ????           FB_CHECKPARITY           bInBit   	                           ??	              iByte   	                          ??
              i            ??           
   iParityVal            ??              iParityModulo            ??              	   CheckByte           ??       )   Zu ?berpr?fendes Byte f?r Parit?tspr?fung   
   bParityBit            ??          Parit?tsbit            ??8U  ?    ????           FB_CLEARFILEBYTIME        
   _fbGetTime                 GETSYSTEMTIME    ??          timestamp reader   _FindFileList                                FB_EnumFindFileList    ??          list Files in directory   _arrFileList   	  ?                         ST_FindFileEntry            ??       '   internal storage for files in directory   _nStepClear            ??          step counter for file-deletion   _CurTime            ??          storage for current time   _CreationTime            ??       !   storage for current creation time   _delTime            ??          time of file to delete   _FileDelete        
                FB_FileDelete    ??          fb for file-deletion   _TON                    TON    ??       0   timer to wait for next check-> hardcoded 6 hours	   _countDel            ??       '   counter to loop through files in folder   _Path    Q       Q     ??           	   _FileTime             
   T_FileTime    ??       "   temporary storage for current time   i            ??          loop counter   	   sPathname    Q       Q    ??              sNetID    Q       Q    ??              nStorageTime           ??                 nErrId           ??              bError            ??	                       ??8U  ?    ????        
   FB_LOGFILE           _buffer   	  ?                 	   LogStruct         ` ??          messagebuffer   _counter         ` ??       #   counts number of messages in buffer   _Step         ` ??          step counter
   _fbGetTime                 GETSYSTEMTIME ` ??          timestamp reader	   _FileTime             
   T_FileTime ` ??       "   temporary storage for current time	   _FileOpen                             FB_FileOpen ` ??       %   to open file and create a file-handle
   _LogString               T_MaxString ` ??       &   temporary storage for message to write
   _FileClose                      FB_FileClose ` ??          close file again
   _WriteLine        	               FB_FilePuts ` ??          Write line to file   i         ` ??          loop counter	   _Pathname    Q       Q  ` ??       4   internal storage for pathname, necessary to add date   _nExtension         ` ??       (   position where the file-extension begins   _datum    Q       Q  ` ??          current date added to file-name   _Path    Q       Q  ` ??          path to search for files   _sTmp    Q       Q  ` ??              sTest    Q       Q  ` ??               fbRTC                      FB_RTC ` ??!                 sNetID    Q       Q    ??       ?   Net-ID der Steuerung auf der das Logging ausgef?hrt werden soll	   sPathname    Q       Q    ??       ~   Pfad des Logfiles Z.B. C:\Windows\temp\logging.txt. Beim Logging wird automatisch das Datum vor der Dateierweiterung eingef?gt   sMessage               T_MaxString   ??          Meldung die geloggt werden soll   nStoragetime          ??       c   Behaltezeit in Tagen -> ALLE Dateien die ?lter sind werden automatisch aus dem Log-Ordner gel?schen      bBufferOverrun            ??       9   der interne Buffer f?r die Log-Meldungen ist aufgebraucht   bError            ??          Fehler beim Schreiben   nErrId           ??          Fehlernummer            ??8U  ?    ????           FB_RTC           TON_Sync                    TON    ??              fbRTC                             RTC_EX2    ??          real-time clock   SyncTime                   
   NT_GetTime    ??              FMN_Sync                 F_TRIG    ??                 sNetID    Q       Q    ??           	   tSyncTime    '     ??              
   bValid_RTC            ??              timeRTC                   
   TimeStruct   ??                       ??8U  ?    ????        
   FB_STOPUHR           fbTimer                    TON    ??
              fbRtrigStart                 R_TRIG    ??              fbFtrigStart                 F_TRIG    ??                 bEnable            ??       O   Die positive Flanke setzt die Zeit zur?ck auf 0 und startet die Zeit wieder neu   bReset            ??          Zeit r?cksetzen      tET           ??       S   Gibt die Zeit an die seit der positiven Flanke von bEnable vergangen ist maximal 1h            ??8U  ?    ????           FB_TAKT           TON1                    TON    ??              TOF1                    TOF    ??              FMN1                 F_TRIG    ??              FMP1                 R_TRIG    ??                 bIn            ??          Starten des Taktes   tOnTime    ?     ??       $   ZEit f?r die das Signal auf TRUE ist   tOffTime    ?     ??       %   Zeit f?r die das Signal auf FALSE ist      bOut            ??          rechteck Ausgangssignal
   bOut_RTrig            ??          Steigende Flanke des Signals
   bOut_FTrig            ??          Fallende Flanke des Signals            ??8U  ?    ????           FB_TIMER           fbTimer                    TON    ??	                 Reset            ??          Zeit r?cksetzen      tET           ??           abgelaufene zeit             ??8U  ?    ????    o   C:\TWINCAT\PLC\LIB\TcSUPS.lib @                                                                                          F_GETVERSIONTCSUPS               nVersionElement           ??       d   
	Possible nVersionElement parameter:
	1	:	major number
	2	:	minor number
	3	:	revision number
      F_GetVersionTcSUPS                                     ?i?K  ?    ????           FB_NT_QUICKSHUTDOWN        
   ADSWRTCTL1                       	   ADSWRTCTL ` ??           
   RisingEdge                 R_TRIG ` ??              DELAY    ???? ` ??                 NETID            
   T_AmsNetId   ??              START            ??              TMOUT    ?     ??                 BUSY            ??              ERR            ??	              ERRID           ??
                       ?i?K  ?   ????           FB_S_UPS           fbWritePersistentData        	               FB_WritePersistentData ` ??              fbNT_QuickShutdown        	               FB_NT_QuickShutdown ` ??              dwTemp         ` ??              WaitForOffTimer                    TON ` ??              bFirstCycle         ` ??              bPowerOKInFirstCycle          ` ??                 sNetID           ''    
   T_AmsNetId   ??           '' = local netid    iPLCPort    !     ??       0    PLC Runtime System for writing persistent data    iUPSPort    ?     ??       5    Port for reading Power State of UPS, dafault 16#4A8    tTimeout    ?     ??           ADS Timeout    eUpsMode           eSUPS_WrPersistData_Shutdown       E_S_UPS_Mode   ??       8    UPS mode (w/wo writing persistent data, w/wo shutdown)    ePersistentMode           SPDM_2PASS       E_PersistentMode   ??       "    mode for writing persistent data    tRecoverTime    '     ??	       l    ON time to recover from short power failure in mode eSUPS_WrPersistData_NoShutdown/eSUPS_CheckPowerStatus        bPowerFailDetect            ??       %    TRUE while powerfailure is detected    eState               E_S_UPS_State   ??           current ups state             ?i?K  ?   ????    u   C:\TWINCAT\PLC\LIB\TcXmlDataSrv.lib @                                                                                          F_GETVERSIONTCXMLDATASRV               nVersionElement           ??                 F_GetVersionTcXmlDataSrv                                     ??8U  ?    ????           FB_SYMNAMEBYADDR        	   fbAdsRWEx                          
   ADSRDWRTEX    ??           
   writeEntry                ST_SymAddrInfo    ??           
   RisingEdge                 R_TRIG    ??                 pSymAddr           ??           	   cbSymSize           ??              bExecute            ??       F    Function block execution is triggered by a rising edge at this input.   tTimeout    ?     ??       3    States the time before the function is cancelled.       bBusy            ??	              bError            ??
              nErrId           ??              sSymName               T_MaxString   ??                       ??8U  ?    ????           FB_XMLSRVREAD     
      handle            ??              stOpenEntry        
                ST_XmlSrvOpenEntry    ??              cbOpenEntry            ??              pCur            ??              fbReadSymName                         FB_SymNameByAddr    ??           	   fbAdsRWEx   	                                
   ADSRDWRTEX            ??              cbToRead            ??              cbLeft            ??              nStep            ??           
   RisingEdge                 R_TRIG    ??           	      sNetId            
   T_AmsNetId   ??           ams net id    ePath           PATH_GENERIC    
   E_OpenPath   ??           Default: Open generic file    nMode           ??              pSymAddr           ??           	   cbSymSize           ??           	   sFilePath               T_MaxString   ??              sXPath               T_MaxString   ??	              bExecute            ??
       F    Function block execution is triggered by a rising edge at this input.   tTimeout    `?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????           FB_XMLSRVREADBYNAME           handle            ??              stOpenEntry        
                ST_XmlSrvOpenEntry    ??              cbOpenEntry            ??              pCur            ??           	   fbAdsRWEx                          
   ADSRDWRTEX    ??              nStep            ??           
   RisingEdge                 R_TRIG    ??                 sNetId            
   T_AmsNetId   ??           ams net id    ePath           PATH_GENERIC    
   E_OpenPath   ??           Default: Open generic file    nMode           ??              sSymName               T_MaxString   ??           	   sFilePath               T_MaxString   ??              sXPath               T_MaxString   ??              bExecute            ??	       F    Function block execution is triggered by a rising edge at this input.   tTimeout    `?     ??
       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????           FB_XMLSRVWRITE     
      handle            ??              stOpenEntry        
                ST_XmlSrvOpenEntry    ??              cbOpenEntry            ??              pCur            ??              fbReadSymName                         FB_SymNameByAddr    ??           	   fbAdsRWEx   	                                
   ADSRDWRTEX            ??           	   cbToWrite            ??              cbLeft            ??              nStep            ??           
   RisingEdge                 R_TRIG    ??           	      sNetId            
   T_AmsNetId   ??           ams net id    ePath           PATH_GENERIC    
   E_OpenPath   ??           Default: Open generic file    nMode           ??              pSymAddr           ??           	   cbSymSize           ??           	   sFilePath               T_MaxString   ??              sXPath               T_MaxString   ??	              bExecute            ??
       F    Function block execution is triggered by a rising edge at this input.   tTimeout    `?     ??       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????           FB_XMLSRVWRITEBYNAME           handle            ??              stOpenEntry        
                ST_XmlSrvOpenEntry    ??              cbOpenEntry            ??              pCur            ??           	   fbAdsRWEx                          
   ADSRDWRTEX    ??              nStep            ??           
   RisingEdge                 R_TRIG    ??                 sNetId            
   T_AmsNetId   ??           ams net id    ePath           PATH_GENERIC    
   E_OpenPath   ??           Default: Open generic file    nMode           ??              sSymName               T_MaxString   ??           	   sFilePath               T_MaxString   ??              sXPath               T_MaxString   ??              bExecute            ??	       F    Function block execution is triggered by a rising edge at this input.   tTimeout    `?     ??
       3    States the time before the function is cancelled.       bBusy            ??              bError            ??              nErrId           ??                       ??8U  ?   ????    R    @                                                                                V          F_ADSLOGERROR               sPrefix               N        O    Debug message prefix string (allows the identification of log message source)    nErrID           N            Error code       F_ADSLOGERROR                                     ^M7U  @    ????           F_ADSLOGSTRING               sPrefix               O        O    Debug message prefix string (allows the identification of log message source)    sMsg               T_MaxString   O            Message string       F_ADSLOGSTRING                                     ^M7U  @    ????           F_CALCVELOCITY           PosDiff   	                                      : 
           	   Index7001                            Position   	                                     :               Velocity                        :                  F_CalcVelocity   	                                                         Axis    	                    Axis_Ref          :                    ^M7U  @    ????           F_CHECKATCANDLEPOS           PositionWindow    ????????   0.8????????    *                     F_CheckAtCandlePos                                Axis    	                    Axis_Ref          *                   ^M7U  @    ????           F_CHECKATFINALGRASPFROMFLOORPOS           PositionWindow          ??   0.5      ??    %                      F_CheckAtFinalGraspFromFloorPos                                Axis    	                    Axis_Ref          %                    ^M7U  @    ????           F_CHECKATFINALGRASPTRAYPOSITION           PositionWindow          ??   1      ??    &                      F_CheckAtFinalGraspTrayPosition                                Axis    	                    Axis_Ref          &                    ^M7U  @    ????        $   F_CHECKATFINALGRASPTURNTABLEPOSITION           PositionWindow          ??   1      ??    '                   $   F_CheckAtFinalGraspTurntablePosition                                Axis    	                    Axis_Ref          '                    ^M7U  @    ????        %   F_CHECKATFINALPUTOBJECTTOTRAYPOSITION           PositionWindow          ??   1      ??    (                   %   F_CheckAtFinalPutObjecttoTrayPosition                                Axis    	                    Axis_Ref          (                    ^M7U  @    ????        $   F_CHECKATFINALSTORETURNTABLEPOSITION           PositionWindow          ??   1      ??    )                   $   F_CheckAtFinalStoreTurntablePosition                                Axis    	                    Axis_Ref          )                    ^M7U  @    ????           F_CHECKATHOMEPOSITION           PositionWindow    ????????   0.8????????    *                      F_CheckAtHomePosition                                Axis    	                    Axis_Ref          *                    ^M7U  @    ????           F_CHECKATLEARNINGPOSITION           PositionWindow          ??   1      ??    +                      F_CheckAtLearningPosition                                Axis    	                    Axis_Ref          +                    ^M7U  @    ????           F_CHECKATMOVEARMOUTPOS           PositionWindow           @   2       @    ,                      F_CheckAtMoveArmOutPos                                Axis    	                    Axis_Ref          ,                    ^M7U  @    ????           F_CHECKATMOVEARMOUTPOS2           PositionWindow           @   2       @    %                     F_CheckAtMoveArmOutPos2                                Axis    	                    Axis_Ref          %                   ^M7U  @    ????           F_CHECKATMOVEARMOUTPOS3           PositionWindow           @   2       @    $                     F_CheckAtMoveArmOutPos3                                Axis    	                    Axis_Ref          $                   ^M7U  @    ????           F_CHECKATPREGRASPFROMFLOORPOS           PositionWindow           @   2       @    -                      F_CheckAtPreGraspFromFloorPos                                Axis    	                    Axis_Ref          -                    ^M7U  @    ????           F_CHECKATPREGRASPFROMTABLEPOS           PositionWindow           @   2       @    #                     F_CheckAtPreGraspFromTablePos                                Axis    	                    Axis_Ref          #                   ^M7U  @    ????           F_CHECKATPREGRASPTRAYPOS1           PositionWindow           @   2       @    .                      F_CheckAtPregraspTrayPos1                                Axis    	                    Axis_Ref          .                    ^M7U  @    ????           F_CHECKATPREGRASPTRAYPOS2           PositionWindow           @   2       @    /                      F_CheckAtPregraspTrayPos2                                Axis    	                    Axis_Ref          /                    ^M7U  @    ????           F_CHECKATPREGRASPTRAYPOS3           PositionWindow           @   2       @    0                      F_CheckAtPregraspTrayPos3                                Axis    	                    Axis_Ref          0                    ^M7U  @    ????        #   F_CHECKATPREGRASPTURNTABLEPOSITION1           PositionWindow          ??   1      ??    1                   #   F_CheckAtPregraspTurntablePosition1                                Axis    	                    Axis_Ref          1                    ^M7U  @    ????        #   F_CHECKATPREGRASPTURNTABLEPOSITION2           PositionWindow          ??   1      ??    2                   #   F_CheckAtPregraspTurntablePosition2                                Axis    	                    Axis_Ref          2                    ^M7U  @    ????           F_CHECKATPREPUTOBJECTTOTRAYPOS1           PositionWindow           @   2       @    3                      F_CheckAtPrePutObjectToTrayPos1                                Axis    	                    Axis_Ref          3                    ^M7U  @    ????           F_CHECKATPREPUTOBJECTTOTRAYPOS2           PositionWindow           @   2       @    4                      F_CheckAtPrePutObjectToTrayPos2                                Axis    	                    Axis_Ref          4                    ^M7U  @    ????           F_CHECKATPREPUTOBJECTTOTRAYPOS3           PositionWindow           @   2       @    5                      F_CheckAtPrePutObjectToTrayPos3                                Axis    	                    Axis_Ref          5                    ^M7U  @    ????        #   F_CHECKATPRESTORETURNTABLEPOSITION1           PositionWindow          ??   1      ??    6                   #   F_CheckAtPreStoreTurntablePosition1                                Axis    	                    Axis_Ref          6                    ^M7U  @    ????        #   F_CHECKATPRESTORETURNTABLEPOSITION2           PositionWindow          ??   1      ??    7                   #   F_CheckAtPreStoreTurntablePosition2                                Axis    	                    Axis_Ref          7                    ^M7U  @    ????           F_CHECKATTURNTABLECCWPOSITION           PositionWindow          ??   1      ??    8                      F_CheckAtTurntableCCWPosition                                Axis    	                    Axis_Ref          8                    ^M7U  @    ????           F_CHECKATTURNTABLECWPOSITION           PositionWindow          ??   1      ??    9                      F_CheckAtTurntableCWPosition                                Axis    	                    Axis_Ref          9                    ^M7U  @    ????           F_CHECKAXISHAVESTOPPED                   F_CheckAxisHaveStopped                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKAXISINPOSITIONWINDOW               SetPosition            0              PositionWindow            0                 F_CheckAxisInPositionWindow                                Axis                Axis_Ref  0                   ^M7U  @    ????           F_CHECKIFALLAXISHOMED                   F_CheckIfAllAxisHomed                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFAXISDISABLED                   F_CheckIfAxisDisabled                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFAXISENABLED                   F_CheckIfAxisEnabled                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFAXISHASERROR                   F_CheckIfAxisHasError                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFAXISHASJOB                   F_CheckIfAxisHasJob                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFAXISISMOVING                   F_CheckIfAxisIsMoving                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFAXISNOTMOVING                   F_CheckIfAxisNotMoving                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFHOMINGISBUSY                   F_CheckIfHomingIsBusy                                Axis    	                    Axis_Ref                              ^M7U  @    ????           F_CHECKIFINPOSAREA                   F_CheckIfInPosArea                                Axis    	                    Axis_Ref                               ^M7U  @    ????           F_CHECKIFINTARGETPOS           i            ! 	                     F_CheckIfInTargetPos                                Axis    	                    Axis_Ref          !                    ^M7U  @    ????           F_CHECKIFINTARGETPOSEX           i            /
              	   fPosition   	                                     /              PositionWindow   	                         /                 F_CheckIfInTargetPosEx                                Axis    	                    Axis_Ref          /                   ^M7U  @    ????           F_CHECKIFNEXTPOSISOUTOFPOSAREA               NextPosition   	                                     "               PositionWindow   	                                     "                  F_CheckIfNextPosIsOutOfPosArea                                Axis    	                    Axis_Ref          "                    ^M7U  @    ????           F_CHECKIFSOFTLIMITMAX                   F_CheckIfSoftLimitMax                                Axis    	                    Axis_Ref          #                    ^M7U  @    ????           F_CHECKIFSOFTLIMITMIN                   F_CheckIfSoftLimitMin                                Axis    	                    Axis_Ref          $                    ^M7U  @    ????           F_CHECKJOINTSIDE                   F_CheckJointSide               E_JointSide                             ^M7U  @    ????           F_CHECKSTARTREF                   F_CheckStartRef            
   E_Decision                             ?[7U  @    ????           F_CUTSTRING           _str               T_MaxString    I               i            I               iCutPos   	                          I 	           	   Index7001                            IN               T_MaxString   I            
   sDelimiter    Q      /Q    I                  F_CutString   	          Q       Q                                      ^M7U  @    ????           F_MERGESENDARMSTATE           _str               T_MaxString    J 	              i            J 
                 Variable    Q       Q    J               sStatus               T_MaxString   J            
   STCommands        +                                                 ST_Commands   J            
   sDelimiter    Q       Q    J                  F_MergeSendArmState               T_MaxString                             ^M7U  @    ????           F_MERGESENDDATA           _str               T_MaxString    D 	              _strData               T_MaxString    D 
              i            D                  CMD    Q       Q    D               Variable    Q       Q    D               Value            D            
   sDelimiter    Q       Q    D                  F_MergeSendData               T_MaxString                             ^M7U  @    ????           F_MERGESTRING           _Merge               T_MaxString    K               i            K 	                 IN               T_MaxString   K               sStatus               T_MaxString   K            
   sDelimiter    Q      ;Q    K                  F_MergeString               T_MaxString                             ^M7U  @    ????           F_MERGESTRINGONEVALUE           _Merge               T_MaxString    L 	              i            L 
                 IN               T_MaxString   L               sStatus               T_MaxString   L               sValue               T_MaxString   L            
   sDelimiter    Q      ;Q    L                  F_MergeStringOneValue               T_MaxString                             ^M7U  @    ????           F_MERGESTRINGVALUE           _Merge               T_MaxString    M               _Values   	                     T_MaxString            M               i            M            	   Index7001                      	      IN   	          Q       Q            M               sStatus               T_MaxString   M            
   sDelimiter    Q      ;Q    M               Value1    Q       Q    M               Value2    Q       Q    M               Value3    Q       Q    M               Value4    Q       Q    M 	              Value5    Q       Q    M 
              Value6    Q       Q    M                  F_MergeStringValue               T_MaxString                             ^M7U  @    ????           F_RESETCOMMANDS           i            ;                      F_ResetCommands                                      ^M7U  @    ????           F_RESETDIALOG                   F_ResetDialog                                      ?U7U  @    ????           F_RESETSTARTREFDIALOG                   F_ResetStartRefDialog                                      ?U7U  @    ????           F_SETHOMINGDISABLE                   F_SetHomingDisable                                STInstances                 ST_Instances  <                    ^M7U  @    ????           F_SETHOMINGENABLE                   F_SetHomingEnable                                STInstances                 ST_Instances  =                    ^M7U  @    ????           F_SETREFDIALOG               nJointNo           b                 F_SetRefDialog                                      PU7U  @    ????           F_SETSTARTREFDIALOG                   F_SetStartRefDialog                                      VU7U  @    ????           F_SWITCHSTEP               pStepCounter                 ?              NextStep           ?           	   pLastStep                 ?                 F_SwitchStep                                      ^M7U  @    ????           FB_CHECKSTATE           _ArmHomedStateOld              	              _ArmIsMovingStateOld              
              _ArmHasStoppedOld                            _ArmIsDisabledStateOld                            _ArmHasErrorStateOld                            _ArmInPosAreaStateOld                            _ArmInTargetPosStateOld                            _ArmSoftLimitMaxStateOld                            _ArmSoftLimitMinStateOld                            _GripperIsClosedOld                           _AtHomePosStateOld                            _AtLearningPosStateOld                            _AtTurntablePosOld                            _AtTrayPosOld                            _AtPreGraspFromFloorPosOld                            _AtPreGraspFromTablePosOld                            _AtCCWPosOld                            _AtCWPosOld                            _EmergencyButtonStateOld                     ?   
	_AtPreGraspPosOld		:	BOOL:=FALSE;
	_AtFinalGraspPosOld		:	BOOL:=FALSE;
	_AtTurntableCWPosOld	:	BOOL:=FALSE;
	_AtTurntableCCWPosOld	:	BOOL:=FALSE;
	   PosWindow   	                     6(0.8)      ??L?   0.8     $              RTRIG                 R_TRIG     %              FMP_CandlePos                 r_trig     &              FMN_CandlePos                 F_TRIG     '                         Axis    	                    Axis_Ref                              ^M7U  @    ????           FB_CLIENTAPPLICATION        	   sToServer           ''       T_MaxString    >               sFromServer           ''       T_MaxString    >               fbClient                                       FB_LocalClient    >        -    Client data exchange control function block    tx        	               FB_FrameStringFifo    >        	    Tx fifo    rx        	               FB_FrameStringFifo    >        	    Rx fifo    errors                      FB_ProtErrorFifo    >            Error fifo    sndTimer                    TON    >               rcvTimer                    TON    >               firstConnect            >               state            >            application state    rxString   	          Q       Q             >               iState    ?      >            
   fbLogState                                   
   FB_LogFile    >               FMP_CandlePos                 R_TRIG    >               FMN_CandlePos                 F_TRIG    >               sPath    Q       Q     >                  bDbg            >        /    TRUE => Enable debug output, FALSE => Disable    sRemoteHost               >               nRemotePort           >               bEnable            >                  eState           eSOCKET_DISCONNECTED       E_SocketConnectionState   > 
           TCP/IP connection state             ^M7U  @    ????           FB_FRAMESTRINGFIFO           buffer   	  ?  ????                 P            Internal buffer memory    fbBuffer           (bOverwrite := FALSE)	                    FB_StringRingBuffer    P        :    Basic (lower level) string buffer control function block       sDesc          Unknown    P        T    Debug message description string (allows the identification of log message source)    bDbg            P        /    TRUE => Enable debug output, FALSE => Disable    putValue           ''       T_MaxString   P        %    String to add (write) to the buffer       bOk            P 	       T    TRUE = New entry added or removed succesfully, FALSE = Fifo overflow or fifo empty    getValue           ''       T_MaxString   P 
       #    String removed (read) from buffer    nCount           P            Number of fifo entries    cbFree           P            Free buffer space             ^M7U  @   ????           FB_HOME     
      eHoming          99       E_STATE_HOMING                IDLE    LastHomingState               E_STATE_HOMING                   i                           RStart                 R_TRIG                   RStop                 R_TRIG                   MoveRelativeAxis4                                 MC_MoveRelative                   MoveRelativeAxis6                                 MC_MoveRelative                   SetPositionAxis4                            MC_SetPosition                    SetPositionAxis6                            MC_SetPosition     !              fbMoveToZero                    FB_MoveToZero     "                 bExecute                              bHomingDone                           bHomingBusy                           bReferenceRestarted                           bHasBeenStopped                           bError                              Axis    	                    Axis_Ref           
              STInstances                 ST_Instances                      ^M7U  @    ????           FB_HOME_NEW           eHoming          99       E_STATE_HOMING    J           IDLE    LastHomingState               E_STATE_HOMING    J              i            J              RStart                 R_TRIG    J              RStop                 R_TRIG    J              MoveRelativeAxis4                                 MC_MoveRelative    J              MoveRelativeAxis6                                 MC_MoveRelative    J              SetPositionAxis4                            MC_SetPosition    J               SetPositionAxis6                            MC_SetPosition    J!              fbMoveToZero                    FB_MoveToZero    J"           
   eJointSide               E_JointSide    J#              fbHomeJoint                         FB_HomeJoint    J$           	   eDecision            
   E_Decision    J%              fbHomeDefault                                FB_Home    J&                 bExecute            J                 bHomingDone            J              bHomingBusy            J              bReferenceRestarted            J              bHasBeenStopped            J              bError            J                 Axis    	                    Axis_Ref          J
              STInstances                 ST_Instances  J                   ??7U  @    ????           FB_HOMEJOINT           eHomingJoint               E_HomingJoint    =              eLastHoming               E_HomingJoint    =           
   eJointSide               E_JointSide    =              fbSetCamSearchDirection                      FB_SetCamSearchDirection    =              fbMoveToZero                    FB_MoveToZero    =                 bExecute            =              nJointNo           =              Axis               AXIS_REF   =                 Busy            =              Error            =	              DONE            =
                       ^M7U  @    ????           FB_INSTANCES           _MoveAbsoluteExecute                            i                        	   _Velocity   	                                                  	   SetPosOpt                 ST_SetPositionOptions                   SetDestallOpt                    ST_PowerStepperStruct                
   Power_Axis   	             1                                                       MC_PowerStepper                           MoveAbsoluteAxis   	                                      MC_MoveAbsolute                           MoveJog   	             '                                             MC_Jog                           SetPosition   	                                 MC_SetPosition                        
   HomingAxis   	                                            MC_Home                           ResetHoming   	                                            MC_Home                        	   ResetAxis   	                              MC_Reset                           StopAxis   	                                        MC_Stop                           logged   	                                          k             !           
   TON_RefCam   	                         TON             "           
   bHackReset   	                           #                         Axis    	                    Axis_Ref                         STInstances                 ST_Instances                      @?7U  @    ????           FB_LEDBLINKING           i                           LastLEDState                            TP_On                   TP                   TP_Off                   TP                   FTrig                 F_TRIG                   RTrig                 R_TRIG                      bSwitch                       ?ffner Kontakt   tTime                             bLED                                    ^M7U  @    ????           FB_LOCALCLIENT        	   fbConnect           ( tReconnect := T#45s )                ȯ          FB_ClientServerConnection    C        "    create/release TCP/IP connection    fbSend                          FB_SocketSend    C            send TCP/IP data 	   fbReceive                           FB_SocketReceive    C            receive TCP/IP data    state            C            global state    tx_state            C        
    tx state    rx_state            C        
    rx state    bDisconnect             C        3    disconnect flag, if set the socket will be closed 	   pollTimer                    TON    C            
   cbReceived            C            count of received data bytes    cbRx            C             byte length of received string    rxFrame               T_MaxString    C               txFrame               T_MaxString    C               buffer   	  ?                       C            Temp. RX buffer    i            C                bAbort             C !              sID                C "                 bDbg            C 	       <    TRUE => Enable debug output, FALSE => Disable debug output    sRemoteHost          192.168.2.122    C 
           IP address of remote server    nRemotePort    ?     C            Remote server port    bEnable            C        5    TRUE => Enable/connect, FALSE => Disable/disconnect 
   tReconnect    ȯ     C        "    Try to reconnect after this time       eState           eSOCKET_DISCONNECTED       E_SocketConnectionState   C            TCP/IP connection state       tx         	               FB_FrameStringFifo  C        	    TX fifo    rx         	               FB_FrameStringFifo  C        	    RX fifo    errors                 FB_ProtErrorFifo  C            Error message fifo         ^M7U  @   ????           FB_LOCALSERVER        	   fbConnect                                  FB_ServerClientConnection    E        "    create/release TCP/IP connection    fbSend                          FB_SocketSend    E            send TCP/IP data 	   fbReceive                           FB_SocketReceive    E            receive TCP/IP data    state            E            global state    tx_state            E        
    tx state    rx_state            E        
    rx state    bDisconnect             E        3    disconnect flag, if set the socket will be closed 	   pollTimer                    TON    E            
   cbReceived            E            count of received data bytes    cbRx            E             byte length of received string    buffer   	  ?                       E            Temp. RX buffer    txFrame               T_MaxString    E               rxFrame               T_MaxString    E               i            E               bAbort             E                sID                E !                 bDbg            E        <    TRUE => Enable debug output, FALSE => Disable debug output    bEnable           E        5    TRUE => Enable/connect, FALSE => Disable/disconnect       eState           eSOCKET_DISCONNECTED       E_SocketConnectionState   E            TCP/IP connection state       hServer                           	   T_HSERVER  E            Server connection handle    tx         	               FB_FrameStringFifo  E        	    TX fifo    rx         	               FB_FrameStringFifo  E        	    RX fifo    PositionBuffer                 FB_PositionFifoBuffer  E               errors                 FB_ProtErrorFifo  E            Error message fifo         ^M7U  @   ????           FB_MOVE           eMOVE          99       E_STATE_MOVE     F       	   IDLE Mode   LastMoveState               E_STATE_MOVE     G              bStartTimer              H              T_ON1                    TON     I              RStart                 R_TRIG     J              i             L              PositionWindow   	                     6(0.8)      ??L?   0.8     M                 bExecuteMove             	              bEnableInterpolationMove             
              bEnableSingleAxisMove                           bSingleAxisIndex                                     %      bError                        
   bAtHomePos                           bAtLearningPos                           bAtPreGraspTurntablePos1                           bAtPreGraspTurntablePos2                           bAtFinalGraspTurntablePos                           bAtPreStoreTurntablePos1                           bAtPreStoreTurntablePos2                           bAtFinalStoreTurntablePos                           bAtPreGraspTrayPos1                           bAtPreGraspTrayPos2                           bAtPreGraspTrayPos3                           bAtFinalGraspTrayPos                           bAtPrePutObjectToTrayPos1                           bAtPrePutObjectToTrayPos2                           bAtPrePutObjectToTrayPos3                            bAtFinalPutObjectToTrayPos             !              bAtMoveArmOutPos             #              bAtPreGraspFromFloorPos             $              bAtFinalGraspFromFloorPos             %              bAtMoveArmOutPos2             '              bAtMoveArmOutPos3             (              bAtPreGraspFromTablePos             )              bAtTurntableCWPos             +              bAtTurntableCCWPos             ,              bAtSoftLimitMax             .              bAtSoftLimitMin             /              bInTargetPos             1              bInPositionArea             2              bHasJob             4           	   bIsMoving             5              bStandStill             6              bHasBeenStopped             7              bAxisDisabled             8           	   bMoveDone             9              bReady             :              bAtCandlePos             ;                 Axis    	                    Axis_Ref           ?              STInstances                 ST_Instances   @                   ^M7U  @    ????           FB_MOVEINTERPOLATION     	      eInterpolation           99       E_STATE_INTERPOLATION                   LastInterpolationState               E_STATE_INTERPOLATION                   _fInterpolationPosition   	                                                     i                           RStart                 R_TRIG                   RTrigInPosArea                 R_TRIG                	fbInterpolate	:	FB_Interpolate;   ReadPosition                          FB_ReadPositionFromFifo                   ReadPositionWindowValue                  FB_ReadPositionWindowValue                   _InPosAreaOld                               bExecuteInterpolation                              bInPositionArea                           bInTargetPosition                           bAtHomePosition                           bHasBeenStopped                        
   bNotMoving                           bHasJob                           bDone                           bError                              Axis    	                    Axis_Ref                         STInstances                 ST_Instances                      ^M7U  @    ????           FB_MOVETOZERO           MoveAbsoule                                 MC_MoveAbsolute    4                 bExecute            4              Axis                Axis_Ref        4                 bDone            4              bError            4              nErrID           4	                       ^M7U  @    ????           FB_POSITIONFIFOBUFFER           fbBuffer           (bOverwrite := FALSE)	                    FB_StringRingBuffer    W        :    Basic (lower level) string buffer control function block       sDesc          Unknown    W        T    Debug message description string (allows the identification of log message source)    bDbg            W        /    TRUE => Enable debug output, FALSE => Disable    putValue           ''       T_MaxString   W        %    String to add (write) to the buffer       bOk            W 	       T    TRUE = New entry added or removed succesfully, FALSE = Fifo overflow or fifo empty    getValue           ''       T_MaxString   W 
       #    String removed (read) from buffer    nCount           W            Number of fifo entries    cbFree           W            Free buffer space             ^M7U  @    ????           FB_PROTERRORFIFO           buffer   	     ????                 ^            Internal buffer memory    fbBuffer                              FB_MemRingBuffer    ^        3    Basic (lower level) buffer control function block       sDesc          Unknown    ^        P    Debug message description string (allows the identification of message source)    bDbg            ^        /    TRUE => Enable debug output, FALSE => Disable    putError           ^        '    Error code to add (write) to the fifo       bOk            ^ 	       T    TRUE = New entry added or removed succesfully, FALSE = Fifo overflow or fifo empty    getError           ^ 
       )    Error code get/removed (read) from fifo    nCount           ^            Number of fifo entries             ^M7U  @    ????           FB_READPOSITIONFROMFIFO        	   sPosition               T_MaxString                
   rxPosition   	          Q       Q                            oldPosition   	                                                     i                             bGetPosition                           fPositionWindow   	                                                    	   fPosition   	                                      
           Nex Position to Move    iNumbrOfPos                   9    Gives the actual number of Positions in the Fifo Buffer    bNoMoreData                    !    Is TRUE if no more data to read    bNewPosition                          bNextPositionIsOutOfPosArea                              Axis    	                    Axis_Ref                              ^M7U  @    ????           FB_READPOSITIONWINDOWVALUE           i                           ReadParameterAxis   	                                   MC_ReadParameter                                  PositionWindow   	                                                       Axis    	                    Axis_Ref                              ^M7U  @    ????           FB_RESET           eReset          99       E_STATE_RESET               IDLE   LastResetState               E_STATE_RESET                   i                           RStart                 R_TRIG                      bExecute                           
   bResetDone             
                 Axis    	                    Axis_Ref                         STInstances                 ST_Instances                      ^M7U  @    ????           FB_SERVERAPPLICATION           sFromClient           ''       T_MaxString    F            	   sToClient           ''       T_MaxString    F               fbServer                                      FB_LocalServer    F        *    Implements one server->client connection    tx        	               FB_FrameStringFifo    F        	    Tx fifo    rx        	               FB_FrameStringFifo    F        	    Rx fifo    errors                      FB_ProtErrorFifo    F            Error fifo    state            F               i            F               rxLength            F               rxString   	          Q       Q             F               txString   	          Q       Q             F               rxPosString   	          Q       Q             F               fbLogReceive                                   
   FB_LogFile    F            	   fbLogSend                                   
   FB_LogFile    F               sPath    Q       Q     F                   bDbg            F        /    TRUE => Enable debug output, FALSE => Disable    bEnable           F 	       -    TRUE => Enable connection, FALSE => Disable       eState           eSOCKET_DISCONNECTED       E_SocketConnectionState   F            TCP/IP connection state       hServer                           	   T_HSERVER  F            Server connection handle         ^M7U  @    ????           FB_SETCAMSEARCHDIRECTION        
   fbADSWrite                          ADSWRITE    G              _wTmp            G                 bExecute            G              nAxisID           G              eSearchDirection               E_JointSide   G                 bBusy            G              bError            G	              nErrID           G
                       ?7U  @    ????           FB_SETDEFAULHOMINGDIRECTION           step            i              oldStep            i              fbSetSearchDirection                      FB_SetCamSearchDirection    i              i            i                 bExecute            i                 bDone            i              bBusy            i              bError            i                       ??7U  @    ????           MAIN     .   	   fPosition   	                                0,0,0,0,0,0                 0              0              0              0              0              0    f           Absolute Position in DEG   fActPosition   	                          f               bProgrammReset            f            eNumerations    eSystemMode           SystemMode_Boot       E_SystemMode    f 	              eMode          800       E_STATE    f 
       :   Programm Mode: Shows in which Mode the Program actually is
   eTurntable          99       E_STATE_TURNTABLE    f           IDLE   eTray          99       E_STATE_TRAY    f           IDLE   eFloor           99       E_STATE_GRASPFLOOR    f           IDLE	   ePreFloor           99       E_STATE_PREGRASPFLOOR    f           IDLE	   ePreTable           99       E_STATE_PREGRASPTABLE    f           IDLE
   ePutObject           99       E_STATE_OBJECT_TO_TRAY    f           IDLE
   eCandlePos           E_CANDLE_IDLE       E_STATE_CANDLE    f               eHomePos           E_HOME_IDLE       E_STATE_HOME    f               LastMode               E_STATE    f           Shows Last Program Mode   LastTurntableState               E_STATE_TURNTABLE    f               LastTrayState               E_STATE_TRAY    f               LastFloorState               E_STATE_GRASPFLOOR    f               LastPreFloorState               E_STATE_PREGRASPFLOOR    f               LastPreTableState               E_STATE_PREGRASPTABLE    f               LastPutObjectState               E_STATE_OBJECT_TO_TRAY    f               LastCandleState               E_STATE_CANDLE    f               LastHomeState               E_STATE_HOME    f           LastJog				:	E_STATE_JOG;   Joint   	                    Axis_Ref            f            
   Power_Axis   	             1                                                       MC_PowerStepper            f !              MoveAbsoluteJoint   	                                      MC_MoveAbsolute            f "           
   HomingAxis   	                                            MC_Home            f #           	   ResetAxis   	                              MC_Reset            f $              TON1                    TON    f &              TON2                    TON    f '              TON3                    TON    f (              TON4                    TON    f )              TGrip                    TON    f *              RTrigDisable                 R_TRIG    f +              fbCheckState                                      FB_CheckState    f ,              fbInstances                                 FB_Instances    f -              fbHOME                                    FB_Home_new    f .              fbMOVE        2                                                        FB_Move    f /              fbMoveInterpolation                                  FB_MoveInterpolation    f 0          fbJOG				:	FB_Jog;   fbRESET                      FB_Reset    f 2              fbLedBlinking        	               FB_LedBlinking    f 3              bExecute             f 4              fbSUPS                             FB_S_UPS    f 5              i            f 6           	   TON_Retry                    ton    f 7              bSaveParams             f 8           
   fbShutDown                      NT_Shutdown    f 9                               ??7U  @    ????           PRG_DATAACCESS           FB_XmlSrvReadConfig                                    FB_XmlSrvRead    I              FB_XmlSrvWriteConfig                                    FB_XmlSrvWrite    I       F   	FB_EventSetReadXML: 	FB_EventSet;
	FB_EventSetSaveXML: 	FB_EventSet;   FE_LoadBusy                 F_TRIG    I
              FE_Save                 F_TRIG    I          FB_BlinkBak: FB_Blinktakt;   FB_XmlSrvWriteConfigBak                                    FB_XmlSrvWrite    I              pathname    e       e     I                 bLoadCfg            I              bSaveCfg            I                 bDoneLoadCfg            I              bDoneSaveCfg            I                       ^M7U  @    ????        
   SCODE_CODE               sc           e               
   SCODE_CODE                                     ^M7U  @    ????           TCPIPCLIENT           fbApplication       }    ( sRemoteHost:= REMOTE_IP_ADDRESS, nRemotePort:= CLIENTPORT, bDbg :=TRUE(* TRUE = enable debug output, FALSE = disable *)  )                               192.168.2.122 ?       FB_ClientApplication    {               bEnable            {        7    TRUE => enable client data exchange, FALSE => disable 
   fbCloseAll        	               FB_SocketCloseAll    {            	   bCloseAll            {                                ^M7U  @   ????           TCPIPSERVER        
   sLocalHost          192.168.2.190     |            Server address 
   nLocalPort    ?      |            Server port number    bEnable            |        7    TRUE => enable server data exchange, FALSE => disable    hServer                          	   T_HSERVER    |            Server connection handle    fbApplication           ( bDbg:=FALSE )                              FB_ServerApplication    | 	       $    Application (connection) instances 
   fbCloseAll        	               FB_SocketCloseAll    |            	   bCloseAll            |                                ^M7U  @   ????            
 ?  4 f   5       u   >   x   y   }   t   !  (  I  F   >  K  &   (   )   +   -   r   A   v   w      H     ?   B      J     z   Y  H   8  ????X  Z  b  c  e  i  =   <   G  ;   g     =     ( ??     K   ȫ    K   ֫    K   ??    K   ??                ?        +     ??localhost ?ژ?u   ??     `??H ?`????@?l? $? ? H? ??wPC? ????w/?w.?w??           ??          ????? x,?w?????     ??? ? ?ʆ? E? ????    ?7?d?             X? ??          ??       n? شR????? n? ??R????? -?     ,   ,                                                        K    S   C:\Documents and Settings\Administrator\Desktop\HOBBIT\Programm\bk\Hobbit_00.pro @ ^M7U[? /*BECKCONFI3*/
        !?> @   @   ?   ?     3   +            
   Standard        TCPIPServer     
   TCPIPClient            	г8U     &   ??*            VAR_GLOBAL
END_VAR
                                                                                  "   ,                  Standard
         Main();????                TCPIPServer        TCPIPServer();????                TCPIPClient
        TCPIPClient();????               ϳ8U                 $????, ? ? A?               otnBrdro           Standard K?Q	K?Q      telc00_H                         	ϳ8U     7,#3169,           VAR_CONFIG
END_VAR
                                                                                   '             , Z Z fy           Globale_Constant ^M7U	^M7U    ,}??           #  VAR_GLOBAL CONSTANT
	LOCAL_IP_ADDRESS							:	STRING(15) := '192.168.2.190';	(* CX HOBBIT*)
	REMOTE_IP_ADDRESS						:	STRING(15) := '192.168.2.122';	(* XPC *)

	SERVERPORT								:	UDINT := 5010;
	CLIENTPORT								:	UDINT := 5020;

	PLCPRJ_MAX_CONNECTIONS					: UDINT := 2;(* Max. number of server<->client connections *)
	PLCPRJ_SERVER_RESPONSE_TIMEOUT			: TIME 	:= T#10s;
	PLCPRJ_CLIENT_SEND_CYCLE_TIME			: TIME 	:= T#5s;

	PLCPRJ_RECEIVER_POLLING_CYCLE_TIME		: TIME 	:= T#40ms;
	PLCPRJ_BUFFER_SIZE						: UINT := 1000; (* Max. internal fifo/receiver buffer size *)

	POSITION_BUFFER_SIZE						: UDINT := 4000; (* Max. internal fifo position receive buffer size*)


	(* Some project specific error codes *)
	PLCPRJ_ERROR_RECEIVE_BUFFER_OVERFLOW	: UDINT := 16#8101; (* receive fifo/buffer overflow*)
	PLCPRJ_ERROR_SEND_BUFFER_OVERFLOW		: UDINT := 16#8102;(* send fifo/buffer overflow *)
	PLCPRJ_ERROR_RESPONSE_TIMEOUT			: UDINT := 16#8103;(* receive timeout error *)
END_VAR

VAR_GLOBAL
	stTurnTableState: ST_TurnTableState;
END_VAR                                                                                               '              ,             B   Globale_Variablen @      SystemTaskInfoArr H  
   SystemInfo H   Hϳ8U	ϳ8U     ZE; 0x:           VAR_GLOBAL
	(*Variables:*)
	stParams 					:	ST_Parameters;

	fVelocity				:	LREAL;					(*Command defined Velocity*)
	(*Positions For Interpolation*)
	StringPositionBuffer		:	ARRAY[-3..POSITION_BUFFER_SIZE] OF BYTE; (* Position Buffer for Interpolation Mode *)
	rxPositionBuffer			:	FB_PositionFifoBuffer;

	(*Commandos:*)
	stCommands			:		ST_Commands;
	stInstances			:		ST_Instances;

	(*INPUT:*)
	bHomeRefAxis1		AT %I* :	BOOL;	(* Axis Reference Switch *)
	bHomeRefAxis2		AT %I* :	BOOL;	(* Axis Reference Switch *)
	bHomeRefAxis3		AT %I* : 	BOOL;	(* Axis Reference Switch *)
	bHomeRefAxis4 		AT %I*: 	BOOL;	(* Axis Reference Switch *)
	bHomeRefAxis5 		AT %I*: 	BOOL;	(* Axis Reference Switch *)
	bHomeRefAxis6		AT %I* : 	BOOL;	(* Axis Reference Switch *)
	bEmergencyButton	AT %I*: 	BOOL;		(* Emergency Button *)

	(*OUTPUT*)
	bGripper			AT %Q* : 	BOOL;	(* FALSE: Gripper is Open; TRUE: Gripper is Closed *)
	bEmergencyButtonLED	AT  %Q*:	BOOL;	(* SOS Button Light *)

	stStartRefDialog		: ST_StartReferenceDialog;
	stReferenceDialog		: ST_ReferenceDialog;
	bHomeRefAxis			: ARRAY [1..6] OF BOOL;		(*reference switches (time-on delayed)*)
	bReset					: BOOL;		(*a reset has been executed*)
	fbSetDefaultHomingDirection: FB_SetDefaulHomingDirection;
END_VAR
                                                                                               '              , ? ? ??        C  TwinCAT_Configuration @      Main.bCloseGripper H     Main.Joint2.PlcToNc H     Main.Joint3.PlcToNc H     Main.Joint1.NcToPlc H     Main.Joint5.NcToPlc H     Main.Joint4.PlcToNc H     Main.Joint2.NcToPlc H     Main.Joint4.NcToPlc H     Main.Joint3.NcToPlc H     Main.Joint5.PlcToNc H     Main.Joint1.PlcToNc H   H^M7U	ϳ8U     ,}??           ?  (* Generated automatically by TwinCAT - (read only) *)
VAR_CONFIG
	Main.Joint[1].PlcToNc AT %QB780 : PLCTONC_AXIS_REF;
	Main.Joint[1].NcToPlc AT %IB772 : NCTOPLC_AXIS_REF;
	Main.Joint[2].PlcToNc AT %QB908 : PLCTONC_AXIS_REF;
	Main.Joint[2].NcToPlc AT %IB900 : NCTOPLC_AXIS_REF;
	Main.Joint[3].PlcToNc AT %QB1036 : PLCTONC_AXIS_REF;
	Main.Joint[3].NcToPlc AT %IB1028 : NCTOPLC_AXIS_REF;
	Main.Joint[4].PlcToNc AT %QB1164 : PLCTONC_AXIS_REF;
	Main.Joint[4].NcToPlc AT %IB1156 : NCTOPLC_AXIS_REF;
	Main.Joint[5].PlcToNc AT %QB1292 : PLCTONC_AXIS_REF;
	Main.Joint[5].NcToPlc AT %IB1284 : NCTOPLC_AXIS_REF;
	Main.Joint[6].PlcToNc AT %QB1420 : PLCTONC_AXIS_REF;
	Main.Joint[6].NcToPlc AT %IB1412 : NCTOPLC_AXIS_REF;
	Main.fbHOME.fbHomeJoint.Axis.PlcToNc AT %QB1556 : PLCTONC_AXIS_REF;
	Main.fbHOME.fbHomeJoint.Axis.NcToPlc AT %IB1544 : NCTOPLC_AXIS_REF;
	.bHomeRefAxis1 AT %IX1540.0 : BOOL;
	.bHomeRefAxis2 AT %IX1540.1 : BOOL;
	.bHomeRefAxis3 AT %IX1540.2 : BOOL;
	.bHomeRefAxis4 AT %IX1540.3 : BOOL;
	.bHomeRefAxis5 AT %IX1540.4 : BOOL;
	.bHomeRefAxis6 AT %IX1540.5 : BOOL;
	.bEmergencyButton AT %IX1540.6 : BOOL;
	.bGripper AT %QX1548.0 : BOOL;
	.bEmergencyButtonLED AT %QX1552.0 : BOOL;
END_VAR                                                                                               '              , x x ??           Variablen_Konfiguration H^M7U	^M7U     ZE; 0x:            VAR_CONFIG
END_VAR
                                                                                                 ?   |0|0 @|    @Z   MS Sans Serif @       HH':'mm':'ss @      dd'-'MM'-'yyyy   dd'-'MM'-'yyyy HH':'mm':'ss?????                               4     ?   ???  ?3 ???   ? ???     
    @??  ???     @      DEFAULT             System      ?   |0|0 @|    @Z   MS Sans Serif @       HH':'mm':'ss @      dd'-'MM'-'yyyy   dd'-'MM'-'yyyy HH':'mm':'ss?????                      )   HH':'mm':'ss @                             dd'-'MM'-'yyyy @        '          ?   , ? ? ?           E_CLIENT_STATE ^M7U	^M7U      t)hact.         i   TYPE E_CLIENT_STATE :
(
	E_CLIENT_INIT	:=	0,
	E_CLIENT_OK		:=	1,
	E_CLIENT_ERROR	:=	100
);
END_TYPE             h  , ? ? ?        
   E_Decision ?\7U	?\7U       ??c          U   TYPE E_Decision: (
	E_Decision_NONE,
	E_Decision_YES,
	E_Decision_NO
);
END_TYPE             >  ,   %?           E_HomingJoint ^M7U	^M7U         &            TYPE E_HomingJoint :
(
	E_HomingJoint_IDLE,
	E_HomingJoint_ShowDialog,
	E_HomingJoint_CheckJointSide,
	E_HomingJoint_SetSearchDirection,
	E_HomingJoint_ExecuteHoming,
	E_HomingJoint_MoveToZero,
	E_HomingJoint_DONE,
	E_HomingJoint_Reset
);
END_TYPE             7  , ? ? ?K           E_JointSide ^M7U	^M7U      9?           ]   TYPE E_JointSide :
(
	E_Joint_Unknown,
	E_Joint_Positive,
	E_Joint_Negative
);
END_TYPE             ?   ,   ?=        
   E_RX_STATE ^M7U	^M7U       2?           L   TYPE E_RX_STATE :
(
	E_RX_INIT		:=		0,
	E_RX_RECEIVED	:=		1
);
END_TYPE             ?   , ? ? ??           E_SERVER_STATE ^M7U	^M7U      nt
			Si        ?   TYPE E_SERVER_STATE :
(
	E_SERVER_INIT	:=	0,		(*Connection is initializing*)
	E_SERVER_CONNECTING := 1,
	E_SERVER_OK		:=	2,		(*Connection is OK*)
	E_SERVER_ERROR	:=	100		(*Connection has ERROR; Read ErrorID for further information*)
);
END_TYPE             }   , ? ? ??           E_STATE ^M7U	^M7U       ???? ?          TYPE E_STATE :
(
	E_HOMING						:=	10,		(*Shows that the Programm is in Homing Mode*)
(*	E_SINGLEHOMING				:=	15,		(*Shows that the Programm is in Singel Axis Homing Mode*)*)
	E_MOVE							:=	20,		(*Shows that the Programm is in Move Mode*)
	E_INTERPOLATION				:=	30,
	E_MOVETOHOMEPOS			:=	40,		(*Shows that the Programm is in Move to Home Position Mode*)
	E_GRASPTURNTABLE			:=	50,		(*Shows that the Programm is in Grasp Turntable Mode*)
	E_STORETURNTABLE			:=	60,
	E_MOVETOLEARNINGPOS		:=	70,
	E_MOVETOTRAY					:=	80,
	E_MOVEGRASPFROMFLOOR		:=	85,
	E_TURNTURNTABLECW			:=	90,
	E_TURNTURNTABLECCW		:=	100,
	E_CLOSEGRIPPER				:=	110,		(*Shows that the Programm is in Closing Gripper Mode*)
	E_OPENGRIPPER				:=	120,		(*Shows that the Programm is in Opening Gripper Mode*)
	E_ENABLEALLAXIS				:=	130,
	E_DISABLEALLAXIS				:=	140,
	E_RESET						:=	150,
	E_STOP							:=	160,		(*Shows that the Programm is in Stop Mode*)
	E_SETPOSTOZERO				:=	170,
	E_MOVEPREGRASPFROMFLOOR 	:= 	180,
	E_MOVEPREGRASPFROMTABLE	:=	185,
	E_PUTOBJECTTOTRAY			:=	190,
	E_JOG							:=	200,
	E_MOVETOCANDLEPOS			:=	210,
	E_WAITFORCOMMAND			:=	800,		(*Shows that the Programm is in Wait For Command Mode*)
	E_ERROR						:=	999		(*Shows that the Programm is in Error Mode*)
);
END_TYPE             ,  , ? ? ??           E_STATE_CANDLE ^M7U	^M7U      h?ئH???        ?   TYPE E_STATE_CANDLE :
(
	E_CANDLE_IDLE,
	E_CANDLE_MoveJoint1,
	E_CANDLE_MoveJoint2,
	E_CANDLE_MoveJoint3,
	E_CANDLE_MoveJoint4,
	E_CANDLE_MoveJoint5,
	E_CANDLE_MoveJoint6,
	E_CANDLE_MoveAll, 
	E_CANDLE_WaitMoveDone
);
END_TYPE             ~   ,                E_STATE_GRASPFLOOR ^M7U	^M7U      ,	','0,	        ?   TYPE E_STATE_GRASPFLOOR :
(

	E_FLOOR_MOVEARMOUT			:=	10,	(**)
	E_FLOOR_MOVEPREGRASP			:=	20,	(**)
	E_FLOOR_MOVEFINALGRASP		:=	30,
	E_FLOOR_MOVETOTRAY				:=	50,	(**)
	E_FLOOR_IDLE						:=	99	(**)
);
END_TYPE             6  , ? ? ??           E_STATE_HOME ^M7U	^M7U      umr3	re        ?   TYPE E_STATE_HOME :
(
	E_HOME_IDLE,
	E_HOME_MoveArmOut3,
	E_HOME_MoveArmOut2,
	E_HOME_MoveArmOut,
	E_HOME_HomePosition
);
END_TYPE                , Z Z fy           E_STATE_HOMING ?c7U	^M7U      > 
rr=>        ?  TYPE E_STATE_HOMING :
(
	E_HOMING_INIT					:=	5,
	E_HOMING_POSCHECK			:=	7,
	E_HOMING_SETDEFAULT			:=	8,
	E_HOMING_DEFAULT			:=	9,
	E_HOMING_JOINT1				:=	10,	(*In this State Joint1 is moving to the reference*)
	E_HOMING_JOINT2				:=	20,	(*In this State Joint2 is moving to the reference*)
	E_HOMING_JOINT3				:=	30,	(*In this State Joint3 is moving to the reference*)
	E_HOMING_JOINT4				:=	40,	(*In this State Joint4 is moving to the reference*)
	E_HOMING_JOINT5				:=	50,	(*In this State Joint5 is moving to the reference*)
	E_HOMING_JOINT6				:=	60,	(*In this State Joint6 is moving to the reference*)
	E_HOMING_DONE				:=	70,	(*In this State the move to the defined home position is started*)
	E_HOMING_RESET				:=	80,
	E_HOMING_MOVERELATIVE		:= 	90,
	E_HOMING_MOVEZERO			:=	95,
	E_HOMING_IDLE					:=	99,
	E_HOMING_RESETCMD			:=  990,
	E_HOMING_ERROR				:=	999,	(*This State shows that an Error while referencing occured, see ErrorIDs*)
	E_HOMING_STOPPED			:=	100
);
END_TYPE             ?   , ? ? ??           E_STATE_INTERPOLATION ^M7U	^M7U      t)hact.         p  TYPE E_STATE_INTERPOLATION :
(
	E_INTERPOL_INIT					:=	10,	(*In this State the command struct is reset*)
	E_INTERPOL_START					:=	20,
	E_INTERPOL_ATHOMEPOS			:=	30,
	E_INTERPOL_MOVING				:=	40,
	E_INTERPOL_READNEXTPOSITION	:=	50,
	E_INTERPOL_DONE					:=	60,
	E_INTERPOL_STOPPED				:=	70,
	E_INTERPOL_IDLE					:=	99,
	E_INTERPOL_ERROR					:=	999
);
END_TYPE             ?   , X X dx           E_STATE_JOG ^M7U	^M7U         &          ?   TYPE E_STATE_JOG :
(

	E_JOG_MOVEJOG			:=	10,	(**)
	E_JOG_DONE				:=	20,	(**)
	E_JOG_MOVING				:=	30, 	(**)
	E_JOG_IDLE					:=	99	(**)
);
END_TYPE             ?   , ? ? V?           E_STATE_MOVE ^M7U	^M7U      	As5	As_        U  TYPE E_STATE_MOVE :
(
	E_MOVE_INIT				:=	10,	(*In this State the command struct is reset*)
	E_MOVE_START				:=	20,
	E_MOVE_MOVING			:=	30,
	E_MOVE_INTERPOLATION	:=	40,
	E_MOVE_SINGLEAXIS			:=	50,
	E_MOVE_DONE				:=	60,
	E_MOVE_STOPPED			:=	70,
	E_MOVE_IDLE				:=	99,
	E_MOVE_RESETCMD			:=990,
	E_MOVE_ERROR				:=	999
);
END_TYPE             ?   ,     ??           E_STATE_OBJECT_TO_TRAY ^M7U	^M7U      			:99**        ?   TYPE E_STATE_OBJECT_TO_TRAY :
(

	E_OBJECT_PREGRASP1			:=	10,	(**)
	E_OBJECT_PREGRASP2			:=	20,	(**)
	E_OBJECT_PREGRASP3			:=	30,	(**)

	E_OBJECT_FINALGRASP			:=	90,	(**)
	(*E_OBJECT_STORE		:=	100,	(**)*)
	E_OBJECT_IDLE			:=	99	(**)
);
END_TYPE             ?   , ? ? ??           E_STATE_PREGRASPFLOOR ^M7U	^M7U       ?4?]        ?   TYPE E_STATE_PREGRASPFLOOR :
(

	E_PRE_FLOOR_MOVEARMOUT			:=	10,	(**)
	E_PRE_FLOOR_MOVEPREGRASP			:=	20,	(**)
	
	
	E_PRE_FLOOR_IDLE						:=	99	(**)
);
END_TYPE                , (           E_STATE_PREGRASPTABLE ^M7U	^M7U       f ?           ?   TYPE E_STATE_PREGRASPTABLE :
(

	E_PRE_TABLE_MOVEARMOUT			:=	10,	(**)
	E_PRE_TABLE_MOVEARMOUT2			:=	15,
	E_PRE_TABLE_MOVEARMOUT3			:=	16,
	E_PRE_TABLE_MOVEPREGRASP			:=	20,	(**)
	
	
	E_PRE_TABLE_IDLE						:=	99	(**)
);
END_TYPE             ?   , Z Z ?y           E_STATE_RESET ^M7U	^M7U                      '  TYPE E_STATE_RESET :
(
	E_RESET_COMMANDS		:=	10,	(*In this State the command struct is reset*)
	E_RESET_AXIS				:=	50, 	(*In this State all Axis are reset*)
	E_RESET_DISABLE			:=	70,	(*In this State all Axis will be disabled *)
	E_RESET_IDLE				:=	99	(*RESET is in idle mode*)
);
END_TYPE             ?   , ? ? ?           E_STATE_TRAY ^M7U	^M7U      t)hact.         ?   TYPE E_STATE_TRAY :
(

	E_TRAY_PREGRASP1			:=	10,	(**)
	E_TRAY_PREGRASP2			:=	20,	(**)
	E_TRAY_PREGRASP3			:=	30,	(**)

	E_TRAY_FINALGRASP			:=	90,	(**)
	E_TRAY_MOVETOHOMEPOS		:=	100,	(**)
	E_TRAY_IDLE			:=	99	(**)
);
END_TYPE             ?   , ? ? ??           E_STATE_TURNTABLE ^M7U	^M7U      t)hact.         I  TYPE E_STATE_TURNTABLE :
(

	E_TURNTABLE_MOVEPREGRASP_1		:=	10,	(**)
	E_TURNTABLE_MOVEPREGRASP_2		:=	15,	(**)
	E_TURNTABLE_MOVEFINALGRASP		:=	20,	(**)
	E_TURNTABLE_CLOSEGRIPPER			:= 	30, 	(**)
	E_TURNTABLE_MOVETOLEARNINGPOS	:=	40,	(**)
	E_TURNTABLE_MOVETOHOMEPOS		:=	50,	(**)
	E_TURNTABLE_IDLE			:=	99	(**)
);
END_TYPE             ?   , ? ? 9           E_STATE_VELOCITY ^M7U	^M7U      ar	:_TG;        ?   TYPE E_STATE_VELOCITY :
(
	E_VELOCITY_CALC			:=	10,
	E_VELOCITY_NEWPOS		:=	20,
	E_VELOCITY_IDLE			:=	99				(**)
);
END_TYPE             V  , ? ? ?           E_SystemMode ^M7U	^M7U         &          ?   TYPE E_SystemMode :
(
	SYSTEMMODE_IDLE:=		0,
	SYSTEMMODE_BOOT:=		1,
	SYSTEMMODE_INIT:=		2,
	SYSTEMMODE_PREOP:=		3,
	SYSTEMMODE_OP:=			4
);
END_TYPE             ?   , ? ? ??           ST_ArmState ^M7U	^M7U      y thhike        a  TYPE ST_ArmState :
STRUCT
	ARM_IN_POS_AREA				:	BOOL;					(*Is TRUE if all axis are in Position Area *)
	ARM_IN_TARGET_POS			:	BOOL;					(*Is TRUE if all axis are at Target Position*)
	ARM_SOFTLIMIT_MAX			:	BOOL;					(*Is TRUE if one or more axis are at the positive software limit*)
	ARM_SOFTLIMIT_MIN				:	BOOL;					(*Is TRUE if one or more axis are at the positive software limit*)
	ARM_HAS_ERROR				:	BOOL;					(*Is TRUE if one ore more axis have an error*)
	ARM_IS_DISABLED				:	BOOL;					(*Is TRUE if one ore more axis are disabled *)
	ARM_HOMED					:	BOOL;					(*Is TRUE if all axis are referenced*)
	ARM_IS_MOVING					:	BOOL;					(*Is TRUE if one ore more axis are moving*)
	ARM_STOPPED					:	BOOL;					(*Is TRUE if all axes have benn stopped*)
	GRIPPER_IS_CLOSED			:	BOOL;					(*Is TRUE if the Gripper is closed*)
	AT_HOME_POS					:	BOOL;					(* TRUE if at home position*)
	AT_LEARNING_POS				:	BOOL;					(* TRUE if at learning position*)
	AT_TRAY_POS					:	BOOL;					(* TRUE if at tray position *)
	AT_TURNTABLE_POS			:	BOOL;					(* TRUE if at Turntable Grasp position*)
	AT_PREGRASPFROMFLOOR_POS	:	BOOL;					(* TRUE if at PreGrasp From Floor position*)
	AT_PREGRASPFROMTABLE_POS	:	BOOL;					(* TRUE if at PreGrasp From Table position*)
	AT_CCW_POS					:	BOOL;					(* TRUE if 6th axis at -168? *)
	AT_CW_POS					:	BOOL;					(* TRUE if 6th axis at 168? *)
	AT_CANDLE_POS				:	BOOL;					(* TRUE if arm is in candle position*)

	(*Merker*)
	ArmInPosAreaChanged			:	BOOL;
	ArmInTargetPosChanged			:	BOOL;
	ArmSoftLimitMaxChanged			:	BOOL;
	ArmSoftLimitMinChanged			:	BOOL;
	ArmHasErrorChanged				:	BOOL;
	ArmIsDisabledChanged			:	BOOL;
	ArmHomedChanged				:	BOOL;
	ArmIsMovingChanged				:	BOOL;
	ArmHasStoppedChanged			:	BOOL;
	GripperIsClosedChanged			:	BOOL;
	AtLearningPosChanged			:	BOOL;
	AtHomePosChanged				:	BOOL;
	AtTrayPosChanged				:	BOOL;
	AtTurntablePosChanged			:	BOOL;
	AtPreGraspFromFloorPosChanged	:	BOOL;
	AtPreGraspFromTablePosChanged	:	BOOL;
	AtCCWPosChanged				:	BOOL;
	AtCWPosChanged				:	BOOL;
	AtCandlePosChanged				:	BOOL;

	AnyDataChanged			:	BOOL;
END_STRUCT
END_TYPE             ?   , < < H[           ST_Commands ^M7U	^M7U      TFCOAN
        [	  TYPE ST_Commands :
STRUCT
(*------------SERVER-----------------*)
	(*SET COMMANDS: *)
	SetAbsolutePosition			:	BOOL;					(*Set a new absolute Position*)
	SetAbsolutePositionValue		:	ARRAY[1..6] OF LREAL;	(*New absolute Positions*)
	SetStartMove					:	BOOL;					(*Start the move*)
	SetMoveVelocity				:	LREAL;
	SetMoveToHomePos			:	BOOL;					(*Start a move to the Home Position*)
	SetMoveToLearningPos		:	BOOL;					(*Start a move to the learning position*)
	SetPutLearningObjectToTray	:	BOOL;					(*Start move an object to tray*)
	SetTurnTurntableCW			:	BOOL;					(*Start turning the turntable CW*)
	SetTurnTurntableCCW			:	BOOL;					(*Start turning the turntable CCW*)
	SetStoreTurntable				:	BOOL;					(*Start a move to store the turntable away*)
	SetMoveToTray				:	BOOL;					(*Start a move to the Tray*)
	SetMoveGraspFromFloor		:	BOOL;					(*Start a move to grasp from Floor*)
	SetMovePreGraspFromFloor	:	BOOL;					(*Start a move to pregrasp from Floor*)
	SetMovePreGraspFromTable	:	BOOL;					(*Start a move to pregrasp from Table*)
	SetEnableInterpolation			:	BOOL;					(*Start interpolatin Mode*)
	SetStartAllAxisRef				:	BOOL;					(*Start a complete axis reference*)
	SetDisableAllAxis				:	BOOL;					(*Disable all axis*)
	SetEnableAllAxis				:	BOOL;					(*Enable all axis*)
	SetOpenGripper				:	BOOL;					(*Open the Gripper*)
	SetCloseGripper				:	BOOL;					(*Close the Gripper*)
	SetReset					:	BOOL;					(*Reset Commands and Axis*)
	SetStopArm					:	BOOL;					(*Stop Arm movement*)
	SetClearPosBuffer				:	BOOL;					(*Clear the Position Buffer*)

	(*GET COMMANDS: *)
	GetActualPos				:	ARRAY[1..6] OF LREAL;		(*Values of actual Positions*)
	(*STATUS; *)
	ArmState					:	ST_ArmState;				(* Arm Status *)
	PositionsForInterpolationReady	:	BOOL;
	EmergencyPressed		:	BOOL;					(* TRUE if button pressed *)
	EmergencyButtonChanged	:	BOOL;

	(*Internal Commands, not used by TCP/IP*)
	SetAllPositionsToZero		:	BOOL;
	SetJogMovePosAxis1		:	BOOL;
	SetJogMoveNegAxis1		:	BOOL;
	SetJogMovePosAxis2		:	BOOL;
	SetJogMoveNegAxis2		:	BOOL;
	SetJogMovePosAxis3		:	BOOL;
	SetJogMoveNegAxis3		:	BOOL;
	SetJogMovePosAxis4		:	BOOL;
	SetJogMoveNegAxis4		:	BOOL;
	SetJogMovePosAxis5		:	BOOL;
	SetJogMoveNegAxis5		:	BOOL;
	SetJogMovePosAxis6		:	BOOL;
	SetJogMoveNegAxis6		:	BOOL;
	SetMoveToCandlePos		:	BOOL;
	SetShutdown				:	BOOL;

END_STRUCT
END_TYPE             ?   , x x ??           ST_Instances ^M7U	^M7U       ]M           q   TYPE ST_Instances :
STRUCT
	Input		:	ST_Instances_Input;
	Output		:	ST_Instances_Output;
END_STRUCT
END_TYPE             ?   , < < H[           ST_Instances_Input ^M7U	^M7U      nt
			Si        ?  TYPE ST_Instances_Input :
STRUCT
	PowerEnable			:	BOOL;

	HomingEnable		:	ARRAY[1..6] OF BOOL;
	HomingMode			:	MC_HomingMode := MC_DefaultHoming; (*For Simulation: MC_ForceCalibration, For Real: MC_DefaultHoming, For Reset: MC_ResetCalibration *)

	MoveAbsoluteEnable			:	BOOL;
	MoveAbsoluteInterpolation		:	BOOL;
	MoveAbsoluteSingleAxis		:	ARRAY[1..6] OF BOOL;
	MoveJogAxisPos				:	ARRAY[1..6] OF BOOL;
	MoveJogAxisNeg				:	ARRAY[1..6] OF BOOL;

	SetPositionEnable		:	BOOL;

	ResetEnable			:	BOOL;

	StopEnable			:	BOOL;

	fPosition		:	ARRAY[1..6] OF LREAL;
	fVelocity		:	ARRAY[1..6] OF LREAL;
	fMaxVelocity	:	LREAL;
END_STRUCT
END_TYPE             ?   , Z Z fy           ST_Instances_Output ^M7U	^M7U      y thhike        ?  TYPE ST_Instances_Output :
STRUCT
	PowerEnableDone	:	BOOL;
	MoveAbsoluteDone	:	BOOL;
	MoveAbsoluteBusy	:	BOOL;
	MoveAbsoluteSingleAxisDone	:	ARRAY[1..6] OF BOOL;
	MoveAbsoluteSingleAxisBusy	:	ARRAY[1..6] OF BOOL;
(*	SetPositionDone		:	BOOL;*)
	HomingEnableDone	:	BOOL;
	ResetEnableDone	:	BOOL;
	ResetFailed			:	BOOL;
	StopDone			:	BOOL;
	ActualPosition		:	ARRAY[1..6] OF LREAL;
	MoveJogDone		:	BOOL;
	MoveJogError		:	BOOL;
	PowerError			:	BOOL;
END_STRUCT
END_TYPE             H  , } } ?2           ST_Parameters ^M7U	^M7U      ?? ??? ?        X  TYPE ST_Parameters :
STRUCT
	MaxVelocity				:	LREAL:=20;				(*Maximum Velocity in DEG per Second*)
	MinVelocity				:	LREAL:=1;				(*Minimum Velocity in DEG per Second*)

	(*__________________________________________PREDEFINED POSITIONS ___________________________ *)
	fZeroPosition				:	ARRAY[1..6] OF LREAL :=  0,		0,		0,		0,		0,		0;
	fHomePosition			:	ARRAY[1..6] OF LREAL:=   90, 		86,		70, 		0, 		110,		0;		(*Home Position: Axis will move to this Position after referencing*)

	(*Positions for moving into the turntable and after storing moving out of the turntable:*)
	fPreGraspTurntable1		:	ARRAY[1..6] OF LREAL :=  90, 		0,		50, 		0,		109, 		90;		(*Pre-grasp Position of the turntable*)
	fPreGraspTurntable2		:	ARRAY[1..6] OF LREAL :=  90,		0,		70,		0,		108,		90;		(*Pre-grasp Position 2 of the turntable*)
	fFinalGraspTurntable		:	ARRAY[1..6] OF LREAL :=  90, 		0,		89, 		0,		91,	 	90;		(*Final grasp Position of the turntable*)

	(*Positions for moving the turntable out and putting the turntable into the storage:*)
	fPreStoreTurntable1		:	ARRAY[1..6] OF LREAL :=  90, 		0,		50, 		-3,		109, 		90;		(*Pre-grasp Position of the turntable*)
	fPreStoreTurntable2		:	ARRAY[1..6] OF LREAL :=  90,		0,		70,		-2,		108,		90;		(*Pre-store Position 2 of the turntable*)
	fFinalStoreTurntable		:	ARRAY[1..6] OF LREAL :=  90, 		0,		90, 		-2,		90.5,	 	90;		(*Final store Position of the turntable*)

	(* Positions for putting an learning object to the tray and then store turntable *)
	fPrePutObjectToTray1		:	ARRAY[1..6] OF LREAL := -47,	 	50,	 	34,	 	68, 		92,	 	95;
	fPrePutObjectToTray2		:	ARRAY[1..6] OF LREAL := -47,	 	78,	 	43,	 	79, 		120,	 	91;
	fPrePutObjectToTray3		:	ARRAY[1..6] OF LREAL := -47,	 	78,	 	26, 		0, 		100,	 	0;
	fFinalPutObjectToTray		:	ARRAY[1..6] OF LREAL := -47,	 	78,	 	43, 		78,	 	120,	 	92;

	fPreGraspTray1			:	ARRAY[1..6] OF LREAL := -47,	 	50,	 	34,	 	68, 		92,	 	95;
	fPreGraspTray2			:	ARRAY[1..6] OF LREAL := -47,	 	78,	 	43,	 	78, 	110,	91;
	fPreGraspTray3			:	ARRAY[1..6] OF LREAL :=  0,	 	78,	 	50, 		0, 		70,	 	0;
	fFinalGraspTray			:	ARRAY[1..6] OF LREAL := -47,	 	78,	 	43, 		78,	 	120,	 	92;

	fMoveArmOut				:	ARRAY[1..6] OF LREAL :=  70,		50,		30,		20,		70,		0;
	fPreGraspFromFloor		:	ARRAY[1..6] OF LREAL :=  69.7,		31.4,		96.31,		122.22,	109.8,	0;
	fFinalGraspFromFloor		:	ARRAY[1..6] OF LREAL :=  67.4,		58.7,		60.2,		151.2,	86.6,		-13.6;

	fMoveArmOut2			:	ARRAY[1..6] OF LREAL := 70,		56,		84,		156,		70,		0;
	fMoveArmOut3			:	ARRAY[1..6] OF LREAL := 0,			56,		114,		156,		110,		104.5;
	fPreGraspFromTable		:	ARRAY[1..6] OF LREAL :=  0,		64,		84,		156,		86,		89;

	fLearningPosition			:	ARRAY[1..6] OF LREAL :=  43,		60,		0,		-23,		125,		168;		(*Learning Position for learning objectes*)

	fTurntableCW				:	ARRAY[1..6] OF LREAL :=  43,		60,		0,		-23,		125,		168;		(* Turn Turntable Clockwise to 170 Degree *)
	fTurntableCCW			:	ARRAY[1..6] OF LREAL :=  43,		60,		0,		-23,		125,		-168;	(* Turn Turntable Clockwise to -170 Degree *)

	fCandlePosition: ARRAY [1..6] OF LREAL := 0,0,0,0,0,0;		(*Candle position*)

	sIdentity				: 	STRING := 'PT2e';
END_STRUCT
END_TYPE             Z  , ? ? ??           ST_ReferenceDialog ^M7U	^M7U      PotiOpon          TYPE ST_ReferenceDialog :
STRUCT
	bActive			:BOOL; 	(*reference wizard active*)
	nJoint			:INT;	(*number of joint to reference next*)
	bPositive		:BOOL;	(*joint is on positive side*)
	bNegative		:BOOL;	(*joint is on negative side*)
END_STRUCT
END_TYPE             X  , d d p           ST_StartReferenceDialog (Q7U	(Q7U         &          ?   TYPE ST_StartReferenceDialog :
STRUCT
	bShow		:BOOL; 	(*reference wizard active*)
	bYes		:BOOL;	(*joint is on positive side*)
	bNo			:BOOL;	(*joint is on negative side*)
	bReserved	:BYTE;
END_STRUCT
END_TYPE             K  ,   %G           ST_TurnTableState ^M7U	^M7U      |	0.			T        h   TYPE ST_TurnTableState :
STRUCT
	TurnTableGrasped		:BOOL;
	InTrayArea				:BOOL;
END_STRUCT
END_TYPE             V N   ,   ?=           F_ADSLOGERROR ^M7U	^M7U      x? ?H??;          FUNCTION F_ADSLOGERROR : DINT
(* This function logs communication error messages to the TwinCAT System Log View *)
VAR_INPUT
	sPrefix		: STRING(20);(* Debug message prefix string (allows the identification of log message source) *)
	nErrID		: DWORD; (* Error code *)
END_VAR
?  IF nErrId = 0 THEN
	F_ADSLOGERROR := ADSLOGSTR( ADSLOG_MSGTYPE_HINT OR ADSLOG_MSGTYPE_LOG, CONCAT( sPrefix, ' No error!   %s'),'' );
ELSIF (  nErrId AND DWORD#16#80000000 ) = 16#80000000 THEN
	F_ADSLOGERROR := ADSLOGDINT( ADSLOG_MSGTYPE_ERROR OR ADSLOG_MSGTYPE_LOG, CONCAT( sPrefix, ' Win32 error: %d' ), DWORD_TO_DINT(SCODE_CODE( nErrId )) );
ELSIF (nErrId AND 16#00008100) =16#00008100 THEN
	F_ADSLOGERROR := ADSLOGDINT( ADSLOG_MSGTYPE_ERROR OR ADSLOG_MSGTYPE_LOG, CONCAT( sPrefix, ' Internal PLC sample project (user) error: %d' ), DWORD_TO_DINT(nErrId) );
ELSIF (nErrId AND 16#00008000) =16#00008000 THEN
	F_ADSLOGERROR := ADSLOGDINT( ADSLOG_MSGTYPE_ERROR OR ADSLOG_MSGTYPE_LOG, CONCAT( sPrefix, ' Internal TCP/IP Connection Server error: %d' ), DWORD_TO_DINT(nErrId) );
ELSE
	F_ADSLOGERROR := ADSLOGDINT( ADSLOG_MSGTYPE_ERROR OR ADSLOG_MSGTYPE_LOG, CONCAT( sPrefix, ' TwinCAT System error: %d' ), DWORD_TO_DINT(nErrId) );
END_IF               O   , Z Z fy           F_ADSLOGSTRING ^M7U	^M7U      x? ?H??;          FUNCTION F_ADSLOGSTRING : DINT
(* This function logs message string to the TwinCAT System Log View *)
VAR_INPUT
	sPrefix	: STRING(20);(* Debug message prefix string (allows the identification of log message source) *)
	sMsg	: T_MaxString;(* Message string *)
END_VAR
VAR
END_VARh   F_ADSLOGSTRING := ADSLOGSTR( ADSLOG_MSGTYPE_HINT OR ADSLOG_MSGTYPE_LOG, CONCAT( sPrefix, '%s' ), sMsg );               :   ,   *=           F_CalcVelocity ^M7U	^M7U       ??          ?   FUNCTION F_CalcVelocity : ARRAY[1..6] OF LREAL
VAR_INPUT
	Position	:	ARRAY[1..6] OF LREAL;
	Velocity	:	LREAL;
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PosDiff	:	ARRAY[1..6] OF LREAL;
END_VAR  
(*Calculating the Position Difference: *)

PosDiff[1] := ABS( Position[1] - Axis[1].NcToPlc.ActPos );
PosDiff[2] := ABS( Position[2] - Axis[2].NcToPlc.ActPos );
PosDiff[3] := ABS( Position[3] - Axis[3].NcToPlc.ActPos );
PosDiff[4] := ABS( Position[4] - Axis[4].NcToPlc.ActPos );
PosDiff[5] := ABS( Position[5] - Axis[5].NcToPlc.ActPos );
PosDiff[6] := ABS( Position[6] - Axis[6].NcToPlc.ActPos );

IF Velocity > stParams.MaxVelocity THEN
	Velocity := stParams.MaxVelocity;
END_IF

(*Axis 6 always turns with Input Velocity*)
F_CalcVelocity[6] := Velocity;

(*Axis 1 has biggest way to move: *)
IF PosDiff[1]<>0 AND
	PosDiff[1] >= PosDiff[2] AND
	PosDiff[1] >= PosDiff[3] AND
	PosDiff[1] >= PosDiff[4] AND
	PosDiff[1] >= PosDiff[5]
(*	PosDiff[1] >= PosDiff[6]*)
THEN
	F_CalcVelocity[1] := Velocity;
	F_CalcVelocity[2] := Velocity/PosDiff[1]*PosDiff[2];
	F_CalcVelocity[3] := Velocity/PosDiff[1]*PosDiff[3];
	F_CalcVelocity[4] := Velocity/PosDiff[1]*PosDiff[4];
	F_CalcVelocity[5] := Velocity/PosDiff[1]*PosDiff[5];
(*	F_CalcVelocity[6] := Velocity/PosDiff[1]*PosDiff[6]; *)
	F_CalcVelocity[6] := stParams.MaxVelocity;

(*Axis 2 has biggest way to move: *)
ELSIF PosDiff[2]<>0 AND
	PosDiff[2] > PosDiff[1] AND
	PosDiff[2] >= PosDiff[3] AND
	PosDiff[2] >= PosDiff[4] AND
	PosDiff[2] >= PosDiff[5]
(*	PosDiff[2] >= PosDiff[6]*)
THEN
	F_CalcVelocity[2] := Velocity;
	F_CalcVelocity[1] := Velocity/PosDiff[2]*PosDiff[1];
	F_CalcVelocity[3] := Velocity/PosDiff[2]*PosDiff[3];
	F_CalcVelocity[4] := Velocity/PosDiff[2]*PosDiff[4];
	F_CalcVelocity[5] := Velocity/PosDiff[2]*PosDiff[5];
(*	F_CalcVelocity[6] := Velocity/PosDiff[2]*PosDiff[6];*)
	F_CalcVelocity[6] := stParams.MaxVelocity;

(*Axis 3 has biggest way to move: *)
ELSIF PosDiff[3]<>0 AND
	PosDiff[3] > PosDiff[1] AND
	PosDiff[3] > PosDiff[2] AND
	PosDiff[3] >= PosDiff[4] AND
	PosDiff[3] >= PosDiff[5]
(*	PosDiff[3] >= PosDiff[6]*)
THEN
	F_CalcVelocity[3] := Velocity;
	F_CalcVelocity[1] := Velocity/PosDiff[3]*PosDiff[1];
	F_CalcVelocity[2] := Velocity/PosDiff[3]*PosDiff[2];
	F_CalcVelocity[4] := Velocity/PosDiff[3]*PosDiff[4];
	F_CalcVelocity[5] := Velocity/PosDiff[3]*PosDiff[5];
(*	F_CalcVelocity[6] := Velocity/PosDiff[3]*PosDiff[6];*)
	F_CalcVelocity[6] := stParams.MaxVelocity;

(*Axis 4 has biggest way to move: *)
ELSIF PosDiff[4]<>0 AND
	PosDiff[4] > PosDiff[1] AND
	PosDiff[4] > PosDiff[2] AND
	PosDiff[4] > PosDiff[3] AND
	PosDiff[4] >= PosDiff[5]
(*	PosDiff[4] >= PosDiff[6]*)
THEN
	F_CalcVelocity[4] := Velocity;
	F_CalcVelocity[1] := Velocity/PosDiff[4]*PosDiff[1];
	F_CalcVelocity[2] := Velocity/PosDiff[4]*PosDiff[2];
	F_CalcVelocity[3] := Velocity/PosDiff[4]*PosDiff[3];
	F_CalcVelocity[5] := Velocity/PosDiff[4]*PosDiff[5];
(*	F_CalcVelocity[6] := Velocity/PosDiff[4]*PosDiff[6];*)
	F_CalcVelocity[6] := stParams.MaxVelocity;

(*Axis 5 has biggest way to move: *)
ELSIF PosDiff[5]<>0 AND
	PosDiff[5] > PosDiff[1] AND
	PosDiff[5] > PosDiff[2] AND
	PosDiff[5] > PosDiff[3] AND
	PosDiff[5] > PosDiff[4]
(*	PosDiff[5] >= PosDiff[6]*)
THEN
	F_CalcVelocity[5] := Velocity;
	F_CalcVelocity[1] := Velocity/PosDiff[5]*PosDiff[1];
	F_CalcVelocity[2] := Velocity/PosDiff[5]*PosDiff[2];
	F_CalcVelocity[3] := Velocity/PosDiff[5]*PosDiff[3];
	F_CalcVelocity[4] := Velocity/PosDiff[5]*PosDiff[4];
(*	F_CalcVelocity[6] := Velocity/PosDiff[5]*PosDiff[6];*)
	F_CalcVelocity[6] := stParams.MaxVelocity;


(*Axis 6 has biggest way to move: *)
ELSIF PosDiff[6]<>0 AND
	PosDiff[6] > PosDiff[1] AND
	PosDiff[6] > PosDiff[2] AND
	PosDiff[6] > PosDiff[3] AND
	PosDiff[6] > PosDiff[4] AND
	PosDiff[6] > PosDiff[5]
THEN
	F_CalcVelocity[6] := stParams.MaxVelocity;
	F_CalcVelocity[1] := Velocity/PosDiff[6]*PosDiff[1];
	F_CalcVelocity[2] := Velocity/PosDiff[6]*PosDiff[2];
	F_CalcVelocity[3] := Velocity/PosDiff[6]*PosDiff[3];
	F_CalcVelocity[4] := Velocity/PosDiff[6]*PosDiff[4];
	F_CalcVelocity[5] := Velocity/PosDiff[6]*PosDiff[5];
END_IF


IF F_CalcVelocity[1] < stParams.MinVelocity THEN
	F_CalcVelocity[1] := stParams.MinVelocity;
END_IF

IF F_CalcVelocity[2] < stParams.MinVelocity THEN
	F_CalcVelocity[2] := stParams.MinVelocity;
END_IF

IF F_CalcVelocity[3] < stParams.MinVelocity THEN
	F_CalcVelocity[3] := stParams.MinVelocity;
END_IF

IF F_CalcVelocity[4] < stParams.MinVelocity THEN
	F_CalcVelocity[4] := stParams.MinVelocity;
END_IF

IF F_CalcVelocity[5] < stParams.MinVelocity THEN
	F_CalcVelocity[5] := stParams.MinVelocity;
END_IF

IF F_CalcVelocity[6] < stParams.MinVelocity THEN
	F_CalcVelocity[6] := stParams.MinVelocity;
END_IF

               *  , K K ?            F_CheckAtCandlePos ^M7U	^M7U      ???x???        ?   FUNCTION F_CheckAtCandlePos : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=0.8;
END_VARZ  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fCandlePosition[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fCandlePosition[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fCandlePosition[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fCandlePosition[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fCandlePosition[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fCandlePosition[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtCandlePos := TRUE;
	END_IF
ELSE
	F_CheckAtCandlePos := FALSE;
END_IF

               %   ,   *=           F_CheckAtFinalGraspFromFloorPos ^M7U	^M7U       ??          ?   FUNCTION F_CheckAtFinalGraspFromFloorPos : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=0.5;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fFinalGraspFromFloor[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fFinalGraspFromFloor[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fFinalGraspFromFloor[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fFinalGraspFromFloor[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fFinalGraspFromFloor[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fFinalGraspFromFloor[6]) < PositionWindow
THEN
	F_CheckAtFinalGraspFromFloorPos := TRUE;

(*	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtFinalGraspFromFloorPos := TRUE;
	END_IF
*)
ELSE
	F_CheckAtFinalGraspFromFloorPos := FALSE;
END_IF               &   ,   *=           F_CheckAtFinalGraspTrayPosition ^M7U	^M7U      ?`	??        ?   FUNCTION F_CheckAtFinalGraspTrayPosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fFinalGraspTray[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fFinalGraspTray[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fFinalGraspTray[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fFinalGraspTray[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fFinalGraspTray[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fFinalGraspTray[6]) < PositionWindow
THEN
(*	F_CheckAtFinalGraspTrayPosition := TRUE;*)

	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtFinalGraspTrayPosition := TRUE;
	END_IF

ELSE
	F_CheckAtFinalGraspTrayPosition := FALSE;
END_IF               '   , ? ? ??        $   F_CheckAtFinalGraspTurntablePosition ^M7U	^M7U      p? ^8??=        ?   FUNCTION F_CheckAtFinalGraspTurntablePosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fFinalGraspTurntable[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fFinalGraspTurntable[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fFinalGraspTurntable[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fFinalGraspTurntable[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fFinalGraspTurntable[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fFinalGraspTurntable[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtFinalGraspTurntablePosition := TRUE;
	END_IF
ELSE
	F_CheckAtFinalGraspTurntablePosition := FALSE;
END_IF               (   , n n $V        %   F_CheckAtFinalPutObjecttoTrayPosition ^M7U	^M7U         &          ?   FUNCTION F_CheckAtFinalPutObjecttoTrayPosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fFinalPutObjectToTray[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fFinalPutObjectToTray[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fFinalPutObjectToTray[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fFinalPutObjectToTray[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fFinalPutObjectToTray[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fFinalPutObjectToTray[6]) < PositionWindow
THEN
(*	F_CheckAtFinalGraspTrayPosition := TRUE;*)

	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtFinalPutObjecttoTrayPosition := TRUE;
	END_IF

ELSE
	F_CheckAtFinalPutObjecttoTrayPosition := FALSE;
END_IF               )   , , , 8L        $   F_CheckAtFinalStoreTurntablePosition ^M7U	^M7U      G?G?GhH        ?   FUNCTION F_CheckAtFinalStoreTurntablePosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fFinalStoreTurntable[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fFinalStoreTurntable[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fFinalStoreTurntable[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fFinalStoreTurntable[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fFinalStoreTurntable[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fFinalStoreTurntable[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtFinalStoreTurntablePosition := TRUE;
	END_IF
ELSE
	F_CheckAtFinalStoreTurntablePosition := FALSE;
END_IF               *   , ? ? ??           F_CheckAtHomePosition ^M7U	^M7U      U=            ?   FUNCTION F_CheckAtHomePosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=0.8;
END_VART  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fHomePosition[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fHomePosition[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fHomePosition[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fHomePosition[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fHomePosition[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fHomePosition[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtHomePosition := TRUE;
	END_IF
ELSE
	F_CheckAtHomePosition := FALSE;
END_IF

               +   , ? ? ??           F_CheckAtLearningPosition ^M7U	^M7U      ????8 ?        ?   FUNCTION F_CheckAtLearningPosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VARu  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fLearningPosition[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fLearningPosition[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fLearningPosition[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fLearningPosition[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fLearningPosition[5]) < PositionWindow (*AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fLearningPosition[6]) < PositionWindow*)
THEN
(*
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtLearningPosition := TRUE;
	END_IF
*)
	IF Axis[1].Status.InTargetPosition AND
		Axis[2].Status.InTargetPosition AND
		Axis[3].Status.InTargetPosition AND
		Axis[4].Status.InTargetPosition AND
		Axis[5].Status.InTargetPosition
	THEN
		F_CheckAtLearningPosition := TRUE;
	END_IF

ELSE
	F_CheckAtLearningPosition := FALSE;
END_IF               ,   ,   *=           F_CheckAtMoveArmOutPos ^M7U	^M7U         ?         ?   FUNCTION F_CheckAtMoveArmOutPos : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VARD  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fMoveArmOut[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fMoveArmOut[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fMoveArmOut[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fMoveArmOut[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fMoveArmOut[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fMoveArmOut[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtMoveArmOutPos := TRUE;
	END_IF
ELSE
	F_CheckAtMoveArmOutPos := FALSE;
END_IF               %  , (           F_CheckAtMoveArmOutPos2 ^M7U	^M7U         &          ?   FUNCTION F_CheckAtMoveArmOutPos2 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VARL  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fMoveArmOut2[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fMoveArmOut2[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fMoveArmOut2[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fMoveArmOut2[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fMoveArmOut2[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fMoveArmOut2[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtMoveArmOutPos2 := TRUE;
	END_IF
ELSE
	F_CheckAtMoveArmOutPos2 := FALSE;
END_IF               $  , X X dx           F_CheckAtMoveArmOutPos3 ^M7U	^M7U       ??          ?   FUNCTION F_CheckAtMoveArmOutPos3 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VARL  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fMoveArmOut3[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fMoveArmOut3[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fMoveArmOut3[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fMoveArmOut3[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fMoveArmOut3[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fMoveArmOut3[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtMoveArmOutPos3 := TRUE;
	END_IF
ELSE
	F_CheckAtMoveArmOutPos3 := FALSE;
END_IF               -   , < < H[           F_CheckAtPreGraspFromFloorPos ^M7U	^M7U      hh??wu        ?   FUNCTION F_CheckAtPreGraspFromFloorPos : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR|  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreGraspFromFloor[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreGraspFromFloor[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreGraspFromFloor[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreGraspFromFloor[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreGraspFromFloor[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreGraspFromFloor[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPreGraspFromFloorPos := TRUE;
	END_IF
ELSE
	F_CheckAtPreGraspFromFloorPos := FALSE;
END_IF               #  , ? ? ??           F_CheckAtPreGraspFromTablePos ^M7U	^M7U       ??          ?   FUNCTION F_CheckAtPreGraspFromTablePos : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR|  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreGraspFromTable[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreGraspFromTable[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreGraspFromTable[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreGraspFromTable[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreGraspFromTable[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreGraspFromTable[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPreGraspFromTablePos := TRUE;
	END_IF
ELSE
	F_CheckAtPreGraspFromTablePos := FALSE;
END_IF               .   , < < H[           F_CheckAtPregraspTrayPos1 ^M7U	^M7U      ?`	??        ?   FUNCTION F_CheckAtPregraspTrayPos1 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR\  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreGraspTray1[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreGraspTray1[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreGraspTray1[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreGraspTray1[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreGraspTray1[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreGraspTray1[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPregraspTrayPos1 := TRUE;
	END_IF
ELSE
	F_CheckAtPregraspTrayPos1 := FALSE;
END_IF               /   , , , d?           F_CheckAtPregraspTrayPos2 ^M7U	^M7U       ??          ?   FUNCTION F_CheckAtPregraspTrayPos2 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR\  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreGraspTray2[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreGraspTray2[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreGraspTray2[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreGraspTray2[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreGraspTray2[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreGraspTray2[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPregraspTrayPos2 := TRUE;
	END_IF
ELSE
	F_CheckAtPregraspTrayPos2 := FALSE;
END_IF               0   , B B z?           F_CheckAtPregraspTrayPos3 ^M7U	^M7U         &          ?   FUNCTION F_CheckAtPregraspTrayPos3 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR\  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreGraspTray3[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreGraspTray3[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreGraspTray3[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreGraspTray3[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreGraspTray3[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreGraspTray3[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPregraspTrayPos3 := TRUE;
	END_IF
ELSE
	F_CheckAtPregraspTrayPos3 := FALSE;
END_IF               1   , Z Z fy        #   F_CheckAtPregraspTurntablePosition1 ^M7U	^M7U      ?mip?Hi        ?   FUNCTION F_CheckAtPregraspTurntablePosition1 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreGraspTurntable1[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreGraspTurntable1[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreGraspTurntable1[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreGraspTurntable1[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreGraspTurntable1[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreGraspTurntable1[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPregraspTurntablePosition1 := TRUE;
	END_IF
ELSE
	F_CheckAtPregraspTurntablePosition1 := FALSE;
END_IF               2   , ? ? ??        #   F_CheckAtPregraspTurntablePosition2 ^M7U	^M7U      ?Я ?`?        ?   FUNCTION F_CheckAtPregraspTurntablePosition2 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreGraspTurntable2[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreGraspTurntable2[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreGraspTurntable2[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreGraspTurntable2[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreGraspTurntable2[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreGraspTurntable2[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtPregraspTurntablePosition2 := TRUE;
	END_IF
ELSE
	F_CheckAtPregraspTurntablePosition2 := FALSE;
END_IF               3   , , , ?           F_CheckAtPrePutObjectToTrayPos1 ^M7U	^M7U         &          ?   FUNCTION F_CheckAtPrePutObjectToTrayPos1 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPrePutObjectToTray1[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPrePutObjectToTray1[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPrePutObjectToTray1[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPrePutObjectToTray1[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPrePutObjectToTray1[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPrePutObjectToTray1[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPrePutObjectToTrayPos1 := TRUE;
	END_IF
ELSE
	F_CheckAtPrePutObjectToTrayPos1 := FALSE;
END_IF               4   , B B ?*           F_CheckAtPrePutObjectToTrayPos2 ^M7U	^M7U         &          ?   FUNCTION F_CheckAtPrePutObjectToTrayPos2 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPrePutObjectToTray2[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPrePutObjectToTray2[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPrePutObjectToTray2[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPrePutObjectToTray2[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPrePutObjectToTray2[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPrePutObjectToTray2[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPrePutObjectToTrayPos2 := TRUE;
	END_IF
ELSE
	F_CheckAtPrePutObjectToTrayPos2 := FALSE;
END_IF               5   , X X @           F_CheckAtPrePutObjectToTrayPos3 ^M7U	^M7U         &          ?   FUNCTION F_CheckAtPrePutObjectToTrayPos3 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=2;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPrePutObjectToTray3[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPrePutObjectToTray3[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPrePutObjectToTray3[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPrePutObjectToTray3[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPrePutObjectToTray3[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPrePutObjectToTray3[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPrePutObjectToTrayPos3 := TRUE;
	END_IF
ELSE
	F_CheckAtPrePutObjectToTrayPos3 := FALSE;
END_IF               6   , (        #   F_CheckAtPreStoreTurntablePosition1 ^M7U	^M7U       ??          ?   FUNCTION F_CheckAtPreStoreTurntablePosition1 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreStoreTurntable1[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreStoreTurntable1[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreStoreTurntable1[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreStoreTurntable1[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreStoreTurntable1[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreStoreTurntable1[6]) < PositionWindow
THEN
	IF F_CheckIfInPosArea(Axis) THEN
		F_CheckAtPreStoreTurntablePosition1 := TRUE;
	END_IF
ELSE
	F_CheckAtPreStoreTurntablePosition1 := FALSE;
END_IF               7   ,              #   F_CheckAtPreStoreTurntablePosition2 ^M7U	^M7U      ???x???        ?   FUNCTION F_CheckAtPreStoreTurntablePosition2 : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fPreStoreTurntable2[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fPreStoreTurntable2[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fPreStoreTurntable2[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fPreStoreTurntable2[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fPreStoreTurntable2[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fPreStoreTurntable2[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtPreStoreTurntablePosition2 := TRUE;
	END_IF
ELSE
	F_CheckAtPreStoreTurntablePosition2 := FALSE;
END_IF               8   , ? ? ??           F_CheckAtTurntableCCWPosition ^M7U	^M7U      X3axc(?        ?   FUNCTION F_CheckAtTurntableCCWPosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR?  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fTurntableCCW[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fTurntableCCW[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fTurntableCCW[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fTurntableCCW[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fTurntableCCW[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fTurntableCCW[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtTurntableCCWPosition := TRUE;
	END_IF
ELSE
	F_CheckAtTurntableCCWPosition := FALSE;
END_IF

(*
IF ABS(Axis[6].NcToPlc.ActPos-stParams.fTurntableCCW[6]) < PositionWindow AND Axis[6].Status.InTargetPosition THEN
	F_CheckAtTurntableCCWPosition := TRUE;
ELSE
	F_CheckAtTurntableCCWPosition := FALSE;
END_IF
*)               9   , ? ? ??           F_CheckAtTurntableCWPosition ^M7U	^M7U       ^_??          ?   FUNCTION F_CheckAtTurntableCWPosition : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	PositionWindow	:	LREAL:=1;
END_VAR4  IF ABS(Axis[1].NcToPlc.ActPos-stParams.fTurntableCW[1]) < PositionWindow AND
	ABS(Axis[2].NcToPlc.ActPos-stParams.fTurntableCW[2]) < PositionWindow AND
	ABS(Axis[3].NcToPlc.ActPos-stParams.fTurntableCW[3]) < PositionWindow AND
	ABS(Axis[4].NcToPlc.ActPos-stParams.fTurntableCW[4]) < PositionWindow AND
	ABS(Axis[5].NcToPlc.ActPos-stParams.fTurntableCW[5]) < PositionWindow AND
	ABS(Axis[6].NcToPlc.ActPos-stParams.fTurntableCW[6]) < PositionWindow
THEN
	IF F_CheckIfInTargetPos(Axis) THEN
		F_CheckAtTurntableCWPosition := TRUE;
	END_IF
ELSE
	F_CheckAtTurntableCWPosition := FALSE;
END_IF

(*
IF ABS(Axis[6].NcToPlc.ActPos-stParams.fTurntableCW[6]) < PositionWindow AND Axis[6].Status.InTargetPosition THEN
	F_CheckAtTurntableCWPosition := TRUE;
ELSE
	F_CheckAtTurntableCWPosition := FALSE;
END_IF
*)                  , ? ? ?           F_CheckAxisHaveStopped ^M7U	^M7U      s e fat         ?   FUNCTION F_CheckAxisHaveStopped : BOOL
VAR_INPUT
END_VAR

VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR

VAR
END_VAR/  IF Axis[1].Status.HasBeenStopped AND
	Axis[2].Status.HasBeenStopped AND
	Axis[3].Status.HasBeenStopped AND
	Axis[4].Status.HasBeenStopped AND
	Axis[5].Status.HasBeenStopped AND
	Axis[6].Status.HasBeenStopped
THEN
	F_CheckAxisHaveStopped := TRUE;
ELSE
	F_CheckAxisHaveStopped := FALSE;
END_IF
               0                      F_CheckAxisInPositionWindow ^M7U	^M7U           J        ?   FUNCTION F_CheckAxisInPositionWindow : BOOL
VAR_INPUT
	SetPosition: REAL;
	PositionWindow: REAL;
END_VAR
VAR_IN_OUT
	Axis: Axis_Ref;
END_VAR
VAR
END_VAR?   IF (ABS(Axis.NcToPlc.ActPos-SetPosition)<PositionWindow) THEN
	F_CheckAxisInPositionWindow	 := TRUE;
ELSE
	F_CheckAxisInPositionWindow	 := FALSE;
END_IF                  , ? ? A?           F_CheckIfAllAxisHomed ^M7U	^M7U      ??????        ?   FUNCTION F_CheckIfAllAxisHomed : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR
VAR
END_VAR?   IF Axis[1].Status.Homed AND
	Axis[2].Status.Homed AND
	Axis[3].Status.Homed AND
	Axis[4].Status.Homed AND
	Axis[5].Status.Homed AND
	Axis[6].Status.Homed
THEN
	F_CheckIfAllAxisHomed := TRUE;
ELSE
	F_CheckIfAllAxisHomed := FALSE;
END_IF                  , ? ? V?           F_CheckIfAxisDisabled ^M7U	^M7U      .               FUNCTION F_CheckIfAxisDisabled : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
END_VAR?  (* If all Axis are Disabled, then output is true: *)

IF Axis[1].Status.Disabled AND
	Axis[2].Status.Disabled AND
	Axis[3].Status.Disabled AND
	Axis[4].Status.Disabled AND
	Axis[5].Status.Disabled AND
	Axis[6].Status.Disabled
THEN
	F_CheckIfAxisDisabled := TRUE;
ELSE
	F_CheckIfAxisDisabled := FALSE;
END_IF                  , < < ?[           F_CheckIfAxisEnabled ^M7U	^M7U       ??          ?   FUNCTION F_CheckIfAxisEnabled : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR
VAR
END_VAR  IF Axis[1].Status.Disabled OR
	Axis[2].Status.Disabled OR
	Axis[3].Status.Disabled OR
	Axis[4].Status.Disabled OR
	Axis[5].Status.Disabled OR
	Axis[6].Status.Disabled
THEN
	F_CheckIfAxisEnabled := FALSE;
ELSE
	F_CheckIfAxisEnabled := TRUE;
END_IF
                  , ? ? _?           F_CheckIfAxisHasError ^M7U	^M7U       Swsn ic        ?   FUNCTION F_CheckIfAxisHasError : BOOL
VAR_INPUT
END_VAR

VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR

VAR
END_VAR?   IF Axis[1].Status.Error OR
	Axis[2].Status.Error OR
	Axis[3].Status.Error OR
	Axis[4].Status.Error OR
	Axis[5].Status.Error OR
	Axis[6].Status.Error
THEN
	F_CheckIfAxisHasError := TRUE;
ELSE
	F_CheckIfAxisHasError := FALSE;
END_IF                  ,   *=           F_CheckIfAxisHasJob ^M7U	^M7U         &          }   FUNCTION F_CheckIfAxisHasJob : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
END_VAR?   IF Axis[1].Status.HasJob OR
	Axis[2].Status.HasJob OR
	Axis[3].Status.HasJob OR
	Axis[4].Status.HasJob OR
	Axis[5].Status.HasJob OR
	Axis[6].Status.HasJob
THEN
	F_CheckIfAxisHasJob := TRUE;
ELSE
	F_CheckIfAxisHasJob := FALSE;
END_IF                  , ? ? ?           F_CheckIfAxisIsMoving ^M7U	^M7U       ??          ?   FUNCTION F_CheckIfAxisIsMoving : BOOL
VAR_INPUT
END_VAR

VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR

VAR
END_VAR?   IF Axis[1].Status.Moving OR
	Axis[2].Status.Moving OR
	Axis[3].Status.Moving OR
	Axis[4].Status.Moving OR
	Axis[5].Status.Moving OR
	Axis[6].Status.Moving
THEN
	F_CheckIfAxisIsMoving := TRUE;
ELSE
	F_CheckIfAxisIsMoving := FALSE;
END_IF
                  , ? ? 8?           F_CheckIfAxisNotMoving ^M7U	^M7U      iore
sM        ?   FUNCTION F_CheckIfAxisNotMoving : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
END_VAR  IF Axis[1].Status.NotMoving AND
	Axis[2].Status.NotMoving AND
	Axis[3].Status.NotMoving AND
	Axis[4].Status.NotMoving AND
	Axis[5].Status.NotMoving AND
	Axis[6].Status.NotMoving
THEN
	F_CheckIfAxisNotMoving := TRUE;
ELSE
	F_CheckIfAxisNotMoving := FALSE;
END_IF                  , ? ? ??           F_CheckIfHomingIsBusy ^M7U	^M7U      ?F)               FUNCTION F_CheckIfHomingIsBusy : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
END_VAR  IF Axis[1].Status.HomingBusy OR
	Axis[2].Status.HomingBusy OR
	Axis[3].Status.HomingBusy OR
	Axis[4].Status.HomingBusy OR
	Axis[5].Status.HomingBusy OR
	Axis[6].Status.HomingBusy
THEN
	F_CheckIfHomingIsBusy := TRUE;
ELSE
	F_CheckIfHomingIsBusy := FALSE;
END_IF                   , Z Z fy           F_CheckIfInPosArea ^M7U	^M7U      ???? ??        ~   FUNCTION F_CheckIfInPosArea : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR
VAR
END_VAR%  IF Axis[1].Status.InPositionArea AND
	Axis[2].Status.InPositionArea AND
	Axis[3].Status.InPositionArea AND
	Axis[4].Status.InPositionArea AND
	Axis[5].Status.InPositionArea AND
	Axis[6].Status.InPositionArea
THEN
	F_CheckIfInPosArea := TRUE;
ELSE
	F_CheckIfInPosArea := FALSE;
END_IF               !   , x x ??           F_CheckIfInTargetPos ^M7U	^M7U      yBto:=AL        ?   FUNCTION F_CheckIfInTargetPos : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR
VAR
	i: INT;
END_VAR?   F_CheckIfInTargetPos := TRUE;

FOR i:=1 TO 6 DO
	IF NOT Axis[i].Status.InTargetPosition THEN
		F_CheckIfInTargetPos := FALSE;
	END_IF
END_FOR               /  , , , r?           F_CheckIfInTargetPosEx ^M7U	^M7U      '?'?'?        ?   FUNCTION F_CheckIfInTargetPosEx : BOOL
VAR_INPUT
	fPosition	:	ARRAY[1..6] OF LREAL;
	PositionWindow 	:ARRAY[1..6] OF REAL;
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
	i: INT;
END_VAR?   F_CheckIfInTargetPosEx := TRUE;

FOR i:=1 TO 6 DO
	IF NOT Axis[i].Status.InTargetPosition
	OR NOT F_CheckAxisInPositionWindow(fPosition[i], PositionWindow[i], Axis[i]) THEN
		F_CheckIfInTargetPosEx := FALSE;
	END_IF
END_FOR               "   , x x ??           F_CheckIfNextPosIsOutOfPosArea ^M7U	^M7U      _?? [ q        ?   FUNCTION F_CheckIfNextPosIsOutOfPosArea : BOOL
VAR_INPUT
	NextPosition		:	ARRAY[1..6] OF LREAL;
	PositionWindow	:	ARRAY[1..6] OF LREAL;
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR
END_VAR  
IF ABS(NextPosition[1] - Axis[1].NcToPlc.ActPos) > PositionWindow[1] OR
	ABS(NextPosition[2] - Axis[2].NcToPlc.ActPos) > PositionWindow[2] OR
	ABS(NextPosition[3] - Axis[3].NcToPlc.ActPos) > PositionWindow[3] OR
	ABS(NextPosition[4] - Axis[4].NcToPlc.ActPos) > PositionWindow[4] OR
	ABS(NextPosition[5] - Axis[5].NcToPlc.ActPos) > PositionWindow[5] OR
	ABS(NextPosition[6] - Axis[6].NcToPlc.ActPos) > PositionWindow[6]
THEN
	F_CheckIfNextPosIsOutOfPosArea := TRUE;
ELSE
	F_CheckIfNextPosIsOutOfPosArea := FALSE;
END_IF               #   , ? ? 8?           F_CheckIfSoftLimitMax ^M7U	^M7U      tuInrgPo        ?   FUNCTION F_CheckIfSoftLimitMax : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR
VAR
END_VARJ  IF Axis[1].Status.SoftLimitMaxExceeded OR
	Axis[2].Status.SoftLimitMaxExceeded OR
	Axis[3].Status.SoftLimitMaxExceeded OR
	Axis[4].Status.SoftLimitMaxExceeded OR
	Axis[5].Status.SoftLimitMaxExceeded OR
	Axis[6].Status.SoftLimitMaxExceeded
THEN
	F_CheckIfSoftLimitMax := TRUE;
ELSE
	F_CheckIfSoftLimitMax := FALSE;
END_IF               $   , ? ? V?           F_CheckIfSoftLimitMin ^M7U	^M7U      tuInrgPo        ?   FUNCTION F_CheckIfSoftLimitMin : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR
VAR
END_VARJ  IF Axis[1].Status.SoftLimitMinExceeded OR
	Axis[2].Status.SoftLimitMinExceeded OR
	Axis[3].Status.SoftLimitMinExceeded OR
	Axis[4].Status.SoftLimitMinExceeded OR
	Axis[5].Status.SoftLimitMinExceeded OR
	Axis[6].Status.SoftLimitMinExceeded
THEN
	F_CheckIfSoftLimitMin := TRUE;
ELSE
	F_CheckIfSoftLimitMin := FALSE;
END_IF               8  , ? ? ?d           F_CheckJointSide ^M7U	^M7U         &          I   FUNCTION F_CheckJointSide : E_JointSide
VAR_INPUT
END_VAR
VAR
END_VAR?   IF stReferenceDialog.bPositive THEN
	F_ResetDialog();
	F_CheckJointSide := E_Joint_Positive;
ELSIF stReferenceDialog.bNegative THEN
	F_ResetDialog();
	F_CheckJointSide := E_Joint_Negative;
ELSE
	F_CheckJointSide := E_Joint_Unknown;
END_IF               `  , 2 2 _?           F_CheckStartRef ?]7U	?[7U       ??          G   FUNCTION F_CheckStartRef : E_Decision
VAR_INPUT
END_VAR
VAR
END_VAR?   IF stStartRefDialog.bYes THEN
	F_ResetStartRefDialog();
	F_CheckStartRef := E_Decision_YES;
ELSIF stStartRefDialog.bNo THEN
	F_ResetStartRefDialog();
	F_CheckStartRef := E_Decision_NO;
ELSE
	F_CheckStartRef := E_Decision_NONE;
END_IF               I   , x x ??           F_CutString ^M7U	^M7U       ]+          ?   FUNCTION F_CutString : ARRAY[0..20] OF STRING
VAR_INPUT
	IN:			T_MaxString;
	sDelimiter:	STRING:='/';
END_VAR
VAR
	_str		:	T_MaxString;
	i: UINT;
	iCutPos	:	ARRAY[0..20] OF INT;
END_VAR?  _str := IN;

FOR i:=0 TO 20 DO
	iCutPos[i] := FIND(_str,sDelimiter);
	IF iCutPos[i] > 0 THEN
		_str := REPLACE( _str, '&', 1, iCutPos[i] );
	ELSE
		EXIT;
	END_IF

END_FOR


FOR i:=0 TO 20 DO
	IF iCutPos[i] > 0 THEN
		IF i=0 THEN
			F_CutString[i]:=LEFT(_str, iCutPos[i]-1);
		ELSE
			F_CutString[i]:=MID(_str, iCutPos[i] - iCutPos[i-1]-1, iCutPos[i-1]+1);
		END_IF
	ELSE
		EXIT;
	END_IF
END_FOR

               J   , ? ? ??           F_MergeSendArmState ^M7U	^M7U       ?(??'0?        ?   FUNCTION F_MergeSendArmState : T_MaxString
VAR_INPUT
	Variable	:		STRING;
	sStatus	:		T_MaxString;
	STCommands	:		ST_Commands;
	sDelimiter	:	STRING;
END_VAR
VAR
	_str		:	T_MaxString;
	i: INT;
END_VAR?	  
_str := CONCAT( _str , Variable );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , sStatus );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_HAS_ERROR) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_HOMED) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_STOPPED) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_IN_POS_AREA) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_IN_TARGET_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_IS_DISABLED) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_IS_MOVING) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_SOFTLIMIT_MAX) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.ARM_SOFTLIMIT_MIN) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_HOME_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_LEARNING_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_TURNTABLE_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_TRAY_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_PREGRASPFROMFLOOR_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_PREGRASPFROMTABLE_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_CCW_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_CW_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.AT_CANDLE_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(STCommands.ArmState.GRIPPER_IS_CLOSED) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(bEmergencyButton) );
_str := CONCAT( _str , sDelimiter );

F_MergeSendArmState := _str;               D   , ? ? ??           F_MergeSendData ^M7U	^M7U      ?v???Xu        ?   FUNCTION F_MergeSendData : T_MaxString
VAR_INPUT
	CMD	:		STRING;
	Variable	:		STRING;
	Value	:		BOOL;
	sDelimiter	:	STRING;
END_VAR
VAR
	_str		:	T_MaxString;
	_strData	:	T_MaxString;
	i: INT;
END_VAR?  _strData := BOOL_TO_STRING(Value);

_str := CONCAT( _str , CMD );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , Variable );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , _strData );
_str := CONCAT( _str , sDelimiter );
(*
_str := CONCAT( _str , BOOL_TO_STRING(_stData.ARM_HOMED) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.ARM_IN_POS_AREA) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.ARM_IN_TARGET_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.ARM_IS_DISABLED) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.ARM_IS_MOVING) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.ARM_SOFTLIMIT_MAX) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.ARM_SOFTLIMIT_MIN) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.AT_HOME_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.AT_LEARNING_POS) );
_str := CONCAT( _str , sDelimiter );
_str := CONCAT( _str , BOOL_TO_STRING(_stData.GRIPPER_IS_CLOSED) );
_str := CONCAT( _str , sDelimiter );
*)
(*
FOR i:=1 TO 6 DO

	_str := CONCAT( _str , DWORD_TO_STRING( Value[i]) );
	_str := CONCAT( _str , sDelimiter );

END_FOR
*)

F_MergeSendData := _str;               K   , ? ? ??           F_MergeString ^M7U	^M7U      x?`?????        ?   FUNCTION F_MergeString : T_MaxString
VAR_INPUT
	IN			:		T_MaxString;
	sStatus		:		T_MaxString;
	sDelimiter	:		STRING:=';';
END_VAR
VAR
	_Merge		:	T_MaxString;
	i: INT;

END_VAR?   _Merge := CONCAT( IN , sDelimiter );
_Merge := CONCAT( _Merge , sStatus );
_Merge := CONCAT( _Merge , sDelimiter );
F_MergeString := _Merge;               L   , ? ? ??           F_MergeStringOneValue ^M7U	^M7U       ?Ȩ8G?        ?   FUNCTION F_MergeStringOneValue : T_MaxString
VAR_INPUT
	IN			:		T_MaxString;
	sStatus		:		T_MaxString;
	sValue		:		T_MaxString;
	sDelimiter	:		STRING:=';';
END_VAR
VAR
	_Merge		:	T_MaxString;
	i: INT;

END_VAR?   _Merge := CONCAT( IN , sDelimiter );
_Merge := CONCAT( _Merge , sStatus );
_Merge := CONCAT( _Merge , sDelimiter );
_Merge := CONCAT( _Merge , sValue );
_Merge := CONCAT( _Merge , sDelimiter );
F_MergeStringOneValue := _Merge;               M   , ? ? ??           F_MergeStringValue ^M7U	^M7U      ??`i?~?x        q  FUNCTION F_MergeStringValue : T_MaxString
VAR_INPUT
	IN			:		ARRAY[0..20] OF STRING;
	sStatus		:		T_MaxString;
	sDelimiter	:		STRING:=';';
	Value1		:		STRING;
	Value2		:		STRING;
	Value3		:		STRING;
	Value4		:		STRING;
	Value5		:		STRING;
	Value6		:		STRING;
END_VAR
VAR
	_Merge		:	T_MaxString;
	_Values		:	ARRAY[0..5] OF T_MaxString;
	i: INT;

END_VAR?  _Values[0] := Value1;
_Values[1] := Value2;
_Values[2] := Value3;
_Values[3] := Value4;
_Values[4] := Value5;
_Values[5] := Value6;

_Merge := CONCAT( IN[1] , sDelimiter );
_Merge := CONCAT( _Merge , sStatus );
_Merge := CONCAT( _Merge , sDelimiter );

FOR i:=0 TO 5 DO
	IF _Values[i] <> '' THEN
		_Merge := CONCAT( _Merge , _Values[i] );
		_Merge := CONCAT( _Merge , sDelimiter );
	ELSE
		EXIT;
	END_IF
END_FOR

F_MergeStringValue := _Merge;               ;   , < < H[           F_ResetCommands ^M7U	^M7U      iore
sM        2  FUNCTION F_ResetCommands : BOOL
(*resets all commands*)
(*|------------------------------------------------------|
  | 16.10.2014|FlA|added some reset stuff of stInstances |
  |------------------------------------------------------|*)
VAR_INPUT
END_VAR
VAR_IN_OUT
END_VAR
VAR
	i 	:	UINT;
END_VAR  (*Reset stCommands*)
stCommands.SetAbsolutePosition:=FALSE;
FOR i:=1 TO 6 DO
	stCommands.SetAbsolutePositionValue[i] := 0;
	stInstances.Input.MoveAbsoluteSingleAxis[i] := FALSE;
	stInstances.Input.HomingEnable[i] := FALSE;
	stInstances.Input.MoveJogAxisNeg[i] := FALSE;
	stInstances.Input.MoveJogAxisPos[i] := FALSE;
END_FOR
stCommands.SetStartMove:=FALSE;
stCommands.SetMoveToHomePos := FALSE;
stCommands.SetMoveToLearningPos := FALSE;
(*stCommands.SetReset := FALSE;*)
stCommands.SetStoreTurntable := FALSE;
stCommands.SetStopArm := FALSE;
stCommands.SetTurnTurntableCCW := FALSE;
stCommands.SetTurnTurntableCW := FALSE;
stCommands.SetMoveToCandlePos := FALSE;
stCommands.SetMovePreGraspFromFloor := FALSE;
stCommands.SetMovePreGraspFromTable := FALSE;
stCommands.SetMoveToTray := FALSE;
(*stCommands.SetStartAxisRef := FALSE;*)
(*stCommands.SetStartAxisRefValue := 0;*)
stCommands.SetStartAllAxisRef:=FALSE;
stCommands.SetDisableAllAxis:=FALSE;
stCommands.SetEnableAllAxis:=FALSE;
stCommands.SetCloseGripper:=FALSE;
stCommands.SetOpenGripper:=FALSE;
(*bGripper := TRUE; (*Close Gripper*)*)
stCommands.SetJogMoveNegAxis1 := FALSE;
stCommands.SetJogMoveNegAxis2 := FALSE;
stCommands.SetJogMoveNegAxis3 := FALSE;
stCommands.SetJogMoveNegAxis4 := FALSE;
stCommands.SetJogMoveNegAxis5 := FALSE;
stCommands.SetJogMoveNegAxis6 := FALSE;
stCommands.SetJogMovePosAxis1 := FALSE;
stCommands.SetJogMovePosAxis2 := FALSE;
stCommands.SetJogMovePosAxis3 := FALSE;
stCommands.SetJogMovePosAxis4 := FALSE;
stCommands.SetJogMovePosAxis5 := FALSE;
stCommands.SetJogMovePosAxis6 := FALSE;

stInstances.Input.MoveAbsoluteEnable:=FALSE;
stInstances.Input.MoveAbsoluteInterpolation:=FALSE;
stInstances.Input.StopEnable := FALSE;
stInstances.Input.SetPositionEnable := FALSE;

bReset := TRUE;
               g  , K K W           F_ResetDialog ?U7U	?U7U         &          ?   FUNCTION F_ResetDialog : BOOL
VAR_INPUT
END_VAR
VAR
END_VAR?   stReferenceDialog.bActive:=FALSE;
stReferenceDialog.bNegative := FALSE;
stReferenceDialog.bPositive := FALSE;
F_ResetDialog := TRUE;               e  ,     ?           F_ResetStartRefDialog d?7U	?U7U         &          G   FUNCTION F_ResetStartRefDialog : BOOL
VAR_INPUT
END_VAR
VAR
END_VAR?   stStartRefDialog.bShow := FALSE;
stStartRefDialog.bNo := FALSE;
stStartRefDialog.bYes := FALSE;
F_ResetStartRefDialog := TRUE;               <   ,   *=           F_SetHomingDisable ^M7U	^M7U      ؏??Xb??        x   FUNCTION F_SetHomingDisable : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	STInstances		:	ST_Instances;
END_VAR
VAR
END_VARO  STInstances.Input.HomingEnable[1] := FALSE;
STInstances.Input.HomingEnable[2] := FALSE;
STInstances.Input.HomingEnable[3] := FALSE;
STInstances.Input.HomingEnable[4] := FALSE;
STInstances.Input.HomingEnable[5] := FALSE;
STInstances.Input.HomingEnable[6] := FALSE;

IF STInstances.Input.HomingEnable[1] AND
	STInstances.Input.HomingEnable[1] AND
	STInstances.Input.HomingEnable[1] AND
	STInstances.Input.HomingEnable[1] AND
	STInstances.Input.HomingEnable[1] AND
	STInstances.Input.HomingEnable[1]
THEN
	F_SetHomingDisable := TRUE;
ELSE
	F_SetHomingDisable := FALSE;
END_IF
               =   ,                F_SetHomingEnable ^M7U	^M7U      ؏??Xb??        w   FUNCTION F_SetHomingEnable : BOOL
VAR_INPUT
END_VAR
VAR_IN_OUT
	STInstances		:	ST_Instances;
END_VAR
VAR
END_VAR
  STInstances.Input.HomingEnable[1] := TRUE;
STInstances.Input.HomingEnable[2] := TRUE;
STInstances.Input.HomingEnable[3] := TRUE;
STInstances.Input.HomingEnable[4] := TRUE;
STInstances.Input.HomingEnable[5] := TRUE;
STInstances.Input.HomingEnable[6] := TRUE;

               b  , d d p.           F_SetRefDialog PU7U	PU7U       ??J          Q   FUNCTION F_SetRefDialog : BOOL
VAR_INPUT
	nJointNo	:INT;
END_VAR
VAR
END_VARK   
stReferenceDialog.nJoint := nJointNo;
stReferenceDialog.bActive := TRUE;               c  , K K W            F_SetStartRefDialog VU7U	VU7U       ??          1   FUNCTION F_SetStartRefDialog : BOOL
VAR
END_VAR!   
stStartRefDialog.bShow := TRUE;               ?  , 2 2 >?           F_SwitchStep ^M7U	^M7U         &          ?   FUNCTION F_SwitchStep : BOOL
VAR_INPUT
	pStepCounter		:POINTER TO INT;
	NextStep			:INT;
	pLastStep			:POINTER TO INT;
END_VAR
VAR
END_VAR:   	pLastStep^ := pStepCounter^;
	pStepCounter^ := NextStep;                  , ? ? ??           FB_CheckState ^M7U	^M7U      ?GP[8h?J        ?  FUNCTION_BLOCK FB_CheckState
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;

END_VAR
VAR
	_ArmHomedStateOld			:	BOOL:=FALSE;
	_ArmIsMovingStateOld			:	BOOL:=FALSE;
	_ArmHasStoppedOld			:	BOOL:=FALSE;
	_ArmIsDisabledStateOld		:	BOOL:=FALSE;
	_ArmHasErrorStateOld			:	BOOL:=FALSE;
	_ArmInPosAreaStateOld		:	BOOL:=FALSE;
	_ArmInTargetPosStateOld		:	BOOL:=FALSE;
	_ArmSoftLimitMaxStateOld		:	BOOL:=FALSE;
	_ArmSoftLimitMinStateOld		:	BOOL:=FALSE;
	_GripperIsClosedOld			:	BOOL:=TRUE;
	_AtHomePosStateOld			:	BOOL:=FALSE;
	_AtLearningPosStateOld		:	BOOL:=FALSE;
	_AtTurntablePosOld			:	BOOL:=FALSE;
	_AtTrayPosOld				:	BOOL:=FALSE;
	_AtPreGraspFromFloorPosOld	:	BOOL:=FALSE;
	_AtPreGraspFromTablePosOld	:	BOOL:=FALSE;
	_AtCCWPosOld				:	BOOL:=FALSE;
	_AtCWPosOld				:	BOOL:=FALSE;
	_EmergencyButtonStateOld		:	BOOL:=FALSE;

(*
	_AtPreGraspPosOld		:	BOOL:=FALSE;
	_AtFinalGraspPosOld		:	BOOL:=FALSE;
	_AtTurntableCWPosOld	:	BOOL:=FALSE;
	_AtTurntableCCWPosOld	:	BOOL:=FALSE;
*)

	PosWindow :ARRAY[1..6] OF REAL := 6(0.8);
	RTRIG: R_TRIG;
	FMP_CandlePos: r_trig;
	FMN_CandlePos: F_TRIG;
END_VAR

VAR_OUTPUT

END_VAR?  
stCommands.ArmState.ARM_HOMED := F_CheckIfAllAxisHomed(Axis);
stCommands.ArmState.ARM_IS_MOVING := F_CheckIfAxisIsMoving(Axis);
stCommands.ArmState.ARM_STOPPED := F_CheckAxisHaveStopped(Axis);
stCommands.ArmState.ARM_IS_DISABLED := F_CheckIfAxisDisabled(Axis);
stCommands.ArmState.ARM_HAS_ERROR := F_CheckIfAxisHasError(Axis);
stCommands.ArmState.ARM_IN_POS_AREA := F_CheckIfInPosArea(Axis);
stCommands.ArmState.ARM_IN_TARGET_POS := F_CheckIfInTargetPosEx(stInstances.Input.fPosition, PosWindow, Axis);
stCommands.ArmState.ARM_SOFTLIMIT_MAX := F_CheckIfSoftLimitMax(Axis);
stCommands.ArmState.ARM_SOFTLIMIT_MIN := F_CheckIfSoftLimitMin(Axis);

stCommands.ArmState.GRIPPER_IS_CLOSED := bGripper;
stCommands.ArmState.AT_HOME_POS := F_CheckAtHomePosition(Axis);
stCommands.ArmState.AT_LEARNING_POS := F_CheckAtLearningPosition(Axis);
stCommands.ArmState.AT_TURNTABLE_POS := F_CheckAtFinalGraspTurntablePosition(Axis);
stCommands.ArmState.AT_TRAY_POS := F_CheckAtFinalGraspTrayPosition(Axis);
stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS := F_CheckAtPreGraspFromFloorPos(Axis);
stCommands.ArmState.AT_PREGRASPFROMTABLE_POS := F_CheckAtPreGraspFromTablePos(Axis);
stCommands.ArmState.AT_CCW_POS := F_CheckAtTurntableCCWPosition(Axis);
stCommands.ArmState.AT_CW_POS := F_CheckAtTurntableCWPosition(Axis);
stCommands.ArmState.AT_CANDLE_POS := F_CheckAtCandlePos(Axis);

stCommands.EmergencyPressed := NOT bEmergencyButton; (*?ffner Kontakt*)

IF _ArmHomedStateOld <> stCommands.ArmState.ARM_HOMED THEN
	stCommands.ArmState.ArmHomedChanged := TRUE;
END_IF

IF _ArmIsMovingStateOld <> stCommands.ArmState.ARM_IS_MOVING THEN
	stCommands.ArmState.ArmIsMovingChanged := TRUE;
END_IF

IF _ArmHasStoppedOld <> stCommands.ArmState.ARM_STOPPED THEN
	stCommands.ArmState.ArmHasStoppedChanged := TRUE;
END_IF

IF _ArmIsDisabledStateOld <> stCommands.ArmState.ARM_IS_DISABLED THEN
	stCommands.ArmState.ArmIsDisabledChanged := TRUE;
END_IF

IF _ArmHasErrorStateOld <> stCommands.ArmState.ARM_HAS_ERROR THEN
	stCommands.ArmState.ArmHasErrorChanged := TRUE;
END_IF

IF _ArmInPosAreaStateOld <> stCommands.ArmState.ARM_IN_POS_AREA THEN
	stCommands.ArmState.ArmInPosAreaChanged := TRUE;
END_IF

IF _ArmInTargetPosStateOld <> stCommands.ArmState.ARM_IN_TARGET_POS THEN
	stCommands.ArmState.ArmInTargetPosChanged := TRUE;
END_IF

IF _ArmSoftLimitMaxStateOld <> stCommands.ArmState.ARM_SOFTLIMIT_MAX THEN
	stCommands.ArmState.ArmSoftLimitMaxChanged := TRUE;
END_IF

IF _ArmSoftLimitMinStateOld <> stCommands.ArmState.ARM_SOFTLIMIT_MIN THEN
	stCommands.ArmState.ArmSoftLimitMinChanged := TRUE;
END_IF

IF _GripperIsClosedOld <> stCommands.ArmState.GRIPPER_IS_CLOSED THEN
	stCommands.ArmState.GripperIsClosedChanged := TRUE;
END_IF


IF _AtHomePosStateOld <> stCommands.ArmState.AT_HOME_POS THEN
	stCommands.ArmState.AtHomePosChanged := TRUE;
END_IF

IF _AtLearningPosStateOld <> stCommands.ArmState.AT_LEARNING_POS THEN
	stCommands.ArmState.AtLearningPosChanged := TRUE;
END_IF

IF _AtTurntablePosOld <> stCommands.ArmState.AT_TURNTABLE_POS THEN
	stCommands.ArmState.AtTurntablePosChanged := TRUE;
END_IF

IF _AtTrayPosOld <> stCommands.ArmState.AT_TRAY_POS THEN
	stCommands.ArmState.AtTrayPosChanged := TRUE;
END_IF

IF _AtPreGraspFromFloorPosOld <> stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS THEN
	stCommands.ArmState.AtPreGraspFromFloorPosChanged := TRUE;
END_IF

IF _AtPreGraspFromTablePosOld <> stCommands.ArmState.AT_PREGRASPFROMTABLE_POS THEN
	stCommands.ArmState.AtPreGraspFromTablePosChanged := TRUE;
END_IF

IF _AtCCWPosOld <> stCommands.ArmState.AT_CCW_POS THEN
	stCommands.ArmState.AtCCWPosChanged := TRUE;
END_IF

IF _AtCWPosOld <> stCommands.ArmState.AT_CW_POS THEN
	stCommands.ArmState.AtCWPosChanged := TRUE;
END_IF

FMP_CandlePos(clk:=stCommands.ArmState.AT_CANDLE_POS);
FMN_CandlePos(clk:=stCommands.ArmState.AT_CANDLE_POS);

IF FMP_CandlePos.Q OR FMN_CandlePos.Q  THEN
	stCommands.ArmState.AtCandlePosChanged := TRUE;
END_IF

IF _EmergencyButtonStateOld <> stCommands.EmergencyPressed THEN
	stCommands.EmergencyButtonChanged := TRUE;
END_IF

(*RTRIG(CLK:= NOT bEmergencyButton); (* ?ffner Kontakt *)

IF RTRIG.Q  THEN
	stCommands.EmergencyPressed := TRUE;
	stCommands.EmergencyButtonChanged := TRUE;
ELSE
	stCommands.EmergencyPressed := FALSE;
END_IF
*)

IF 	stCommands.ArmState.ArmHomedChanged				OR
	stCommands.ArmState.ArmIsMovingChanged				OR
	stCommands.ArmState.ArmHasStoppedChanged			OR
	stCommands.ArmState.ArmIsDisabledChanged			OR
	stCommands.ArmState.ArmHasErrorChanged 				OR
	stCommands.ArmState.ArmInPosAreaChanged			OR
	stCommands.ArmState.ArmInTargetPosChanged			OR
	stCommands.ArmState.ArmSoftLimitMaxChanged			OR
	stCommands.ArmState.ArmSoftLimitMinChanged			OR
	stCommands.ArmState.GripperIsClosedChanged			OR
	stCommands.ArmState.AtHomePosChanged				OR
	stCommands.ArmState.AtLearningPosChanged			OR
	stCommands.ArmState.AtTurntablePosChanged			OR
	stCommands.ArmState.AtTrayPosChanged				OR
	stCommands.ArmState.AtPreGraspFromFloorPosChanged 	OR
	stCommands.ArmState.AtPreGraspFromFloorPosChanged	OR
	stCommands.ArmState.AtCCWPosChanged				OR
	stCommands.ArmState.AtCWPosChanged				OR
	stCommands.EmergencyButtonChanged					OR
	stCommands.ArmState.AtCandlePosChanged
THEN
	stCommands.ArmState.AnyDataChanged := TRUE;
ELSE
	stCommands.ArmState.AnyDataChanged := FALSE;
END_IF


_ArmHomedStateOld			:=	stCommands.ArmState.ARM_HOMED;
_ArmIsMovingStateOld			:=	stCommands.ArmState.ARM_IS_MOVING;
_ArmHasStoppedOld			:=	stCommands.ArmState.ARM_STOPPED;
_ArmIsDisabledStateOld		:=	stCommands.ArmState.ARM_IS_DISABLED;
_ArmHasErrorStateOld			:=	stCommands.ArmState.ARM_HAS_ERROR;
_ArmInPosAreaStateOld		:=	stCommands.ArmState.ARM_IN_POS_AREA;
_ArmInTargetPosStateOld		:=	stCommands.ArmState.ARM_IN_TARGET_POS;
_ArmSoftLimitMaxStateOld		:=	stCommands.ArmState.ARM_SOFTLIMIT_MAX;
_ArmSoftLimitMinStateOld		:=	stCommands.ArmState.ARM_SOFTLIMIT_MIN;
_GripperIsClosedOld			:=	stCommands.ArmState.GRIPPER_IS_CLOSED;
_AtHomePosStateOld			:=	stCommands.ArmState.AT_HOME_POS;
_AtLearningPosStateOld		:=	stCommands.ArmState.AT_LEARNING_POS;
_AtTrayPosOld				:=	stCommands.ArmState.AT_TRAY_POS;
_AtTurntablePosOld			:= 	stCommands.ArmState.AT_TURNTABLE_POS;
_AtPreGraspFromFloorPosOld	:= 	stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS;
_AtPreGraspFromTablePosOld	:= 	stCommands.ArmState.AT_PREGRASPFROMTABLE_POS;
_AtCCWPosOld				:=	stCommands.ArmState.AT_CCW_POS;
_AtCWPosOld				:=	stCommands.ArmState.AT_CW_POS;

_EmergencyButtonStateOld := 	stCommands.EmergencyPressed;               >   , ? ? ??           FB_ClientApplication ^M7U	^M7U                      ?  FUNCTION_BLOCK FB_ClientApplication
(* Sample client application. Adapt this code to match your needs. *)
VAR_INPUT
	bDbg		: BOOL 			:= FALSE;(* TRUE => Enable debug output, FALSE => Disable *)
	sRemoteHost	: STRING(15);
	nRemotePort	: UDINT;
	bEnable		: BOOL 			:= FALSE;
END_VAR
VAR_OUTPUT
	eState		: E_SocketConnectionState := eSOCKET_DISCONNECTED;(* TCP/IP connection state *)
END_VAR
VAR
	sToServer	: T_MaxString	:= '';
	sFromServer	: T_MaxString	:= '';
	fbClient		: FB_LocalClient;(* Client data exchange control function block *)
	tx 			: FB_FrameStringFifo;(* Tx fifo *)
	rx 			: FB_FrameStringFifo;(* Rx fifo *)
	errors		: FB_ProtErrorFifo;(* Error fifo *)
	sndTimer	: TON;
	rcvTimer		: TON;
	firstConnect	: BOOL:=TRUE;
	state		: BYTE;(* application state *)
	rxString		:	ARRAY[0..20] OF STRING;
	iState		:	UINT:=999;
	fbLogState	: FB_LogFile;
	FMP_CandlePos: R_TRIG;
	FMN_CandlePos: F_TRIG;
	sPath: STRING;
END_VAR  (*-------------------------- trigger data exchange -----------------------------------------------*)
fbClient( bDbg := bDbg, rx := rx, tx := tx, errors := errors, bEnable := bEnable, sRemoteHost := sRemoteHost, nRemotePort := nRemotePort, eState=>eState );

(*--------------------------Simple TCP/IP application-----------------------------------------------*)
CASE state OF
	0:(* init state *)
		sndTimer( IN := FALSE, PT := PLCPRJ_CLIENT_SEND_CYCLE_TIME );
		rcvTimer( IN := FALSE, PT := PLCPRJ_SERVER_RESPONSE_TIMEOUT );
		state := 1;

	1:
		(* send string to server *)
		IF fbClient.eState = eSOCKET_CONNECTED THEN
			(*sndTimer( IN := TRUE );*)
			IF sndTimer.Q (*OR iState<>999*) OR stCommands.ArmState.AnyDataChanged THEN
				ProcessSendData;
				sndTimer( IN := FALSE );

				tx.A_AddTail( putValue := sToServer );(* put string to the tx fifo *)
				IF tx.bOk THEN(* success *)
					rcvTimer( IN := FALSE );
					state := 2;(* wait for response (ECHO from server) ) *)
				ELSE(* TX fifo overflow => log error *)
					errors.A_AddTail( putError := PLCPRJ_ERROR_SEND_BUFFER_OVERFLOW );
				END_IF

			END_IF
		ELSE
			state := 0;
		END_IF

	2:(* wait for response from server *)
		REPEAT
			rx.A_RemoveHead();(* fetch string from rx fifo *)
			IF rx.bOk THEN(* success *)
				sFromServer := rx.getValue;
				(* TODO: Check if the response string == request string *)
				ProcessReceiveData;

				state := 0;(* repeat send-receive cycle *)
			ELSE
				rcvTimer( IN := TRUE );
				IF rcvTimer.Q THEN(* receive timeout => log error *)
					rcvTimer( IN := FALSE );
					errors.A_AddTail( putError := PLCPRJ_ERROR_RESPONSE_TIMEOUT );
					state := 0;
				END_IF
			END_IF
		UNTIL NOT rx.bOk
		END_REPEAT


END_CASE

fbLogState();

(*------------------------------- get error messages from error fifo ------------------------------------------*)
REPEAT
	errors.A_RemoveHead( bDbg := bDbg );
	IF errors.bOk THEN
		;(* TODO: Implement error handler *)
	END_IF
UNTIL NOT errors.bOk
END_REPEAT A   , ? ? ??           ProcessReceiveData ^M7U?	  
(*split up the received Command*)
F_SplitString(
	p:=ADR(rxString),
	code:=sFromServer,
	delimiter:=';',
	tarElCnt:=SIZEOF(rxString)/SIZEOF(rxString[0]),
	sizeLine:=SIZEOF(rxString[0]));

(*rxString := F_CutString(sFromServer, ';');*)

IF rxString[1] = 'COMMAND_OK' THEN

	IF rxString[0] = 'ArmHasError' THEN
		stCommands.ArmState.ArmHasErrorChanged := FALSE;

	ELSIF rxString[0] = 'ArmHomed' THEN
		stCommands.ArmState.ArmHomedChanged := FALSE;

	ELSIF rxString[0] = 'ArmInPositionArea' THEN
		stCommands.ArmState.ArmInPosAreaChanged := FALSE;

	ELSIF rxString[0] = 'ArmInTargetPosition' THEN
		stCommands.ArmState.ArmInTargetPosChanged := FALSE;

	ELSIF rxString[0] = 'ArmIsDisabled' THEN
		stCommands.ArmState.ArmIsDisabledChanged := FALSE;

	ELSIF rxString[0] = 'ArmIsMoving' THEN
		stCommands.ArmState.ArmIsMovingChanged := FALSE;

	ELSIF rxString[0] ='ArmHasStopped' THEN
		stCommands.ArmState.ArmHasStoppedChanged := FALSE;

	ELSIF rxString[0] = 'ArmSoftLimitMax' THEN
		stCommands.ArmState.ArmSoftLimitMaxChanged := FALSE;

	ELSIF rxString[0] = 'ArmSoftLimitMin' THEN
		stCommands.ArmState.ArmSoftLimitMinChanged := FALSE;

	ELSIF rxString[0] = 'GripperIsClosed' THEN
		stCommands.ArmState.GripperIsClosedChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtHomePos' THEN
		stCommands.ArmState.AtHomePosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtLearningPos' THEN
		stCommands.ArmState.AtLearningPosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtTurntablePos' THEN
		stCommands.ArmState.AtTurntablePosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtTrayPos' THEN
		stCommands.ArmState.AtTrayPosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtPreGraspFromFloorPos' THEN
		stCommands.ArmState.AtPreGraspFromFloorPosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtPreGraspFromTablePos' THEN
		stCommands.ArmState.AtPreGraspFromTablePosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtCCWPos' THEN
		stCommands.ArmState.AtCCWPosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtCWPos' THEN
		stCommands.ArmState.AtCWPosChanged := FALSE;

	ELSIF rxString[0] = 'ArmAtCandlePos' THEN
		stCommands.ArmState.AtCandlePosChanged := FALSE;

	ELSIF rxString[0] = 'EmergencyPressed' THEN
		stCommands.EmergencyButtonChanged := FALSE;

	END_IF
ELSE
	(*Resend or Error log*)
	;
END_IF

(*Check if all changed data was sent: *)
IF NOT stCommands.ArmState.GripperIsClosedChanged AND
	NOT stCommands.ArmState.AnyDataChanged
THEN
	stCommands.ArmState.AnyDataChanged := FALSE;
END_IFB   , x x ??           ProcessSendData ^M7U"  
IF sndTimer.Q AND NOT stCommands.ArmState.AnyDataChanged THEN
	(*If Send Cycle Timer True and no data changed then Send all States sequently*)
	iState := 10;
END_IF

IF firstConnect THEN
	iState := 10;
	firstConnect := FALSE;
END_IF

sPath := 'C:\Documents and Settings\Administrator\Desktop\Hobbit\Log\';
sPath := CONCAT(sPath, stParams.sIdentity);
sPath := CONCAT(sPath, '_State.txt');

fbLogState.sPathname := sPath;

CASE iState OF
	10:
		(*Send ArmHasError State: *)
		sToServer := F_MergeSendData( 'STATE' , 'ArmHasError', stCommands.ArmState.ARM_HAS_ERROR , ';' );
		iState := 20;

	20:
		(*Send ArmHomed State:*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmHomed', stCommands.ArmState.ARM_HOMED , ';' );
		iState := 30;

	30:
		(*Send ArmInPositionArea State:*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmInPositionArea', stCommands.ArmState.ARM_IN_POS_AREA , ';' );
		iState := 40;

	40:
		(*Send ArmInTargetPosition State:*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmInTargetPosition', stCommands.ArmState.ARM_IN_TARGET_POS , ';' );
		iState := 50;

	50:
		(*Send ArmIsDisabled State:*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmIsDisabled', stCommands.ArmState.ARM_IS_DISABLED , ';' );
		iState := 60;

	60:
		(*Send ArmIsMoving State *)
		sToServer := F_MergeSendData( 'STATE' , 'ArmIsMoving', stCommands.ArmState.ARM_IS_MOVING , ';' );
		iState := 70;

	70:
		(*Send ArmStopped State *)
		sToServer := F_MergeSendData( 'STATE' , 'ArmHasStopped', stCommands.ArmState.ARM_STOPPED , ';' );
		iState := 80;

	80:
		(*Send ArmSoftLimitMax State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmSoftLimitMax', stCommands.ArmState.ARM_SOFTLIMIT_MAX , ';' );
		iState := 90;

	90:
		(*Send ArmSoftLimitMin State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmSoftLimitMin', stCommands.ArmState.ARM_SOFTLIMIT_MIN , ';' );
		iState := 100;

	100:
		(*Send ArmAtHomePos State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtHomePos', stCommands.ArmState.AT_HOME_POS , ';' );
		iState := 110;

	110:
		(*Send ArmAtLearningPos State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtLearningPos', stCommands.ArmState.AT_LEARNING_POS , ';' );
		iState := 120;

	120:
		(*Send ArmAtTurntablePos State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtTurntablePos', stCommands.ArmState.AT_TURNTABLE_POS , ';' );
		iState := 130;

	130:
		(*Send ArmAtTrayPos State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtTrayPos', stCommands.ArmState.AT_TRAY_POS , ';' );
		iState := 140;

	140:
		(*Send ArmAtPreGraspFromFloor State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtPreGraspFromFloorPos', stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS , ';' );
		iState := 145;

	145:
		(*Send ArmAtPreGraspFromTable State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtPreGraspFromFloorPos', stCommands.ArmState.AT_PREGRASPFROMTABLE_POS , ';' );
		iState := 150;

	150:
		(*Send ArmAtCCWPos*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtCCWPos', stCommands.ArmState.AT_CCW_POS , ';' );
		iState := 160;

	160:
		(*Send ArmAtCWPos*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtCWPos', stCommands.ArmState.AT_CW_POS , ';' );
		iState := 200;

	200:
		(*Send GripperIsClosed State*)
		sToServer := F_MergeSendData( 'STATE' , 'GripperIsClosed', stCommands.ArmState.GRIPPER_IS_CLOSED , ';' );
		iState := 210;

	210:
		(*Send GripperIsClosed State*)
		sToServer := F_MergeSendData( 'STATE' , 'ArmAtCandlePos', stCommands.ArmState.AT_CANDLE_POS , ';' );
		iState := 220;

	220:
		(*Send Emergency State*)
		sToServer := F_MergeSendData( 'STATE' , 'EmergencyPressed', stCommands.EmergencyPressed , ';' );
		iState := 999;
END_CASE

FMP_CandlePos(clk:=stCommands.ArmState.AT_CANDLE_POS);
FMN_CandlePos(clk:=stCommands.ArmState.AT_CANDLE_POS);

IF stCommands.ArmState.ArmHasErrorChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmHasError', stCommands.ArmState.ARM_HAS_ERROR , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmHomedChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmHomed', stCommands.ArmState.ARM_HOMED , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmInPosAreaChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmInPositionArea', stCommands.ArmState.ARM_IN_POS_AREA , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmInTargetPosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmInTargetPosition', stCommands.ArmState.ARM_IN_TARGET_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmIsDisabledChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmIsDisabled', stCommands.ArmState.ARM_IS_DISABLED , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmIsMovingChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmIsMoving', stCommands.ArmState.ARM_IS_MOVING , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmHasStoppedChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmHasStopped', stCommands.ArmState.ARM_STOPPED , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmSoftLimitMaxChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmSoftLimitMax', stCommands.ArmState.ARM_SOFTLIMIT_MAX , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.ArmSoftLimitMinChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmSoftLimitMin', stCommands.ArmState.ARM_SOFTLIMIT_MIN , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);


ELSIF stCommands.ArmState.AtHomePosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtHomePos', stCommands.ArmState.AT_HOME_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtLearningPosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtLearningPos', stCommands.ArmState.AT_LEARNING_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtTurntablePosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtTurntablePos', stCommands.ArmState.AT_TURNTABLE_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtTrayPosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtTrayPos', stCommands.ArmState.AT_TRAY_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtPreGraspFromFloorPosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtPreGraspFromFloorPos', stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtPreGraspFromTablePosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtPreGraspFromTablePos', stCommands.ArmState.AT_PREGRASPFROMTABLE_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtCCWPosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtCCWPos', stCommands.ArmState.AT_CCW_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtCWPosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtCWPos', stCommands.ArmState.AT_CW_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.GripperIsClosedChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'GripperIsClosed', stCommands.ArmState.GRIPPER_IS_CLOSED , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.ArmState.AtCandlePosChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'ArmAtCandlePos', stCommands.ArmState.AT_CANDLE_POS , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

ELSIF stCommands.EmergencyButtonChanged THEN
	sToServer := F_MergeSendData( 'STATE' , 'EmergencyPressed', stCommands.EmergencyPressed , ';' );
	(*LOG STATE*)
	fbLogState.LogMessage(sMessage := sToServer);

(*
ELSIF stCommands.ArmState.GripperIsClosedChanged THEN

	sToServer := CONCAT( 'STATE' , ';' );
	sToServer := CONCAT( sToServer , 'GripperState' );
	sToServer := CONCAT( sToServer , ';' );
	sToServer := CONCAT( sToServer , BOOL_TO_STRING(stCommands.ArmState.GRIPPER_IS_CLOSED) );
	sToServer := CONCAT( sToServer , ';' );

ELSE

	stCommands.DataChanged := FALSE;
*)
END_IF             P   , ? ? 9           FB_FrameStringFifo ^M7U	^M7U       ?(?????        ?  FUNCTION_BLOCK FB_FrameStringFifo
(* Tx/Rx (string data) fifo control function block *)
VAR_INPUT
	sDesc		: STRING(20)	:= 'Unknown';(* Debug message description string (allows the identification of log message source) *)
	bDbg		: BOOL		:= FALSE; (* TRUE => Enable debug output, FALSE => Disable *)
	putValue		: T_MaxString := ''; (* String to add (write) to the buffer *)
END_VAR
VAR_OUTPUT
	bOk			: BOOL; 	 	(* TRUE = New entry added or removed succesfully, FALSE = Fifo overflow or fifo empty *)
	getValue		: T_MaxString := ''; (* String removed (read) from buffer *)
	nCount		: UDINT	:= 0;	(* Number of fifo entries *)
	cbFree		: UDINT := 0;	(* Free buffer space *)
END_VAR
VAR
	buffer		: ARRAY[-3..PLCPRJ_BUFFER_SIZE] OF BYTE;(* Internal buffer memory *)
	fbBuffer 		: FB_StringRingBuffer := (bOverwrite := FALSE);(* Basic (lower level) string buffer control function block *)
END_VAR   ; T   , < < ?[        	   A_AddTail ^M7Uc  (* adds new fifo entry *)
fbBuffer.A_AddTail( 	pBuffer:= ADR(buffer), cbBuffer:= SIZEOF(buffer),
					putValue:= putValue, bOk=>bOk, nCount=>nCount );
IF bOk THEN
	cbFree := PLCPRJ_BUFFER_SIZE - fbBuffer.cbSize;(* calculate the free buffer space *)
	IF bDbg THEN(* log message *)
		F_ADSLOGSTRING( CONCAT( sDesc, '<=' ), putValue );
	END_IF
END_IFU   , Z Z ?y           A_RemoveHead ^M7Uk  (* removes oldest fifo entry *)
fbBuffer.A_RemoveHead( pBuffer:= ADR(buffer), cbBuffer:= SIZEOF(buffer),
						bOk=>bOk, getValue=>getValue, nCount=>nCount );
IF bOk THEN
	cbFree := PLCPRJ_BUFFER_SIZE - fbBuffer.cbSize;(* calculate the free buffer space *)
	IF bDbg THEN(* log message *)
		F_ADSLOGSTRING( CONCAT( sDesc, '=>' ), getValue );
	END_IF
END_IFV   , x x ??           A_Reset ^M7U?   (* resets fifo = clears all data *)
fbBuffer.A_Reset( pBuffer:= ADR(buffer), cbBuffer:= SIZEOF(buffer),
				bOk=>bOk, getValue=>getValue, nCount=>nCount );
cbFree := PLCPRJ_BUFFER_SIZE;                ,                FB_Home ??7U	^M7U         S          ?  FUNCTION_BLOCK FB_Home
(*executes homing movement of the arm*)
(*|------------------------------------------------------------------------|
  | 16.10.2014|FlA|added execution of a reset-command in the state-machine |
  |------------------------------------------------------------------------|*)
VAR_INPUT
	bExecute		:	BOOL;
END_VAR
VAR_IN_OUT
	Axis			:	ARRAY[1..6] OF Axis_Ref;
	STInstances	:	ST_Instances;
END_VAR
VAR_OUTPUT
	bHomingDone			:	BOOL;
	bHomingBusy			:	BOOL;
	bReferenceRestarted		:	BOOL;
	bHasBeenStopped		:	BOOL;
	bError					:	BOOL;
END_VAR
VAR
	eHoming			:	E_STATE_HOMING:=99;		(* IDLE *)
	LastHomingState	:	E_STATE_HOMING;

	i	:	UINT;

	RStart	:	R_TRIG;
	RStop	:	R_TRIG;

	MoveRelativeAxis4	:	MC_MoveRelative;
	MoveRelativeAxis6	:	MC_MoveRelative;

	SetPositionAxis4		:	MC_SetPosition;
	SetPositionAxis6		:	MC_SetPosition;
	fbMoveToZero: FB_MoveToZero;
END_VAR?"  (*set status-values*)
(*bHomingDone := F_CheckIfAllAxisHomed(Axis);*)
(*bHomingBusy := F_CheckIfHomingIsBusy(Axis);*)
bHasBeenStopped := F_CheckAxisHaveStopped(Axis);

(*call instances*)
MoveRelativeAxis4(Axis := Axis[4]);
MoveRelativeAxis6(Axis := Axis[6]);
SetPositionAxis4(Axis := Axis[4]);
SetPositionAxis6(Axis := Axis[6]);

(*call MoveToZero-instace*)
fbMoveToZero( );

(*check for axis errors*)
IF F_CheckIfAxisHasError(Axis) THEN
	eHoming := E_HOMING_ERROR;
	bError := TRUE;
	bHomingDone := FALSE;
ELSE
	bError := FALSE;
END_IF

(*execute a reset*)
IF stCommands.SetReset
OR stCommands.SetStopArm THEN
(*	eHoming := E_HOMING_IDLE;*)
	eHoming := E_HOMING_RESETCMD;
END_IF

CASE eHoming OF
	E_HOMING_INIT:
		IF NOT F_CheckIfAllAxisHomed(Axis) THEN
(*
			(*For simulating homing:*)
			STInstances.Input.HomingMode := MC_ForceCalibration;
*)

			(*For real homing*)
			STInstances.Input.HomingMode := MC_DefaultHoming;

			IF  F_CheckIfAxisEnabled(Axis) THEN
				eHoming := E_HOMING_JOINT1;
				LastHomingState := E_HOMING_INIT;
				fbMoveToZero.bExecute := FALSE;
			ELSE
				STInstances.Input.PowerEnable := TRUE;
			END_IF
		ELSE
			LastHomingState := eHoming;
			eHoming := E_HOMING_RESET;
		END_IF


	E_HOMING_JOINT1:
		STInstances.Input.HomingEnable[1] := TRUE;
		(*Wait Until Axis is Referenced: *)
		(*MoveRelative.Execute := FALSE;*)
		IF Axis[1].Status.Homed THEN
			STInstances.Input.HomingEnable[1] := FALSE;
			LastHomingState := E_HOMING_JOINT1;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := E_HOMING_JOINT1;
			eHoming := E_HOMING_JOINT2;
		END_IF


	E_HOMING_JOINT2:
		STInstances.Input.HomingEnable[2] := TRUE;

		(*Wait Until Axis is Referenced: *)
		IF Axis[2].Status.Homed THEN
			STInstances.Input.HomingEnable[2] := FALSE;
			LastHomingState := E_HOMING_JOINT2;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := E_HOMING_JOINT2;
			eHoming := E_HOMING_JOINT3;
		END_IF


	E_HOMING_JOINT3:
		STInstances.Input.HomingEnable[3] := TRUE;
		(*Wait Until Axis is Referenced: *)
		IF Axis[3].Status.Homed THEN
			STInstances.Input.HomingEnable[3] := FALSE;
			LastHomingState := E_HOMING_JOINT3;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := E_HOMING_JOINT3;
			eHoming := E_HOMING_JOINT4;
		END_IF


	E_HOMING_JOINT4:
		IF NOT bHomeRefAxis4 THEN
			MoveRelativeAxis4.Execute := FALSE;
			IF NOT MoveRelativeAxis4.Busy THEN
				STInstances.Input.HomingEnable[4] := TRUE;
				(*Wait Until Axis is Referenced: *)
				IF Axis[4].Status.Homed THEN
					STInstances.Input.HomingEnable[4] := FALSE;
					LastHomingState := E_HOMING_JOINT4;
					eHoming := E_HOMING_MOVEZERO;
				ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
					LastHomingState := E_HOMING_JOINT4;
					eHoming := E_HOMING_JOINT5;
				END_IF
			END_IF
		ELSIF Axis[4].Status.Homed THEN
			LastHomingState := E_HOMING_JOINT4;
			eHoming := E_HOMING_MOVEZERO;
		ELSE
			SetPositionAxis4.Position := 0;
			SetPositionAxis4.Execute := TRUE;
			LastHomingState := eHoming;
			eHoming := E_HOMING_MOVERELATIVE;
		END_IF

	E_HOMING_JOINT5:
		STInstances.Input.HomingEnable[5] := TRUE;
		(*Wait Until Axis is Referenced: *)
		IF Axis[5].Status.Homed THEN
			STInstances.Input.HomingEnable[5] := FALSE;
			LastHomingState := E_HOMING_JOINT5;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := E_HOMING_JOINT4;
			eHoming := E_HOMING_JOINT5;
		END_IF


	E_HOMING_JOINT6:
		IF NOT bHomeRefAxis6 THEN
			MoveRelativeAxis6.Execute := FALSE;
			IF NOT MoveRelativeAxis6.Busy THEN
				STInstances.Input.HomingEnable[6] := TRUE;
				(*Wait Until Axis is Referenced: *)
				IF Axis[6].Status.Homed THEN
					STInstances.Input.HomingEnable[6] := FALSE;
					LastHomingState := E_HOMING_JOINT6;
					eHoming := E_HOMING_DONE;
				ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
					LastHomingState := E_HOMING_JOINT6;
					eHoming := E_HOMING_DONE;
				END_IF
			END_IF
		ELSIF Axis[6].Status.Homed THEN
			LastHomingState := E_HOMING_JOINT6;
			eHoming := E_HOMING_DONE;
		ELSE
			SetPositionAxis6.Position := 0;
			SetPositionAxis6.Execute := TRUE;
			LastHomingState := eHoming;
			eHoming := E_HOMING_MOVERELATIVE;
		END_IF

	E_HOMING_DONE:
		IF F_CheckIfAllAxisHomed(Axis) THEN
			LastHomingState := eHoming;
			eHoming := E_HOMING_IDLE;
			bHomingDone := TRUE;
			bHomingBusy := FALSE;
		ELSE
			LastHomingState := eHoming;
			eHoming := E_HOMING_INIT;
		END_IF

		F_SetHomingDisable(STInstances);


	E_HOMING_RESET:
		F_SetHomingEnable(STInstances);
		IF NOT F_CheckIfAllAxisHomed(Axis) THEN
			F_SetHomingDisable(STInstances);
			LastHomingState := eHoming;
			eHoming := E_HOMING_INIT;
		END_IF

	E_HOMING_MOVEZERO:
		CASE LastHomingState OF
			E_HOMING_JOINT1:
				fbMoveToZero.Axis := ADR(Axis[1]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint2;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT2:
				fbMoveToZero.Axis := ADR(Axis[2]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint3;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT3:
				fbMoveToZero.Axis := ADR(Axis[3]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint4;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT4:
				fbMoveToZero.Axis := ADR(Axis[4]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint5;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT5:
				fbMoveToZero.Axis := ADR(Axis[5]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint6;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT6:
				;
		END_CASE

	E_HOMING_MOVERELATIVE:
		IF LastHomingState = E_HOMING_JOINT4 THEN
			IF NOT SetPositionAxis4.Busy THEN
				SetPositionAxis4.Execute := FALSE;
				MoveRelativeAxis4.Distance := 20;
				MoveRelativeAxis4.Velocity := 5;
				MoveRelativeAxis4.Execute:=TRUE;
				LastHomingState := eHoming;
				eHoming := E_HOMING_JOINT4;
			END_IF
		ELSIF LastHomingState = E_HOMING_JOINT6 THEN
			IF NOT SetPositionAxis6.Busy THEN
				SetPositionAxis6.Execute := FALSE;
				MoveRelativeAxis6.Distance := 20;
				MoveRelativeAxis6.Velocity := 5;
				MoveRelativeAxis6.Execute:=TRUE;
				LastHomingState := eHoming;
				eHoming := E_HOMING_JOINT6;
			END_IF
		END_IF

	E_HOMING_ERROR:
		F_ResetCommands();
		IF NOT F_CheckIfAxisHasError(Axis) THEN
			eHoming := E_HOMING_IDLE;
			bHomingBusy := FALSE;
		END_IF

	E_HOMING_RESETCMD:
		MoveRelativeAxis4.Execute := FALSE;
		MoveRelativeAxis6.Execute := FALSE;
		SetPositionAxis4.Execute := FALSE;
		SetPositionAxis6.Execute := FALSE;
		fbMoveToZero.bExecute := FALSE;
		bHomingBusy := FALSE;
		LastHomingState := eHoming;
		eHoming:=E_HOMING_IDLE;

	E_HOMING_IDLE:
		IF bExecute
		AND NOT bHomingDone THEN		(*Start On Rising Edge*)
			IF bHomeRefAxis4
			AND bHomeRefAxis6 THEN
				IF F_CheckIfAllAxisHomed(Axis) THEN
					bReferenceRestarted := TRUE;
					STInstances.Input.HomingMode := MC_ResetCalibration;
				END_IF
				LastHomingState := eHoming;
				eHoming := E_HOMING_INIT;
				bHomingBusy := TRUE;
				F_SetHomingDisable(STInstances);
			ELSE
				LastHomingState := eHoming;
				eHoming := E_HOMING_ERROR;
			END_IF
		ELSIF NOT bExecute THEN
			bHomingDone :=FALSE;
			bReferenceRestarted := FALSE;
		ELSE
			bReferenceRestarted := FALSE;
		END_IF

END_CASE               J  , ? ? ?           FB_Home_new ͳ8U	??7U       ??             FUNCTION_BLOCK FB_Home_new
(*executes homing movement of the arm*)
(*|------------------------------------------------------------------------|
  | 16.10.2014|FlA|added execution of a reset-command in the state-machine |
  |------------------------------------------------------------------------|*)
VAR_INPUT
	bExecute		:	BOOL;
END_VAR
VAR_IN_OUT
	Axis			:	ARRAY[1..6] OF Axis_Ref;
	STInstances		:	ST_Instances;
END_VAR
VAR_OUTPUT
	bHomingDone			:	BOOL;
	bHomingBusy			:	BOOL;
	bReferenceRestarted		:	BOOL;
	bHasBeenStopped		:	BOOL;
	bError					:	BOOL;
END_VAR
VAR
	eHoming			:	E_STATE_HOMING:=99;		(* IDLE *)
	LastHomingState	:	E_STATE_HOMING;

	i	:	UINT;

	RStart	:	R_TRIG;
	RStop	:	R_TRIG;

	MoveRelativeAxis4	:	MC_MoveRelative;
	MoveRelativeAxis6	:	MC_MoveRelative;

	SetPositionAxis4		:	MC_SetPosition;
	SetPositionAxis6		:	MC_SetPosition;
	fbMoveToZero: FB_MoveToZero;
	eJointSide: E_JointSide;
	fbHomeJoint: FB_HomeJoint;
	eDecision: E_Decision;
	fbHomeDefault: FB_Home;
END_VAR?'  (*set status-values*)
(*bHomingDone := F_CheckIfAllAxisHomed(Axis);
bHomingBusy := F_CheckIfHomingIsBusy(Axis);*)
bHasBeenStopped := F_CheckAxisHaveStopped(Axis);

(*call instances*)
MoveRelativeAxis4(Axis := Axis[4]);
MoveRelativeAxis6(Axis := Axis[6]);
SetPositionAxis4(Axis := Axis[4]);
SetPositionAxis6(Axis := Axis[6]);

(*call MoveToZero-instace*)
fbMoveToZero( );

(*check for axis errors*)
IF F_CheckIfAxisHasError(Axis) THEN
	eHoming := E_HOMING_ERROR;
	bError := TRUE;
	bHomingDone := FALSE;
ELSE
	bError := FALSE;
END_IF

(*execute a reset*)
IF stCommands.SetReset
OR stCommands.SetStopArm THEN
(*	eHoming := E_HOMING_IDLE;*)
	eHoming := E_HOMING_RESETCMD;
END_IF

CASE eHoming OF
	E_HOMING_INIT:
		IF NOT F_CheckIfAllAxisHomed(Axis) THEN
(*
			(*For simulating homing:*) 
			STInstances.Input.HomingMode := MC_ForceCalibration;
*)
			(*For real homing*)
			STInstances.Input.HomingMode := MC_DefaultHoming;

			IF F_CheckIfAxisEnabled(Axis) THEN
				eHoming := E_HOMING_POSCHECK;
				eDecision := E_Decision_NONE;
				F_SetStartRefDialog();
				LastHomingState := E_HOMING_INIT;
				fbMoveToZero.bExecute := FALSE;
			ELSE
				STInstances.Input.PowerEnable := TRUE;
			END_IF
		ELSE
			LastHomingState := eHoming;
			eHoming := E_HOMING_RESET;
		END_IF

	E_HOMING_POSCHECK:
		eDecision := F_CheckStartRef();
		IF eDecision = E_Decision_YES THEN
			LastHomingState := eHoming;
			eHoming := E_HOMING_SETDEFAULT;
			fbSetDefaultHomingDirection.bExecute := TRUE;
		ELSIF eDecision = E_Decision_NO THEN
			LastHomingState := eHoming;
			eHoming := E_HOMING_JOINT2;
		END_IF

	E_HOMING_SETDEFAULT:
		IF NOT fbSetDefaultHomingDirection.bBusy THEN
			fbSetDefaultHomingDirection.bExecute := FALSE;
			IF fbSetDefaultHomingDirection.bDone THEN
				LastHomingState := eHoming;
				eHoming := E_HOMING_DEFAULT;
				fbHomeDefault.bExecute := TRUE;
			ELSIF fbSetDefaultHomingDirection.bError THEN
				LastHomingState := eHoming;
				eHoming := E_HOMING_ERROR;
			END_IF
		END_IF
	E_HOMING_DEFAULT:
		IF NOT fbHomeDefault.bHomingBusy THEN
			fbHomeDefault.bExecute := FALSE;
			LastHomingState := eHoming;
			IF fbHomeDefault.bHomingDone THEN
				eHoming := E_HOMING_DONE;
			ELSE
				eHoming := E_HOMING_ERROR;
			END_IF
		END_IF
	E_HOMING_JOINT1:

		fbHomeJoint.Axis := axis[1];
		fbHomeJoint.nJointNo := 1;
		fbHomeJoint.bExecute := TRUE;
		(*Wait Until Axis is Referenced: *)
		(*MoveRelative.Execute := FALSE;*)
		IF Axis[1].Status.Homed AND fbHomeJoint.DONE THEN
(*			STInstances.Input.HomingEnable[1] := FALSE;*)
			fbHomeJoint.bExecute := FALSE;
			eJointSide := E_Joint_Unknown;
			LastHomingState := E_HOMING_JOINT1;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			eJointSide := E_Joint_Unknown;
			LastHomingState := E_HOMING_JOINT1;
			eHoming := E_HOMING_JOINT2;
		END_IF


	E_HOMING_JOINT2:

(*		STInstances.Input.HomingEnable[2] := TRUE;*)
		fbHomeJoint.Axis := axis[2];
		fbHomeJoint.nJointNo := 2;
		fbHomeJoint.bExecute := TRUE;
		(*Wait Until Axis is Referenced: *)
		IF Axis[2].Status.Homed AND fbHomeJoint.DONE THEN
			fbHomeJoint.bExecute := FALSE;
			STInstances.Input.HomingEnable[2] := FALSE;
			LastHomingState := E_HOMING_JOINT2;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := E_HOMING_JOINT2;
			eHoming := E_HOMING_JOINT3;
		END_IF


	E_HOMING_JOINT3:
(*		STInstances.Input.HomingEnable[3] := TRUE;*)
		fbHomeJoint.Axis := axis[3];
		fbHomeJoint.nJointNo := 3;
		fbHomeJoint.bExecute := TRUE;
		(*Wait Until Axis is Referenced: *)
		IF Axis[3].Status.Homed AND fbHomeJoint.DONE THEN
			fbHomeJoint.bExecute := FALSE;
			STInstances.Input.HomingEnable[3] := FALSE;
			LastHomingState := E_HOMING_JOINT3;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := E_HOMING_JOINT3;
			eHoming := E_HOMING_JOINT4;
		END_IF


	E_HOMING_JOINT4:
		fbHomeJoint.Axis := axis[4];
		fbHomeJoint.nJointNo := 4;
		fbHomeJoint.bExecute := TRUE;
		(*Wait Until Axis is Referenced: *)
		IF Axis[4].Status.Homed AND fbHomeJoint.DONE THEN
			fbHomeJoint.bExecute := FALSE;
			STInstances.Input.HomingEnable[4] := FALSE;
			LastHomingState := eHoming;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := eHoming;
			eHoming := E_HOMING_JOINT5;
		END_IF

	E_HOMING_JOINT5:
		fbHomeJoint.Axis := axis[5];
		fbHomeJoint.nJointNo := 5;
		fbHomeJoint.bExecute := TRUE;
		(*Wait Until Axis is Referenced: *)
		IF Axis[5].Status.Homed AND fbHomeJoint.DONE THEN
			fbHomeJoint.bExecute := FALSE;
			STInstances.Input.HomingEnable[5] := FALSE;
			LastHomingState := eHoming;
			eHoming := E_HOMING_MOVEZERO;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := eHoming;
			eHoming := E_HOMING_JOINT6;
		END_IF


	E_HOMING_JOINT6:
		fbHomeJoint.Axis := axis[6];
		fbHomeJoint.nJointNo := 6;
		fbHomeJoint.bExecute := TRUE;
		(*Wait Until Axis is Referenced: *)
		IF Axis[6].Status.Homed AND fbHomeJoint.DONE THEN
			fbHomeJoint.bExecute := FALSE;
			STInstances.Input.HomingEnable[6] := FALSE;
			LastHomingState := eHoming;
			eHoming := E_HOMING_DONE;
		ELSIF STInstances.Input.HomingMode = MC_ResetCalibration THEN
			LastHomingState := eHoming;
			eHoming := E_HOMING_DONE;
		END_IF

	E_HOMING_DONE:
		IF F_CheckIfAllAxisHomed(Axis) THEN
			LastHomingState := eHoming;
			eHoming := E_HOMING_IDLE;
			bHomingDone := TRUE;
			bHomingBusy := FALSE;
		ELSE
			LastHomingState := eHoming;
			eHoming := E_HOMING_INIT;
		END_IF

		F_SetHomingDisable(STInstances);


	E_HOMING_RESET:
		F_SetHomingEnable(STInstances);
		IF NOT F_CheckIfAllAxisHomed(Axis) THEN
			F_SetHomingDisable(STInstances);
			LastHomingState := eHoming;
			eHoming := E_HOMING_INIT;
		END_IF

	E_HOMING_MOVEZERO:
		CASE LastHomingState OF
			E_HOMING_JOINT1:
				fbMoveToZero.Axis := ADR(Axis[1]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint6;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT2:
				fbMoveToZero.Axis := ADR(Axis[2]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint3;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT3:
				fbMoveToZero.Axis := ADR(Axis[3]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint4;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT4:
				fbMoveToZero.Axis := ADR(Axis[4]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint5;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT5:
				fbMoveToZero.Axis := ADR(Axis[5]);
				fbMoveToZero.bExecute := TRUE;
				IF fbMoveToZero.bDone THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHoming := e_Homing_Joint1;
				ELSIF fbMoveToZero.bError  THEN
					fbMoveToZero.bExecute := FALSE;
					LastHomingState := eHoming;
					eHOming := E_HOMING_Error;
				END_IF
			E_HOMING_JOINT6:
				;
		END_CASE

	E_HOMING_MOVERELATIVE:
		IF LastHomingState = E_HOMING_JOINT4 THEN
			IF NOT SetPositionAxis4.Busy THEN
				SetPositionAxis4.Execute := FALSE;
				MoveRelativeAxis4.Distance := 20;
				MoveRelativeAxis4.Velocity := 5;
				MoveRelativeAxis4.Execute:=TRUE;
				LastHomingState := eHoming;
				eHoming := E_HOMING_JOINT4;
			END_IF
		ELSIF LastHomingState = E_HOMING_JOINT6 THEN
			IF NOT SetPositionAxis6.Busy THEN
				SetPositionAxis6.Execute := FALSE;
				MoveRelativeAxis6.Distance := 20;
				MoveRelativeAxis6.Velocity := 5;
				MoveRelativeAxis6.Execute:=TRUE;
				LastHomingState := eHoming;
				eHoming := E_HOMING_JOINT6;
			END_IF
		END_IF

	E_HOMING_ERROR:
		F_ResetCommands();
		bHomingBusy := FALSE;
		IF NOT F_CheckIfAxisHasError(Axis) THEN
			eHoming := E_HOMING_IDLE;
			bError := TRUE;
		END_IF

	E_HOMING_RESETCMD:
		MoveRelativeAxis4.Execute := FALSE;
		MoveRelativeAxis6.Execute := FALSE;
		SetPositionAxis4.Execute := FALSE;
		SetPositionAxis6.Execute := FALSE;
		fbMoveToZero.bExecute := FALSE;
		fbHomeJoint.bExecute := FALSE;
		fbHomeDefault.bExecute := FALSE;
		LastHomingState := eHoming;
		eHoming:=E_HOMING_IDLE;
		bHomingBusy := FALSE;
		F_ResetStartRefDialog();

	E_HOMING_IDLE:
		F_SetHomingDisable(STInstances);
(*		RStart(CLK:= bExecute);*)
		IF bExecute
		AND NOT bHomingDone THEN		(*Start On Rising Edge*)
			IF F_CheckIfAllAxisHomed(Axis) THEN
				bReferenceRestarted := TRUE;
				STInstances.Input.HomingMode := MC_ResetCalibration;
			END_IF
			LastHomingState := eHoming;
			eHoming := E_HOMING_INIT;
			bHomingBusy := TRUE;
		ELSIF NOT bExecute THEN
			bHomingDone := FALSE;
			bReferenceRestarted := FALSE;
		ELSE
			bReferenceRestarted := FALSE;
		END_IF

END_CASE

fbHomeJoint(
	bExecute:= ,
	nJointNo:= ,
	Axis:= ,
	Busy=> , 
	Error=> , 
	DONE=> );

fbHomeDefault(
	bExecute:= ,
	Axis:= Axis,
	STInstances:= stInstances,
	bHomingDone=> ,
	bHomingBusy=> ,
	bReferenceRestarted=> ,
	bHasBeenStopped=> ,
	bError=> );               =  ,     ?           FB_HomeJoint ?7U	^M7U         &          i  FUNCTION_BLOCK FB_HomeJoint
VAR_INPUT
	bExecute 	:BOOL;
	nJointNo	:INT;
	Axis		:AXIS_REF;
END_VAR
VAR_OUTPUT
	Busy		:BOOL;
	Error		:BOOL;
	DONE		:BOOL;
END_VAR
VAR
	eHomingJoint	: E_HomingJoint;
	eLastHoming		: E_HomingJoint;
	eJointSide 		: E_JointSide;
	fbSetCamSearchDirection: FB_SetCamSearchDirection;
	fbMoveToZero: FB_MoveToZero;
END_VARz  (*reset state machine if stop or reset has been executed*)
IF stCommands.SetReset
OR stCommands.SetStopArm THEN
	F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_Reset, pLastStep:=ADR(eLastHoming));
END_IF

(*state machine*)
CASE eHomingJoint OF
	E_HomingJoint_IDLE:
		IF bExecute AND NOT BUSY THEN
			(*check if joint is already standing on reference cam*)
			IF NOT bHomeRefAxis[nJointNo] THEN
				(*not on cam -> ask user*)
				F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_ShowDialog, pLastStep:=ADR(eLastHoming));
			ELSE
				(*already on cam -> do referencing*)
				F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_ExecuteHoming, pLastStep:=ADR(eLastHoming));
			END_IF
		END_IF

	E_HomingJoint_ShowDialog:
		(*show referencing dialog*)
		eJointSide := E_Joint_Unknown;
		F_SetRefDialog(nJointNo);
		F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_CheckJointSide, pLastStep:=ADR(eLastHoming));

	E_HomingJoint_CheckJointSide:
		(*analyze users input*)
		eJointSide := F_CheckJointSide();
		IF eJointSide <> E_Joint_Unknown THEN
			F_ResetDialog();
			F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_SetSearchDirection, pLastStep:=ADR(eLastHoming));
		END_IF

	E_HomingJoint_SetSearchDirection:
		(*set search directinon for calibration cam... *)
		CASE nJointNo OF
			1,2,3,4,5,6:
			CASE eJointSide OF
				E_Joint_Negative:
					fbSetCamSearchDirection.eSearchDirection:=E_Joint_Negative;
				E_Joint_Positive:
					fbSetCamSearchDirection.eSearchDirection:=E_Joint_Positive;
			END_CASE
			7:
			CASE eJointSide OF
				E_Joint_Negative:
					fbSetCamSearchDirection.eSearchDirection:=E_Joint_Positive;
				E_Joint_Positive:
					fbSetCamSearchDirection.eSearchDirection:=E_Joint_Negative;
			END_CASE
		END_CASE

		(*... and write it down to the NC*)
		fbSetCamSearchDirection.bExecute :=TRUE;
		IF fbSetCamSearchDirection.bBusy THEN
			fbSetCamSearchDirection.bExecute :=FALSE;
			F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_ExecuteHoming, pLastStep:=ADR(eLastHoming));
		END_IF

	E_HomingJoint_ExecuteHoming:

		IF NOT fbSetCamSearchDirection.bBusy AND NOT fbSetCamSearchDirection.bError THEN
			(*setting of search direction worked -> start referencing*)
			stInstances.Input.HomingEnable[nJointNo] := TRUE;
		ELSIF fbSetCamSearchDirection.bError THEN
			(*setting of search direction failed -> go to error*)
			Busy := FALSE;
			Error := TRUE;
			F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_IDLE, pLastStep:=ADR(eLastHoming));
		END_IF

		IF Axis.Status.Homed THEN
			(*axis homed -> go to next step*)
			stInstances.Input.HomingEnable[nJointNo] := FALSE;
			F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_MoveToZero, pLastStep:=ADR(eLastHoming));
		END_IF

	E_HomingJoint_MoveToZero:
		(*move axis to zero position*)
		fbMoveToZero.bExecute := TRUE;

		IF fbMoveToZero.bDone THEN
			(*move to zero done*)
			F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_DONE, pLastStep:=ADR(eLastHoming));
		ELSIF fbMoveToZero.bError THEN
			(*something went wrong*)
			Busy := FALSE;
			Error := TRUE;
			F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_IDLE, pLastStep:=ADR(eLastHoming));
		END_IF

	E_HomingJoint_Done:
		(*homing done -> set succesful-flag*)
		Done := TRUE;
		Busy := FALSE;
		F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_IDLE, pLastStep:=ADR(eLastHoming));

	E_HomingJoint_Reset:
		(*a reset has been executed*)
		DONE := FALSE;
		Busy := FALSE;
		fbSetCamSearchDirection.bExecute := FALSE;
		fbMoveToZero.bExecute := FALSE;
		eJointSide := E_Joint_Unknown;
		F_ResetDialog();
		F_SwitchStep(pStepCounter:=ADR(eHomingJoint), NextStep:= E_HomingJoint_IDLE, pLastStep:=ADR(eLastHoming));

END_CASE

fbSetCamSearchDirection(
	bExecute:= ,
	nAxisID:= Axis.NcToPlc.AxisId,
	eSearchDirection:= ,
	bBusy=> ,
	bError=> ,
	nErrID=>  );

fbMoveToZero(
	bExecute:= ,
	Axis:= ADR(Axis),
	bDone=> ,
	bError=> , 
	nErrID=> );                  ,                FB_Instances z?7U	@?7U      ЋH`" ?        ,  FUNCTION_BLOCK FB_Instances
(*calls nearly all instances of used functions-blocks*)
(*|--------------------------------------------------------------------|
  | 16.10.2014|FlA|added check for MC_Power errors and MC_Reset errors |
  |--------------------------------------------------------------------|*)
VAR_INPUT

END_VAR
VAR_OUTPUT

END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
	STInstances	:	ST_Instances;
END_VAR
VAR
	_MoveAbsoluteExecute	:	BOOL;
	i		:	UINT;
	_Velocity		:	ARRAY[1..6] OF LREAL;

	SetPosOpt	:	ST_SetPositionOptions;
	SetDestallOpt	:	ST_PowerStepperStruct;

	Power_Axis	: 	ARRAY[1..6] OF MC_PowerStepper;
	MoveAbsoluteAxis: 	ARRAY[1..12] OF MC_MoveAbsolute;
	MoveJog		:	ARRAY[1..6] OF MC_Jog;
	SetPosition		:	ARRAY[1..6] OF MC_SetPosition;
	HomingAxis	:	ARRAY[1..6] OF MC_Home;
	ResetHoming	:	ARRAY[1..6] OF MC_Home;
	ResetAxis	:	ARRAY[1..6] OF MC_Reset;
	StopAxis		:	ARRAY[1..6] OF MC_Stop;
	logged: ARRAY[1..6] OF BOOL;
	k: INT;
	TON_RefCam: ARRAY[1..6] OF TON;
	bHackReset: ARRAY[1..6] OF BOOL;
END_VAR?/  (*assign hardware inputs to internal array-variable*)
TON_RefCam[1].IN := bHomeRefAxis1;
TON_RefCam[2].IN := bHomeRefAxis2;
TON_RefCam[3].IN := bHomeRefAxis3;
TON_RefCam[4].IN := bHomeRefAxis4;
TON_RefCam[5].IN := bHomeRefAxis5;
TON_RefCam[6].IN := bHomeRefAxis6;

FOR k := 1 TO 6 DO
	TON_RefCam[k](pt:=t#20ms);
	bHomeRefAxis[k] := TON_RefCam[k].Q;
END_FOR


IF STInstances.Input.MoveAbsoluteInterpolation THEN
	_MoveAbsoluteExecute := NOT _MoveAbsoluteExecute;		(* Toggle *)
ELSE
	_MoveAbsoluteExecute := STInstances.Input.MoveAbsoluteEnable; (* no Toggle *)
END_IF

(*_Velocity := F_CalcVelocity(STInstances.Input.fPosition, Axis); *)
_Velocity := STInstances.Input.fVelocity;

SetDestallOpt.DestallDetectMode := PwStDetectMode_Lagging;
SetDestallOpt.DestallMode:= PwStMode_SetError;(*PwStMode_UseOverride;*)
SetDestallOpt.DestallEnable := TRUE;

(*-----------------------Axis 1:--------------------------------*)
(*Power*)
Power_Axis[1](
	Enable:= STInstances.Input.PowerEnable,
	Enable_Positive:= TRUE,
	Enable_Negative:= TRUE,
	DestallParams := SetDestallOpt,
	Axis:= Axis[1] );

(*Homing *)
HomingAxis[1](
	Execute:= STInstances.Input.HomingEnable[1],
	Position:= DEFAULT_HOME_POSITION,
	HomingMode:= stInstances.Input.HomingMode,
	bCalibrationCam:= bHomeRefAxis[1],
	Axis:= Axis[1] );

(*Move Absolute*)
MoveAbsoluteAxis[1](
	Execute:= _MoveAbsoluteExecute OR STInstances.Input.MoveAbsoluteSingleAxis[1],
	Position:= STInstances.Input.fPosition[1],
	Velocity:= _Velocity[1],
	Axis:= Axis[1] );

MoveAbsoluteAxis[7](
	Execute:= NOT _MoveAbsoluteExecute,
	Position:= STInstances.Input.fPosition[1],
	Velocity:= _Velocity[1],
	Axis:= Axis[1] );

(*Move Jog*)
MoveJog[1](
	JogForward:= stInstances.Input.MoveJogAxisPos[1],
	JogBackwards:= stInstances.Input.MoveJogAxisNeg[1],
	Mode:= MC_JOGMODE_INCHING,
	Position:=2,
	Velocity:=5,
	Axis:= Axis[1]
);


(*Set Position*)
SetPosOpt.ClearPositionLag:=TRUE;
SetPosition[1](
	Execute:= STInstances.Input.SetPositionEnable,
	Position:= 0,
	Options:= SetPosOpt,
	Axis:= Axis[1]);

(*Axis Reset:*)
ResetAxis[1](
	Execute:= STInstances.Input.ResetEnable OR bHackReset[1],
	Axis:= Axis[1] );


(*Stop Axis:*)
StopAxis[1](
	Execute:= STInstances.Input.StopEnable,
	Axis:= Axis[1]);

(*-----------------------Axis 2:--------------------------------*)
(*Power*)
Power_Axis[2](
	Enable:= STInstances.Input.PowerEnable,
	Enable_Positive:= TRUE,
	Enable_Negative:= TRUE,
	DestallParams := SetDestallOpt,
	Axis:= Axis[2] );

(*Homing*)
HomingAxis[2](
	Execute:= STInstances.Input.HomingEnable[2],
	Position:= DEFAULT_HOME_POSITION,
	HomingMode:= stInstances.Input.HomingMode,
	bCalibrationCam:= bHomeRefAxis[2],
	Axis:= Axis[2] );

(*Move Absolute*)
MoveAbsoluteAxis[2](
	Execute:= _MoveAbsoluteExecute OR STInstances.Input.MoveAbsoluteSingleAxis[2],
	Position:= STInstances.Input.fPosition[2],
	Velocity:= _Velocity[2],
	Axis:= Axis[2] );

MoveAbsoluteAxis[8](
	Execute:= NOT _MoveAbsoluteExecute,
	Position:= STInstances.Input.fPosition[2],
	Velocity:= _Velocity[2],
	Axis:= Axis[2] );

(*Move Jog*)
MoveJog[2](
	JogForward:= stInstances.Input.MoveJogAxisPos[2],
	JogBackwards:= stInstances.Input.MoveJogAxisNeg[2],
	Mode:= MC_JOGMODE_INCHING,
	Position:=2,
	Velocity:=5,
	Axis:= Axis[2]
);

(*Set Position*)
SetPosition[2](
	Execute:= STInstances.Input.SetPositionEnable,
	Position:= 0,
	Options:= SetPosOpt,
	Axis:= Axis[2]);

(*Axis Reset:*)
ResetAxis[2](
	Execute:= STInstances.Input.ResetEnable OR bHackReset[2],
	Axis:= Axis[2] );

(*Stop Axis:*)
StopAxis[2](
	Execute:= STInstances.Input.StopEnable,
	Axis:= Axis[2]);


(*-----------------------Axis 3:--------------------------------*)
(*Power*)
Power_Axis[3](
	Enable:= STInstances.Input.PowerEnable,
	Enable_Positive:= TRUE,
	Enable_Negative:= TRUE,
	DestallParams := SetDestallOpt,
	Axis:= Axis[3] );

(*Homing*)
HomingAxis[3](
	Execute:= STInstances.Input.HomingEnable[3],
	Position:= DEFAULT_HOME_POSITION,
	HomingMode:= stInstances.Input.HomingMode,
	bCalibrationCam:= bHomeRefAxis[3],
	Axis:= Axis[3] );

(*MoveAbsolute*)
MoveAbsoluteAxis[3](
	Execute:= _MoveAbsoluteExecute OR STInstances.Input.MoveAbsoluteSingleAxis[3],
	Position:= STInstances.Input.fPosition[3],
	Velocity:= _Velocity[3],
	Axis:= Axis[3] );

MoveAbsoluteAxis[9](
	Execute:= NOT _MoveAbsoluteExecute,
	Position:= STInstances.Input.fPosition[3],
	Velocity:= _Velocity[3],
	Axis:= Axis[3] );

(*Move Jog*)
MoveJog[3](
	JogForward:= stInstances.Input.MoveJogAxisPos[3],
	JogBackwards:= stInstances.Input.MoveJogAxisNeg[3],
	Mode:= MC_JOGMODE_INCHING,
	Position:=2,
	Velocity:=5,
	Axis:= Axis[3]
);

(*Set Position*)
SetPosition[3](
	Execute:= STInstances.Input.SetPositionEnable,
	Position:= 0,
	Options:= SetPosOpt,
	Axis:= Axis[3]);

(*Axis Reset:*)
ResetAxis[3](
	Execute:= STInstances.Input.ResetEnable OR bHackReset[3],
	Axis:= Axis[3] );

(*Stop Axis:*)
StopAxis[3](
	Execute:= STInstances.Input.StopEnable,
	Axis:= Axis[3]);


(*-----------------------Axis 4:--------------------------------*)
(*Power*)
Power_Axis[4](
	Enable:= STInstances.Input.PowerEnable,
	Enable_Positive:= TRUE,
	Enable_Negative:= TRUE,
	DestallParams := SetDestallOpt,
	Axis:= Axis[4] );

(*Homing*)
HomingAxis[4](
	Execute:= STInstances.Input.HomingEnable[4],
	Position:= DEFAULT_HOME_POSITION,
	HomingMode:= stInstances.Input.HomingMode,
	bCalibrationCam:= bHomeRefAxis[4],
	Axis:= Axis[4] );

(*Move Absolute*)
MoveAbsoluteAxis[4](
	Execute:= _MoveAbsoluteExecute OR STInstances.Input.MoveAbsoluteSingleAxis[4],
	Position:= STInstances.Input.fPosition[4],
	Velocity:= _Velocity[4],
	Axis:= Axis[4] );

MoveAbsoluteAxis[10](
	Execute:= NOT _MoveAbsoluteExecute,
	Position:= STInstances.Input.fPosition[4],
	Velocity:= _Velocity[4],
	Axis:= Axis[4] );

(*Move Jog*)
MoveJog[4](
	JogForward:= stInstances.Input.MoveJogAxisPos[4],
	JogBackwards:= stInstances.Input.MoveJogAxisNeg[4],
	Mode:= MC_JOGMODE_INCHING,
	Position:=2,
	Velocity:=5,
	Axis:= Axis[4]
);

(*Set Position*)
SetPosition[4](
	Execute:= STInstances.Input.SetPositionEnable,
	Position:= 0,
	Options:= SetPosOpt,
	Axis:= Axis[4]);

(*Axis Reset:*)
ResetAxis[4](
	Execute:= STInstances.Input.ResetEnable OR bHackReset[4],
	Axis:= Axis[4] );

(*Stop Axis:*)
StopAxis[4](
	Execute:= STInstances.Input.StopEnable,
	Axis:= Axis[4]);


(*-----------------------Axis 5:--------------------------------*)
(*Power*)
Power_Axis[5](
	Enable:= STInstances.Input.PowerEnable,
	Enable_Positive:= TRUE,
	Enable_Negative:= TRUE,
	DestallParams := SetDestallOpt,
	Axis:= Axis[5] );

(*Homing*)
HomingAxis[5](
	Execute:= STInstances.Input.HomingEnable[5],
	Position:= DEFAULT_HOME_POSITION,
	HomingMode:= stInstances.Input.HomingMode,
	bCalibrationCam:= bHomeRefAxis[5],
	Axis:= Axis[5] );

(*Move Absolute*)
MoveAbsoluteAxis[5](
	Execute:= _MoveAbsoluteExecute OR STInstances.Input.MoveAbsoluteSingleAxis[5],
	Position:= STInstances.Input.fPosition[5],
	Velocity:= _Velocity[5],
	Axis:= Axis[5] );

MoveAbsoluteAxis[11](
	Execute:= NOT _MoveAbsoluteExecute,
	Position:= STInstances.Input.fPosition[5],
	Velocity:= _Velocity[5],
	Axis:= Axis[5] );

(*Move Jog*)
MoveJog[5](
	JogForward:= stInstances.Input.MoveJogAxisPos[5],
	JogBackwards:= stInstances.Input.MoveJogAxisNeg[5],
	Mode:= MC_JOGMODE_INCHING,
	Position:=2,
	Velocity:=5,
	Axis:= Axis[5]
);

(*Set Position*)
SetPosition[5](
	Execute:= STInstances.Input.SetPositionEnable,
	Position:= 0,
	Options:= SetPosOpt,
	Axis:= Axis[5]);

(*Axis Reset:*)
ResetAxis[5](
	Execute:= STInstances.Input.ResetEnable OR bHackReset[5],
	Axis:= Axis[5] );

(*Stop Axis:*)
StopAxis[5](
	Execute:= STInstances.Input.StopEnable,
	Axis:= Axis[5]);


(*-----------------------Axis 6:--------------------------------*)
(*Power*)
Power_Axis[6](
	Enable:= STInstances.Input.PowerEnable,
	Enable_Positive:= TRUE,
	Enable_Negative:= TRUE,
	DestallParams := SetDestallOpt,
	Axis:= Axis[6] );

(*Homing *)
HomingAxis[6](
	Execute:= STInstances.Input.HomingEnable[6],
	Position:= DEFAULT_HOME_POSITION,
	HomingMode:= stInstances.Input.HomingMode,
	bCalibrationCam:= bHomeRefAxis[6],
	Axis:= Axis[6] );

(*Move Absolute*)
MoveAbsoluteAxis[6](
	Execute:= _MoveAbsoluteExecute OR STInstances.Input.MoveAbsoluteSingleAxis[6],
	Position:= STInstances.Input.fPosition[6],
	Velocity:= _Velocity[6],
	Axis:= Axis[6] );

MoveAbsoluteAxis[12](
	Execute:= NOT _MoveAbsoluteExecute,
	Position:= STInstances.Input.fPosition[6],
	Velocity:= _Velocity[6],
	Axis:= Axis[6] );

(*Move Jog*)
MoveJog[6](
	JogForward:= stInstances.Input.MoveJogAxisPos[6],
	JogBackwards:= stInstances.Input.MoveJogAxisNeg[6],
	Mode:= MC_JOGMODE_INCHING,
	Position:=2,
	Velocity:=5,
	Axis:= Axis[6]
);

(*Set Position*)
SetPosition[6](
	Execute:= STInstances.Input.SetPositionEnable,
	Position:= 0,
	Options:= SetPosOpt,
	Axis:= Axis[6]);

(*Axis Reset:*)
ResetAxis[6](
	Execute:= STInstances.Input.ResetEnable OR bHackReset[6],
	Axis:= Axis[6] );

(*Stop Axis:*)
StopAxis[6](
	Execute:= STInstances.Input.StopEnable,
	Axis:= Axis[6]);


STInstances.Output.PowerEnableDone := F_CheckIfAxisEnabled(Axis);

IF MoveAbsoluteAxis[1].Done AND
	MoveAbsoluteAxis[2].Done AND
	MoveAbsoluteAxis[3].Done AND
	MoveAbsoluteAxis[4].Done AND
	MoveAbsoluteAxis[5].Done AND
	MoveAbsoluteAxis[6].Done
THEN
	STInstances.Output.MoveAbsoluteDone := TRUE;
ELSE
	STInstances.Output.MoveAbsoluteDone := FALSE;
END_IF

IF MoveAbsoluteAxis[1].Busy OR (*and*)
	MoveAbsoluteAxis[2].Busy OR (*AND*)
	MoveAbsoluteAxis[3].Busy OR (*AND*)
	MoveAbsoluteAxis[4].Busy OR (*AND*)
	MoveAbsoluteAxis[5].Busy OR (*AND*)
	MoveAbsoluteAxis[6].Busy
THEN
	STInstances.Output.MoveAbsoluteBusy := TRUE;
ELSE
	STInstances.Output.MoveAbsoluteBusy := FALSE;
END_IF

FOR i:=1 TO 6 DO
	STInstances.Output.MoveAbsoluteSingleAxisDone[i] := MoveAbsoluteAxis[i].Done;
END_FOR

IF ResetAxis[1].Done AND
	ResetAxis[2].Done AND
	ResetAxis[3].Done AND
	ResetAxis[4].Done AND
	ResetAxis[5].Done AND
	ResetAxis[6].Done
THEN
	STInstances.Output.ResetEnableDone := TRUE;
ELSE
	STInstances.Output.ResetEnableDone := FALSE;
END_IF

STInstances.Output.ResetFailed :=FALSE;
FOR i := 1 TO 6 DO
	STInstances.Output.ResetFailed := STInstances.Output.ResetFailed OR ResetAxis[i].Error;
END_FOR

IF StopAxis[1].Done AND
	StopAxis[2].Done AND
	StopAxis[3].Done AND
	StopAxis[4].Done AND
	StopAxis[5].Done AND
	StopAxis[6].Done
THEN
	STInstances.Output.StopDone := TRUE;
ELSE
	STInstances.Output.StopDone := FALSE;
END_IF

FOR i:=1 TO 6 DO
	STInstances.Output.ActualPosition[i] := Axis[i].NcToPlc.ActPos;
END_FOR


IF MoveJog[1].Done OR
	MoveJog[2].Done OR
	MoveJog[3].Done OR
	MoveJog[4].Done OR
	MoveJog[5].Done OR
	MoveJog[6].Done
THEN
	STInstances.Output.MoveJogDone := TRUE;
ELSE
	STInstances.Output.MoveJogDone := FALSE;
END_IF

IF MoveJog[1].Error OR
	MoveJog[2].Error OR
	MoveJog[3].Error OR
	MoveJog[4].Error OR
	MoveJog[5].Error OR
	MoveJog[6].Error
THEN
	STInstances.Output.MoveJogError := TRUE;
ELSE
	STInstances.Output.MoveJogError := FALSE;
END_IF

(*check for errors with powering motors*)
STInstances.Output.PowerError := FALSE;
FOR i := 1 TO 6 DO
	STInstances.Output.PowerError := STInstances.Output.PowerError OR	Power_Axis[i].Error ;

	IF Axis[i].Status.HasJob AND NOT Axis[i].Status.Moving AND NOT stInstances.Input.HomingEnable[i] THEN
		bHackReset[i] := TRUE;
		logged[i] :=TRUE;
		IF NOT logged[i] THEN
			ADSLOGSTR( ADSLOG_MSGTYPE_ERROR OR ADSLOG_MSGTYPE_LOG, 'DEBUG: Axis %s has stopped working!!!', INT_TO_STRING(i) );
		END_IF
	ELSE
		bHackReset[i] := FALSE;
		logged[i] := FALSE;
	END_IF

	ResetHoming[i](
	Execute:= SetPosition[i].Done,
	Position:= 0,
	HomingMode:= MC_ResetCalibration,
	BufferMode:= ,
	Options:= ,
	bCalibrationCam:= ,
	Axis:= Axis[i],
	Done=> ,
	Busy=> ,
	Active=> ,
	CommandAborted=> ,
	Error=> ,
	ErrorID=> );

END_FOR
                  ,                FB_LedBlinking ^M7U	^M7U       ??          ?   FUNCTION_BLOCK FB_LedBlinking
VAR_INPUT
	bSwitch	:	BOOL; (*?ffner Kontakt*)
	tTime	:	TIME;
END_VAR

VAR_OUTPUT
	bLED 	:	BOOL;
END_VAR
VAR
	i	:	UINT;
	LastLEDState : BOOL;

	TP_On: TP;
	TP_Off: TP;
	FTrig: F_TRIG;
	RTrig: R_TRIG;
END_VAR?  (*
	5 mal Blinken
	wenn bSwitch lowsignal
*)
FTrig(CLK:= bSwitch);

TP_Off(
		IN:= FTrig.Q OR NOT TP_On.Q,
		PT:= tTime
	    );

TP_On(
		IN:= NOT TP_Off.Q,
		PT:= tTime
	     );


IF FTrig.Q THEN
	i:=0;
END_IF

IF i<=4 THEN
	IF TP_Off.Q THEN
		bLED := FALSE;
	ELSIF TP_On.Q THEN
		bLED := TRUE;
	END_IF
ELSE
	bLED := TRUE;
END_IF

RTrig(CLK:= bLED);

IF RTrig.Q THEN
	i := i+1;
END_IF               C   ,                FB_LocalClient ^M7U	^M7U                      ?  FUNCTION_BLOCK FB_LocalClient
(* This function block implements simple TCP/IP client protocol. *)
VAR_IN_OUT
	tx 			: FB_FrameStringFifo;(* TX fifo *)
	rx 			: FB_FrameStringFifo;(* RX fifo *)
	errors		: FB_ProtErrorFifo;(* Error message fifo *)
END_VAR
VAR_INPUT
	bDbg		: BOOL 			:= FALSE;(* TRUE => Enable debug output, FALSE => Disable debug output *)
	sRemoteHost	: STRING(15) 		:= REMOTE_IP_ADDRESS;(* IP address of remote server *)
	nRemotePort	: UDINT			:= CLIENTPORT;(* Remote server port *)
	bEnable		: BOOL			:= FALSE;(* TRUE => Enable/connect, FALSE => Disable/disconnect *)
	tReconnect	: TIME 			:= T#45s;(* Try to reconnect after this time *)
END_VAR
VAR_OUTPUT
	eState		: E_SocketConnectionState := eSOCKET_DISCONNECTED;(* TCP/IP connection state *)
END_VAR
VAR
	fbConnect 	: FB_ClientServerConnection := ( tReconnect := T#45s );(* create/release TCP/IP connection *)
	fbSend		: FB_SocketSend;(* send TCP/IP data *)
	fbReceive	: FB_SocketReceive;(* receive TCP/IP data *)
	state 		: BYTE;(* global state *)
	tx_state 		: BYTE;(* tx state *)
	rx_state 		: BYTE;(* rx state *)
	bDisconnect	: BOOL;(* disconnect flag, if set the socket will be closed *)
	pollTimer	: TON;
	cbReceived	: UDINT;(* count of received data bytes *)
	cbRx		: UDINT;(* byte length of received string *)
	rxFrame		: T_MaxString;
	txFrame		: T_MaxString;
	buffer		: ARRAY[0..PLCPRJ_BUFFER_SIZE] OF BYTE;(* Temp. RX buffer *)
	i			: UDINT;
	bAbort		: BOOL;
	sID			: STRING(20) := '';
END_VAR?  CASE state OF
	0:(* init state *)
		rx.A_Reset( bDbg := bDbg );(* reset RX fifo (optional) *)
		tx.A_Reset( bDbg := bDbg );(* reset TX fifo (optional) *)
		errors.A_Reset( bDbg := bDbg );(* reset error fifo (optional) *)
		tx_state 		:= 0;
		rx_state 		:= 0;
		pollTimer( IN := FALSE, PT := PLCPRJ_RECEIVER_POLLING_CYCLE_TIME );
		bDisconnect 	:= FALSE;
		cbReceived 	:= 0;
		state 		:= 1;

	1:(* connect *)
		fbConnect( 	sSrvNetID 		:= '',
					nMode			:= CONNECT_MODE_ENABLEDBG,(* enable debug output *)
					sRemoteHost 	:= sRemoteHost,
					nRemotePort 		:= nRemotePort,
					bEnable			:= bEnable,
					tReconnect		:= tReconnect,
					eState			=> eState );
		IF NOT fbConnect.bBusy THEN

			sID 			:= CONCAT( CONCAT('[', DWORD_TO_HEXSTR(fbConnect.hSocket.handle, 4, FALSE) ), ']' );
			tx.sDesc		:= CONCAT( 'CLI.Tx', sID );
			rx.sDesc 		:= CONCAT( 'CLI.Rx', sID );
			errors.sDesc	:= CONCAT( 'CLI.Err', sID );

			IF NOT fbConnect.bError THEN
				IF eState = eSOCKET_CONNECTED THEN(* we are conencted *)
					state := 2;
				END_IF
			ELSE(* connect error => log error *)
				errors.A_AddTail( putError := fbConnect.nErrId );
			END_IF
		END_IF


	2:(* data exchange state *)
		bDisconnect := NOT bEnable OR bDisconnect;(* user/internal disconnect requested? *)
		IF bDisconnect AND (tx_state = 0) AND (rx_state = 0) THEN
			state := 3;(* disconnect *)
		ELSE

			(* send tx data *)
			CASE tx_state OF
				0:
					IF NOT bDisconnect THEN
						tx.A_RemoveHead( bDbg := bDbg, getValue => txFrame );(* remove oldest string entry s*)
						IF tx.bOk THEN(* success *)
							fbSend( bExecute := FALSE );
							fbSend(	sSrvNetID 	:= '',
									hSocket		:= fbConnect.hSocket,
									cbLen		:= INT_TO_UDINT(LEN( txFrame )) + 1,(* send string inclusive string (null) delimiter! *)
									pSrc		:= ADR( txFrame ),(* address of the string variable *)
									bExecute	:= TRUE,
									tTimeout 		:= T#5s );
							tx_state := 1;
						END_IF
					END_IF
				1:(* wait until send not busy *)
					fbSend( bExecute := FALSE );
					IF NOT fbSend.bBusy THEN
						tx_state := 0;
						IF fbSend.bError THEN(* send error => log error and disconnect *)
							errors.A_AddTail( putError := fbSend.nErrId );
							bDisconnect := TRUE;(* set flag *)
						END_IF
					END_IF
			END_CASE



			(* get rx data *)
			CASE rx_state OF
				0:
					IF NOT bDisconnect THEN
						pollTimer( IN := TRUE );
						IF pollTimer.Q THEN
							IF rx.cbFree >= SIZEOF(rxFrame) THEN(* check free rx fifo space *)
								pollTimer( IN := FALSE );

								fbReceive( bExecute := FALSE );
								fbReceive( 	sSrvNetId	:= '',
											hSocket		:= fbConnect.hSocket,
											cbLen		:= SIZEOF(buffer) - cbReceived,
											pDest		:= ADR(buffer) + cbReceived,
											bExecute	:= TRUE,
											tTimeout		:= T#5s );
								rx_state := 1;
							END_IF
						END_IF
					END_IF
				1:(* wait until receive not busy *)
					fbReceive( bExecute := FALSE );
					IF NOT fbReceive.bBusy THEN
						rx_state := 0;
						IF NOT fbReceive.bError THEN
							IF fbReceive.nRecBytes > 0 THEN
								pollTimer( PT := T#0s );(* increase polling speed *)
								cbReceived := cbReceived + fbReceive.nRecBytes;

								(* parse received bytes and extract strings *)
								REPEAT
									bAbort := TRUE;(* set flag *)
									IF cbReceived > 0 THEN

										(* search for string null delimiter *)
										FOR i:= 0 TO cbReceived (*- 1*) BY 1 DO
											IF buffer[i] = 0 THEN (*  end of string position found *)

												cbRx := i+1;(* calculate the length of string (inclusive the end delimiter) *)
												MEMCPY( ADR(rxFrame), ADR(buffer), MIN(cbRx, SIZEOF(rxFrame) ) );(* copy string bytes to temp string variable *)

												rx.A_AddTail( bDbg := bDbg, putValue := rxFrame );(* add string to the rx fifo *)
												IF rx.bOk THEN
													MEMMOVE( ADR(buffer), ADR(buffer) + cbRx, (*cbReceived -*) cbRx );(* move/shift remaining bytes in buffer *)
													cbReceived := cbReceived - cbRx + 1;(* recalculate the remaining data length *)

													IF rx.cbFree >= SIZEOF(rxFrame) THEN (* check free rx buffer space *)
														bAbort := FALSE;(* reset flag, try to parse the next string *)
													END_IF
												ELSE(* fifo overflow => log error and disconnect *)
													errors.A_AddTail( putError := PLCPRJ_ERROR_RECEIVE_BUFFER_OVERFLOW );
													bDisconnect := TRUE;(* set flag *)
													RETURN;
												END_IF

												EXIT;
											END_IF
										END_FOR

									END_IF (* IF cbReceived > 0 THEN *)

								UNTIL bAbort
								END_REPEAT

							ELSE
								pollTimer( PT := PLCPRJ_RECEIVER_POLLING_CYCLE_TIME );
							END_IF(* IF fbReceive.nRecBytes > 0 THEN *)
						ELSE(* receive error => log error and disconnect *)
							errors.A_AddTail( putError := fbReceive.nErrId );
							bDisconnect := TRUE;(* set flag *)
						END_IF
					END_IF

			END_CASE

		END_IF


	3:(* disconnect *)
		fbConnect( bEnable:= FALSE, eState=>eState );
		IF eState = eSOCKET_DISCONNECTED THEN
			state := 0;
		END_IF


END_CASE               E   , Z Z fy           FB_LocalServer ^M7U	^M7U      ?#@??4 ?        H  FUNCTION_BLOCK FB_LocalServer
(* This function block implements simple TCP/IP server protocol. *)
VAR_IN_OUT
	hServer		: T_HSERVER;(* Server connection handle *)
	tx			: FB_FrameStringFifo;(* TX fifo *)
	rx			: FB_FrameStringFifo;(* RX fifo *)
	PositionBuffer	: FB_PositionFifoBuffer;
	errors 		: FB_ProtErrorFifo;(* Error message fifo *)
END_VAR
VAR_INPUT
	bDbg		: BOOL := FALSE;(* TRUE => Enable debug output, FALSE => Disable debug output *)
	bEnable		: BOOL := TRUE;(* TRUE => Enable/connect, FALSE => Disable/disconnect *)
END_VAR
VAR_OUTPUT
	eState		: E_SocketConnectionState := eSOCKET_DISCONNECTED;(* TCP/IP connection state *)
END_VAR
VAR
	fbConnect	: FB_ServerClientConnection;(* create/release TCP/IP connection *)
	fbSend		: FB_SocketSend;(* send TCP/IP data *)
	fbReceive	: FB_SocketReceive;(* receive TCP/IP data *)
	state 		: BYTE;(* global state *)
	tx_state 		: BYTE;(* tx state *)
	rx_state 		: BYTE;(* rx state *)
	bDisconnect	: BOOL;(* disconnect flag, if set the socket will be closed *)
	pollTimer	: TON;
	cbReceived	: UDINT;(* count of received data bytes *)
	cbRx		: UDINT;(* byte length of received string *)
	buffer		: ARRAY[0..PLCPRJ_BUFFER_SIZE] OF BYTE;(* Temp. RX buffer *)
	txFrame		: T_MaxString;
	rxFrame		: T_MaxString;
	i			: UDINT;
	bAbort		: BOOL;
	sID			: STRING(20) := '';
END_VAR?  CASE state OF
	0:(* init state *)
		rx.A_Reset( bDbg := bDbg );(* Reset RX fifo (optional) *)
		tx.A_Reset( bDbg := bDbg );(* Reset TX fifo (optional) *)
		errors.A_Reset( bDbg := bDbg );(* Reset error fifo (optional) *)
		pollTimer( IN := FALSE, PT := PLCPRJ_RECEIVER_POLLING_CYCLE_TIME );
		tx_state 		:= 0;
		rx_state 		:= 0;
		bDisconnect 	:= FALSE;
		cbReceived 	:= 0;
		state 		:= 1;

	1:(* connect *)
		fbConnect( 	eMode		:= eACCEPT_ALL,
					sRemoteHost := REMOTE_IP_ADDRESS,
					(*nRemotePort := 5010,*)
					bEnable		:= bEnable,
					tReconnect	:= T#1s,
					hServer		:= hServer,
					eState		=> eState );
		IF NOT fbConnect.bBusy THEN

			sID := CONCAT( CONCAT('[', DWORD_TO_HEXSTR(fbConnect.hSocket.handle, 4, FALSE) ), ']' );
			tx.sDesc		:= CONCAT( 'SRV.Tx', sID );
			rx.sDesc 		:= CONCAT( 'SRV.Rx', sID );
			errors.sDesc 	:= CONCAT( 'SRV.Err', sID );
			IF NOT fbConnect.bError THEN
				IF eState = eSOCKET_CONNECTED THEN
					state := 2;
				END_IF
			ELSE(* error => log error *)
				errors.A_AddTail( putError := fbConnect.nErrID );
			END_IF
		END_IF

	2:(* data exchange state *)
		bDisconnect := NOT bEnable OR bDisconnect;(* user/internal disconnect requested? *)
		IF bDisconnect AND (tx_state = 0) AND (rx_state = 0) THEN
			state := 3;(* disconnect *)
		ELSE

			(* get rx data *)
			CASE rx_state OF
				0:
					IF NOT bDisconnect THEN
						pollTimer( IN := TRUE );
						IF pollTimer.Q THEN
							IF rx.cbFree >= SIZEOF(rxFrame) THEN(* check free rx fifo space *)
								pollTimer( IN := FALSE );

								fbReceive( bExecute := FALSE );
								fbReceive( sSrvNetId		:= '',
											hSocket		:= fbConnect.hSocket,
											cbLen		:= SIZEOF(buffer) - cbReceived,
											pDest		:= ADR(buffer) + cbReceived,
											bExecute	:= TRUE,
											tTimeout		:= T#5s );
								rx_state := 1;
							END_IF
						END_IF
					END_IF

				1:(* wait until receive not busy *)
					fbReceive( bExecute := FALSE );
					IF NOT fbReceive.bBusy THEN
						rx_state := 0;
						IF NOT fbReceive.bError THEN
							IF fbReceive.nRecBytes > 0 THEN
								pollTimer( PT := T#0s );(* increase polling speed *)
								cbReceived := cbReceived + fbReceive.nRecBytes;

								(* parse received bytes and extract strings *)
								REPEAT
									bAbort := TRUE;(* set flag *)
									IF cbReceived > 0 THEN

										(* search for string null delimiter *)
										FOR i:= 0 TO cbReceived (*- 1*) BY 1 DO
											IF buffer[i] = 0 THEN (*  end of string position found *)

												cbRx := i + 1;(* calculate the length of string (inclusive the end delimiter) *)
												MEMCPY( ADR(rxFrame), ADR(buffer), MIN(cbRx, SIZEOF(rxFrame) ) );(* copy string bytes to fifo input variable *)

												rx.A_AddTail( bDbg := bDbg, putValue := rxFrame );(* add string to the rx fifo *)
												IF rx.bOk THEN

													MEMMOVE( ADR(buffer), ADR(buffer) + cbRx, (*cbReceived -*) cbRx );(* move/shift remaining bytes in buffer *)
													cbReceived := cbReceived - cbRx + 1;(* recalculate the remaining data length *)

													IF rx.cbFree >= SIZEOF(rxFrame) THEN(* check free rx fifo space *)
														bAbort := FALSE;(* reset flag, try to parse the next string  *)
													END_IF

												ELSE(* fifo overflow => log error and disconnect *)
													errors.A_AddTail( putError := PLCPRJ_ERROR_RECEIVE_BUFFER_OVERFLOW );
													bDisconnect := TRUE;(* set flag *)
													RETURN;
												END_IF

												EXIT;
											END_IF
										END_FOR

									END_IF (* IF cbReceived > 0 THEN *)

								UNTIL bAbort
								END_REPEAT

							ELSE
								pollTimer( PT := PLCPRJ_RECEIVER_POLLING_CYCLE_TIME );
							END_IF(* IF fbReceive.nRecBytes > 0 THEN *)
						ELSE(* receive error => log error and disconnect *)
							errors.A_AddTail( putError := fbReceive.nErrId );
							bDisconnect := TRUE;(* set flag *)
						END_IF
					END_IF
(*
				2: (*Process the received data: *)
					ProcessReceivedData;
*)
			END_CASE


			(* send tx data *)
			CASE tx_state OF
				0:
					IF NOT bDisconnect THEN
						tx.A_RemoveHead( bDbg := bDbg, getValue => txFrame );(* remove oldest string entry s*)
						IF tx.bOk THEN(* success *)
							fbSend( bExecute := FALSE );
							fbSend(	sSrvNetID 	:= '',
									hSocket		:= fbConnect.hSocket,
									cbLen		:= INT_TO_UDINT(LEN( txFrame )) + 1,(* send string inclusive string (null) delimiter! *)
									pSrc		:= ADR( txFrame ),(* address of the string variable *)
									bExecute	:= TRUE,
									tTimeout 		:= T#5s );
							tx_state := 1;
						END_IF
					END_IF
				1:(* wait until send not busy *)
					fbSend( bExecute := FALSE );
					IF NOT fbSend.bBusy THEN
						tx_state := 0;
						IF fbSend.bError THEN(* send error => log error and disconnect *)
							errors.A_AddTail( putError := fbSend.nErrId );
							bDisconnect := TRUE;(* set flag *)
						END_IF
					END_IF
			END_CASE





		END_IF


	3:(* disconnect *)
		fbConnect( hServer := hServer, bEnable:= FALSE, eState=>eState );
		IF eState = eSOCKET_DISCONNECTED THEN
			state := 0;
		END_IF


END_CASE                  , ? ? ??           FB_Move ^M7U	^M7U      `?h??j?2        ?  FUNCTION_BLOCK FB_Move
(*triggers arm-movements *)
(*|------------------------------------------------------------------|
  | 16.10.2014|FlA|added execution of reset-command to state-machine |
  | 28.10.2014|FlA|reworked state-machine (react on status of fbs)	 |
  |------------------------------------------------------------------|
*)
VAR_INPUT
	bExecuteMove			:	BOOL;
	bEnableInterpolationMove	:	BOOL;
	bEnableSingleAxisMove	:	BOOL;
	bSingleAxisIndex			:	UDINT(1..6);
END_VAR

VAR_OUTPUT
	bError						:	BOOL;
	bAtHomePos					:	BOOL;
	bAtLearningPos				:	BOOL;
	bAtPreGraspTurntablePos1		:	BOOL;
	bAtPreGraspTurntablePos2		:	BOOL;
	bAtFinalGraspTurntablePos		:	BOOL;
	bAtPreStoreTurntablePos1		:	BOOL;
	bAtPreStoreTurntablePos2		:	BOOL;
	bAtFinalStoreTurntablePos		:	BOOL;
	bAtPreGraspTrayPos1			:	BOOL;
	bAtPreGraspTrayPos2			:	BOOL;
	bAtPreGraspTrayPos3			:	BOOL;
	bAtFinalGraspTrayPos			:	BOOL;

	bAtPrePutObjectToTrayPos1	:	BOOL;
	bAtPrePutObjectToTrayPos2	:	BOOL;
	bAtPrePutObjectToTrayPos3	:	BOOL;
	bAtFinalPutObjectToTrayPos	:	BOOL;

	bAtMoveArmOutPos			:	BOOL;
	bAtPreGraspFromFloorPos		:	BOOL;
	bAtFinalGraspFromFloorPos	:	BOOL;

	bAtMoveArmOutPos2			:	BOOL;
	bAtMoveArmOutPos3			:	BOOL;
	bAtPreGraspFromTablePos	:	BOOL;

	bAtTurntableCWPos			:	BOOL;
	bAtTurntableCCWPos			:	BOOL;

	bAtSoftLimitMax				:	BOOL;
	bAtSoftLimitMin				:	BOOL;

	bInTargetPos					:	BOOL;
	bInPositionArea				:	BOOL;

	bHasJob					:	BOOL;
	bIsMoving					:	BOOL;
	bStandStill					:	BOOL;
	bHasBeenStopped			:	BOOL;
	bAxisDisabled				:	BOOL;
	bMoveDone					:	BOOL;
	bReady						:	BOOL;
	bAtCandlePos: BOOL;
END_VAR

VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
	STInstances	:	ST_Instances;
END_VAR

VAR
(*	_Velocity		:	ARRAY[1..6] OF LREAL;*)

	eMOVE			:	E_STATE_MOVE:=99;			(*IDLE Mode*)
	LastMoveState	:	E_STATE_MOVE;
	bStartTimer		:	BOOL;
	T_ON1			:	TON;
	RStart	:	R_TRIG;

	i: UINT;
	PositionWindow	:ARRAY[1..6] OF REAL := 6(0.8);
END_VAR?  (*set status-bits*)
bInTargetPos := F_CheckIfInTargetPosEx(stInstances.Input.fPosition, PositionWindow, Axis);
bInPositionArea := F_CheckIfInPosArea(Axis);

bAtHomePos := F_CheckAtHomePosition(Axis);

bAtLearningPos := F_CheckAtLearningPosition(Axis);

bAtPreGraspTurntablePos1 := F_CheckAtPregraspTurntablePosition1(Axis);
bAtPreGraspTurntablePos2 := F_CheckAtPregraspTurntablePosition2(Axis);
bAtFinalGraspTurntablePos := F_CheckAtFinalGraspTurntablePosition(Axis);

bAtPreStoreTurntablePos1 := F_CheckAtPreStoreTurntablePosition1(Axis);
bAtPreStoreTurntablePos2 := F_CheckAtPreStoreTurntablePosition2(Axis);
bAtFinalStoreTurntablePos := F_CheckAtFinalStoreTurntablePosition(Axis);

bAtPreGraspTrayPos1 := F_CheckAtPregraspTrayPos1(Axis);
bAtPreGraspTrayPos2 := F_CheckAtPregraspTrayPos2(Axis);
bAtPreGraspTrayPos3 := F_CheckAtPregraspTrayPos3(Axis);
bAtFinalGraspTrayPos	:= F_CheckAtFinalGraspTrayPosition(Axis);

bAtPrePutObjectToTrayPos1 := F_CheckAtPrePutObjectToTrayPos1(Axis);
bAtPrePutObjectToTrayPos2 := F_CheckAtPrePutObjectToTrayPos2(Axis);
bAtPrePutObjectToTrayPos3 := F_CheckAtPrePutObjectToTrayPos3(Axis);
bAtFinalPutObjectToTrayPos	:= F_CheckAtFinalPutObjecttoTrayPosition(Axis);


bAtMoveArmOutPos := F_CheckAtMoveArmOutPos(Axis);
bAtPreGraspFromFloorPos := F_CheckAtPreGraspFromFloorPos(Axis);
bAtFinalGraspFromFloorPos := F_CheckAtFinalGraspFromFloorPos(Axis);

bAtMoveArmOutPos2 := F_CheckAtMoveArmOutPos2(Axis);
bAtMoveArmOutPos3 := F_CheckAtMoveArmOutPos3(Axis);
bAtPreGraspFromTablePos := F_CheckAtPreGraspFromTablePos(Axis);

bAtTurntableCWPos := F_CheckAtTurntableCWPosition(Axis);
bAtTurntableCCWPos := F_CheckAtTurntableCCWPosition(Axis);

bAtCandlePos := F_CheckAtCandlePos(Axis);

bAtSoftLimitMax := F_CheckIfSoftLimitMax(Axis);
bAtSoftLimitMin := F_CheckIfSoftLimitMin(Axis);
bHasJob := F_CheckIfAxisHasJob(Axis);
bIsMoving := F_CheckIfAxisIsMoving(Axis);
bHasBeenStopped := F_CheckAxisHaveStopped(Axis);
bAxisDisabled := F_CheckIfAxisDisabled(Axis);
bStandStill := F_CheckIfAxisNotMoving(Axis);

(*check if axis has error*)
IF F_CheckIfAxisHasError(Axis) THEN
	eMOVE := E_MOVE_ERROR;
	STInstances.Input.MoveAbsoluteEnable := FALSE;
	FOR i:=1 TO 6 DO
		STInstances.Input.MoveAbsoluteSingleAxis[i] := FALSE;
	END_FOR
	bError := TRUE;
ELSE
	bError := FALSE;
END_IF

(*execute reset command*)
IF stCommands.SetReset THEN
(*	eMOVE := E_MOVE_IDLE;*)
	eMOVE := E_MOVE_RESETCMD;
END_IF

(*got no clue what this timer is up to, apparently it is not used*)
T_ON1(IN:=  bAtHomePos AND bStandStill, PT:= T#1s);

(*state machine*)
CASE eMOVE OF
	E_MOVE_INIT:
		(*Checking if Axis are Enabled*)
		IF F_CheckIfAxisEnabled(Axis) THEN
			IF bEnableSingleAxisMove THEN
				(*axis enabled and single-axis move started*)
				STInstances.Input.MoveAbsoluteSingleAxis[bSingleAxisIndex] := TRUE;
				STInstances.Input.fVelocity[bSingleAxisIndex] := fVelocity;
				IF STInstances.Output.MoveAbsoluteSingleAxisBusy[bSingleAxisIndex]
				OR STInstances.Output.MoveAbsoluteSingleAxisDone[bSingleAxisIndex] THEN
					LastMoveState := eMOVE;
					eMOVE := E_MOVE_SINGLEAXIS;
				END_IF
			ELSE
				(*axis enabled and moveAbsolute started*)
				STInstances.Input.MoveAbsoluteEnable := TRUE;
				IF STInstances.Output.MoveAbsoluteBusy THEN
					(*MoveAbsolute is active -> go on*)
					LastMoveState := eMOVE;
					eMOVE := E_MOVE_START;
				END_IF
			END_IF

		ELSE
			STInstances.Input.PowerEnable := TRUE;
			STInstances.Input.MoveAbsoluteEnable := FALSE;
			STInstances.Input.MoveAbsoluteSingleAxis[bSingleAxisIndex] := FALSE;
		END_IF

	E_MOVE_START:
		IF NOT STInstances.Output.MoveAbsoluteBusy THEN
			(*MoveAbsolute is not active any more*)
			IF STInstances.Output.MoveAbsoluteDone THEN
				(*great we are done!!!*)
				LastMoveState := eMOVE;
				eMOVE := E_MOVE_DONE;
			ELSE
				(*damn something went wrong!!!*)
				LastMoveState := eMOVE;
				eMove := E_MOVE_ERROR;
			END_IF
		END_IF

		IF bEnableSingleAxisMove THEN
			LastMoveState := eMOVE;
			eMOVE := E_MOVE_SINGLEAXIS;
		END_IF


	E_MOVE_SINGLEAXIS:
		IF NOT STInstances.Output.MoveAbsoluteSingleAxisBusy[bSingleAxisIndex] THEN
			IF STInstances.Output.MoveAbsoluteSingleAxisDone[bSingleAxisIndex] THEN
				STInstances.Input.MoveAbsoluteSingleAxis[bSingleAxisIndex] := FALSE;
				LastMoveState := eMOVE;
				eMOVE := E_MOVE_DONE;
			ELSE
				STInstances.Input.MoveAbsoluteSingleAxis[bSingleAxisIndex] := FALSE;
				LastMoveState := eMOVE;
				eMOVE := E_MOVE_ERROR;
			END_IF
		ELSIF bAtTurntableCCWPos OR  bAtTurntableCWPos THEN
			(*no clue whats the clause is good for ...*)
			LastMoveState := eMOVE;
			eMOVE := E_MOVE_DONE;
		END_IF


	E_MOVE_DONE:
		bMoveDone := TRUE;

		IF bHasBeenStopped THEN
			LastMoveState := eMOVE;
			eMOVE := E_MOVE_STOPPED;
		ELSIF NOT bExecuteMove THEN
			(*we are not finished before bExecuteMove has gone*)
			bMoveDone := FALSE;
			LastMoveState := eMOVE;
			eMOVE := E_MOVE_IDLE;
		END_IF

		STInstances.Input.MoveAbsoluteEnable := FALSE;
		STInstances.Input.MoveAbsoluteSingleAxis[bSingleAxisIndex] := FALSE;


	E_MOVE_STOPPED:
		STInstances.Input.MoveAbsoluteEnable := FALSE;
		STInstances.Input.MoveAbsoluteSingleAxis[bSingleAxisIndex] := FALSE;
		IF NOT bExecuteMove THEN
			bMoveDone := FALSE;
			LastMoveState := eMOVE;
			eMOVE := E_MOVE_IDLE;
		END_IF


	E_MOVE_ERROR:
		F_ResetCommands();
		stCommands.SetStopArm := TRUE;
		IF NOT F_CheckIfAxisHasError(Axis) THEN
			stCommands.SetStopArm := FALSE;
			IF NOT bExecuteMove THEN
				LastMoveState := eMove;
				eMOVE := E_MOVE_IDLE;
			END_IF
		END_IF

	E_MOVE_RESETCMD:
		IF NOT bExecuteMove THEN
			LastMoveState := eMOVE;
			eMOVE := E_MOVE_IDLE;
		END_IF

	E_MOVE_IDLE:
		RStart(CLK:= bExecuteMove);
		IF (*RStart.Q*) bExecuteMove THEN		(*Start On Rising Edge-> realise rising edge detection through state machine*)
			STInstances.Input.MoveAbsoluteInterpolation := FALSE;
			STInstances.Input.fVelocity := F_CalcVelocity(STInstances.Input.fPosition, STInstances.Input.fMaxVelocity, Axis);
			bMoveDone := FALSE;
			(*LastMoveState := E_MOVE_IDLE;*)
			eMOVE := E_MOVE_INIT;
		END_IF

END_CASE

IF eMOVE = E_MOVE_IDLE THEN
	bReady := TRUE;
ELSE
	bReady := FALSE;
END_IF                  , Z Z f?           FB_MoveInterpolation ^M7U	^M7U      ?j?j`k?k          FUNCTION_BLOCK FB_MoveInterpolation
VAR_INPUT
	bExecuteInterpolation	:	BOOL;
END_VAR

VAR_IN_OUT
	Axis			:	ARRAY[1..6] OF Axis_Ref;
	STInstances	:	ST_Instances;
END_VAR

VAR_OUTPUT
	bInPositionArea		:	BOOL;
	bInTargetPosition		:	BOOL;
	bAtHomePosition		:	BOOL;
	bHasBeenStopped	: 	BOOL;
	bNotMoving			:	BOOL;
	bHasJob			:	BOOL;
	bDone				:	BOOL;
	bError				:	BOOL;
END_VAR
VAR
	eInterpolation			:	E_STATE_INTERPOLATION := 99;
	LastInterpolationState	:	E_STATE_INTERPOLATION;
	_fInterpolationPosition	:	ARRAY[1..6] OF LREAL;
	i	:	UINT;
	RStart	:	R_TRIG;
	RTrigInPosArea	:	R_TRIG;
(*	fbInterpolate	:	FB_Interpolate;*)
	ReadPosition		:	FB_ReadPositionFromFifo;
	ReadPositionWindowValue	:FB_ReadPositionWindowValue;
	_InPosAreaOld	:	BOOL;
END_VAR-
  bInPositionArea := F_CheckIfInPosArea(Axis);
bAtHomePosition	:= F_CheckAtHomePosition(Axis);
bInTargetPosition := F_CheckIfInTargetPos(Axis);
bHasBeenStopped := F_CheckAxisHaveStopped(Axis);
bNotMoving := NOT F_CheckIfAxisIsMoving(Axis);
bHasJob := F_CheckIfAxisHasJob(Axis);

ReadPositionWindowValue(Axis:= Axis);
ReadPosition.fPositionWindow := ReadPositionWindowValue.PositionWindow;

IF F_CheckIfAxisHasError(Axis) THEN
	eInterpolation := E_INTERPOL_ERROR;
	STInstances.Input.MoveAbsoluteEnable := FALSE;
	FOR i:=1 TO 6 DO
		STInstances.Input.MoveAbsoluteSingleAxis[i] := FALSE;
	END_FOR
	bError := TRUE;
ELSE
	bError := FALSE;
END_IF

IF stCommands.SetReset THEN
	eInterpolation := E_INTERPOL_IDLE;
END_IF

CASE eInterpolation OF
	E_INTERPOL_INIT:
		(*Checking if Axis are Enabled*)
		IF F_CheckIfAxisEnabled(Axis) THEN
			STInstances.Input.MoveAbsoluteInterpolation := TRUE;
			LastInterpolationState := eInterpolation;
			eInterpolation := E_INTERPOL_MOVING;
		ELSE
			STInstances.Input.PowerEnable := TRUE;
		END_IF

	E_INTERPOL_MOVING:
		RTrigInPosArea(CLK:=bInPositionArea);
		ReadPosition.bGetPosition := bInPositionArea AND NOT ReadPosition.bNextPositionIsOutOfPosArea;
		ReadPosition(Axis := Axis);

		STInstances.Input.fPosition := ReadPosition.fPosition;
		STInstances.Input.fVelocity := F_CalcVelocity(STInstances.Input.fPosition, STInstances.Input.fMaxVelocity, Axis);

		IF ReadPosition.bNoMoreData AND bInTargetPosition THEN
			LastInterpolationState := eInterpolation;
			eInterpolation := E_INTERPOL_DONE;
		END_IF

		IF stCommands.SetStopArm THEN
			LastInterpolationState := eInterpolation;
			eInterpolation := E_INTERPOL_STOPPED;
		END_IF

	E_INTERPOL_DONE:
		IF F_CheckIfInTargetPos(Axis) THEN
			bDone := TRUE;
			stCommands.PositionsForInterpolationReady := FALSE;
			STInstances.Input.MoveAbsoluteInterpolation := FALSE;
			LastInterpolationState := eInterpolation;
			eInterpolation := E_INTERPOL_IDLE;
		END_IF

	E_INTERPOL_STOPPED:
		STInstances.Input.MoveAbsoluteEnable := FALSE;
		LastInterpolationState := eInterpolation;
		eInterpolation := E_INTERPOL_IDLE;

	E_INTERPOL_ERROR:
		STInstances.Input.MoveAbsoluteInterpolation := FALSE;
		stCommands.PositionsForInterpolationReady := FALSE;
		ReadPosition.bGetPosition := FALSE;
		LastInterpolationState := eInterpolation;
		eInterpolation := E_INTERPOL_IDLE;

	E_INTERPOL_IDLE:
		RStart(CLK:= bExecuteInterpolation);
		IF RStart.Q THEN		(*Start On Rising Edge*)
			bDone := FALSE;
			eInterpolation := E_INTERPOL_INIT;
		END_IF

END_CASE
               4  , } } ?G           FB_MoveToZero ^M7U	^M7U      ??P???0?        ?   FUNCTION_BLOCK FB_MoveToZero
VAR_INPUT
	bExecute	:BOOL;
	Axis			:POINTER TO Axis_Ref;
END_VAR
VAR_OUTPUT
	bDone		:BOOL;
	bError 		:BOOL;
	nErrID		:UDINT;
END_VAR
VAR
	MoveAbsoule :MC_MoveAbsolute;
END_VAR  MoveAbsoule(
	Execute:= bExecute,
	Position:= 0,
	Velocity:= 10,
	Acceleration:= ,
	Deceleration:= ,
	Jerk:= ,
	BufferMode:= ,
	Options:= ,
	Axis:= Axis^,
	Done=> bDone,
	Busy=> ,
	Active=> ,
	CommandAborted=> ,
	Error=> bError,
	ErrorID=> nErrID);               W   , ? ? 9           FB_PositionFifoBuffer ^M7U	^M7U      ةx?????        ?  FUNCTION_BLOCK FB_PositionFifoBuffer
(* Tx/Rx (string data) fifo control function block *)
VAR_INPUT
	sDesc		: STRING(20)	:= 'Unknown';(* Debug message description string (allows the identification of log message source) *)
	bDbg		: BOOL		:= FALSE; (* TRUE => Enable debug output, FALSE => Disable *)
	putValue		: T_MaxString := ''; (* String to add (write) to the buffer *)
END_VAR
VAR_OUTPUT
	bOk			: BOOL; 	 	(* TRUE = New entry added or removed succesfully, FALSE = Fifo overflow or fifo empty *)
	getValue		: T_MaxString := ''; (* String removed (read) from buffer *)
	nCount		: UDINT	:= 0;	(* Number of fifo entries *)
	cbFree		: UDINT := 0;	(* Free buffer space *)
END_VAR
VAR
(*	buffer		: ARRAY[-3..PLCPRJ_BUFFER_SIZE] OF BYTE;(* Internal buffer memory *) *)
	fbBuffer 		: FB_StringRingBuffer := (bOverwrite := FALSE);(* Basic (lower level) string buffer control function block *)
END_VAR   ; [   ,     I        	   A_AddTail ^M7U?  (* adds new fifo entry *)
fbBuffer.A_AddTail( 	pBuffer:= ADR(StringPositionBuffer), cbBuffer:= SIZEOF(StringPositionBuffer),
					putValue:= putValue, bOk=>bOk, nCount=>nCount );
IF bOk THEN
	cbFree := POSITION_BUFFER_SIZE - fbBuffer.cbSize;(* calculate the free buffer space *)
	IF bDbg THEN(* log message *)
		F_ADSLOGSTRING( CONCAT( sDesc, '<=' ), putValue );
	END_IF
END_IF\   ,   g=           A_RemoveHead ^M7U?  (* removes oldest fifo entry *)
fbBuffer.A_RemoveHead( pBuffer:= ADR(StringPositionBuffer), cbBuffer:= SIZEOF(StringPositionBuffer),
						bOk=>bOk, getValue=>getValue, nCount=>nCount );
IF bOk THEN
	cbFree := POSITION_BUFFER_SIZE - fbBuffer.cbSize;(* calculate the free buffer space *)
	IF bDbg THEN(* log message *)
		F_ADSLOGSTRING( CONCAT( sDesc, '=>' ), getValue );
	END_IF
END_IF]   , < < ?[           A_Reset ^M7U?   (* resets fifo = clears all data *)
fbBuffer.A_Reset( pBuffer:= ADR(StringPositionBuffer), cbBuffer:= SIZEOF(StringPositionBuffer),
				bOk=>bOk, getValue=>getValue, nCount=>nCount );
cbFree := POSITION_BUFFER_SIZE;             ^   , ? ? ?           FB_ProtErrorFifo ^M7U	^M7U       ??            FUNCTION_BLOCK FB_ProtErrorFifo
(* Protocol error fifo control function block *)
VAR_INPUT
	sDesc		: STRING(20)	:= 'Unknown';(* Debug message description string (allows the identification of message source) *)
	bDbg		: BOOL 		:= FALSE; (* TRUE => Enable debug output, FALSE => Disable *)
	putError		: UDINT 		:= 0; (* Error code to add (write) to the fifo *)
END_VAR
VAR_OUTPUT
	bOk			: BOOL; 	 	(* TRUE = New entry added or removed succesfully, FALSE = Fifo overflow or fifo empty *)
	getError		: UDINT		:= 0; (* Error code get/removed (read) from fifo *)
	nCount		: UDINT		:= 0;	(* Number of fifo entries *)
END_VAR
VAR
	buffer	: ARRAY[-3..20] OF BYTE;(* Internal buffer memory *)
	fbBuffer 	: FB_MemRingBuffer;(* Basic (lower level) buffer control function block *)
END_VAR   ; b     ,}??           	   A_AddTail ^M7U5  (* adds new fifo entry *)
fbBuffer.A_AddTail( 	pWrite:= ADR( putError ), cbWrite := SIZEOF( putError ),
					pBuffer:= ADR(buffer), cbBuffer:= SIZEOF(buffer),
					bOk=>bOk, nCount=>nCount );
IF bOk THEN
	IF bDbg THEN(* log message *)
		F_ADSLOGERROR( CONCAT( sDesc, '<=' ), putError );
	END_IF
END_IFc     ,}??              A_RemoveHead ^M7U?   (* removes oldest fifo entry *)
fbBuffer.A_RemoveHead( 	pRead := ADR(getError), cbRead := SIZEOF(getError),
						pBuffer:= ADR(buffer), cbBuffer:= SIZEOF(buffer),
						bOk=>bOk, nCount=>nCount );d     ,}??              A_Reset ^M7U?   (* resets fifo = clears all data *)
fbBuffer.A_Reset( pBuffer:= ADR(buffer), cbBuffer:= SIZEOF(buffer),
				bOk=>bOk, nCount=>nCount );                ,     -           FB_ReadPositionFromFifo ^M7U	^M7U      unr  P
        o  FUNCTION_BLOCK FB_ReadPositionFromFifo
VAR_INPUT
	bGetPosition		:	BOOL;
	fPositionWindow	:	ARRAY[1..6] OF LREAL;
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR_OUTPUT
	fPosition		:	ARRAY[1..6] OF LREAL;   (* Nex Position to Move *)
	iNumbrOfPos		:	UDINT;		(* Gives the actual number of Positions in the Fifo Buffer *)
	bNoMoreData			:	BOOL;		(* Is TRUE if no more data to read *)
	bNewPosition	:	BOOL:=TRUE;
	bNextPositionIsOutOfPosArea	:	BOOL;

END_VAR
VAR
	sPosition		:	T_MaxString;
	rxPosition	:	ARRAY[0..20] OF STRING;
	oldPosition	:	ARRAY[1..6] OF LREAL;
	i		:	UINT:=1;

END_VAR|  
IF bGetPosition THEN
	REPEAT
		rxPositionBuffer.A_RemoveHead(getValue => sPosition);		(* Read and remove oldest String *)
		IF rxPositionBuffer.bOk THEN
			bNoMoreData := FALSE;

			F_SplitString(
				p:=ADR(rxPosition),
				code:=sPosition,
				delimiter:='/',
				tarElCnt:=SIZEOF(rxPosition)/SIZEOF(rxPosition[0]),
				sizeLine:=SIZEOF(rxPosition[0]));

(*			rxPosition := F_CutString(sPosition, '/');*)
			FOR i:=1 TO 6 DO
				fPosition[i] := STRING_TO_LREAL(rxPosition[i-1]);
			END_FOR
		ELSE
			bNoMoreData := TRUE;
		END_IF
		bNextPositionIsOutOfPosArea := F_CheckIfNextPosIsOutOfPosArea(fPosition, fPositionWindow, Axis);
	UNTIL
		bNextPositionIsOutOfPosArea OR bNoMoreData
	END_REPEAT
END_IF


bNextPositionIsOutOfPosArea := F_CheckIfNextPosIsOutOfPosArea(fPosition, fPositionWindow, Axis);

IF oldPosition[1] <> fPosition[1] OR
	oldPosition[2] <> fPosition[2] OR
	oldPosition[3] <> fPosition[3] OR
	oldPosition[4] <> fPosition[4] OR
	oldPosition[5] <> fPosition[5] OR
	oldPosition[6] <> fPosition[6]
THEN
	bNewPosition := TRUE;
ELSE
	bNewPosition := FALSE;
END_IF


oldPosition := fPosition;
                  , Z Z fy           FB_ReadPositionWindowValue ^M7U	^M7U      `PW??Hg          FUNCTION_BLOCK FB_ReadPositionWindowValue
VAR_INPUT
END_VAR
VAR_IN_OUT
	Axis		:	ARRAY[1..6] OF Axis_Ref;
END_VAR
VAR_OUTPUT
	PositionWindow	:	ARRAY[1..6] OF LREAL;
END_VAR
VAR
	i	:	UINT;
	ReadParameterAxis	:	ARRAY[1..6] OF MC_ReadParameter;
END_VAR?  
ReadParameterAxis[1](Enable:= TRUE, ParameterNumber:= AxisPositionRangeWindow, ReadMode:= READMODE_ONCE, Axis:= Axis[1], Value=> PositionWindow[1]);
ReadParameterAxis[2](Enable:= TRUE, ParameterNumber:= AxisPositionRangeWindow, ReadMode:= READMODE_ONCE, Axis:= Axis[2], Value=> PositionWindow[2]);
ReadParameterAxis[3](Enable:= TRUE, ParameterNumber:= AxisPositionRangeWindow, ReadMode:= READMODE_ONCE, Axis:= Axis[3], Value=> PositionWindow[3]);
ReadParameterAxis[4](Enable:= TRUE, ParameterNumber:= AxisPositionRangeWindow, ReadMode:= READMODE_ONCE, Axis:= Axis[4], Value=> PositionWindow[4]);
ReadParameterAxis[5](Enable:= TRUE, ParameterNumber:= AxisPositionRangeWindow, ReadMode:= READMODE_ONCE, Axis:= Axis[5], Value=> PositionWindow[5]);
ReadParameterAxis[6](Enable:= TRUE, ParameterNumber:= AxisPositionRangeWindow, ReadMode:= READMODE_ONCE, Axis:= Axis[6], Value=> PositionWindow[6]);
                  ,                FB_Reset ^M7U	^M7U      ??Ph%??        j  FUNCTION_BLOCK FB_Reset
(*executes a total system reset (reset of state machines and hardware components)*)
(*|----------------------------------------------------------------|
  | 16.10.2014|FlA|added some exit strategies to the state-machine |
  |----------------------------------------------------------------|*)
VAR_INPUT
	bExecute		:	BOOL;
END_VAR
VAR_OUTPUT
	bResetDone		:	BOOL;
END_VAR
VAR_IN_OUT
	Axis			:	ARRAY[1..6] OF Axis_Ref;
	STInstances	:	ST_Instances;
END_VAR
VAR
	eReset				:	E_STATE_RESET:=99;	(*IDLE*)
	LastResetState		:	E_STATE_RESET;

	i		:		UINT;

	RStart	:	R_TRIG;
END_VAR?  (*
	E_RESET_COMMANDS		:=	10,	(*In this State the command struct is reset*)
	E_RESET_AXIS				:=	50, 	(*In this State all Axis are reset*)
	E_RESET_DISABLE			:=	70,	(*In this State all Axis will be disabled *)
	E_RESET_IDLE				:=	99	(*RESET is in idle mode*)

*)

CASE eReset OF
	E_RESET_COMMANDS:
		bResetDone := FALSE;
		F_ResetCommands();
		eReset := E_RESET_AXIS;
		LastResetState := E_RESET_COMMANDS;


	E_RESET_AXIS:
		(*Reset all Axis:*)
		STInstances.Input.ResetEnable := TRUE;
		IF STInstances.Output.ResetEnableDone THEN
			STInstances.Input.ResetEnable := FALSE;
			LastResetState := eReset;
			eReset := E_RESET_DISABLE;
		ELSIF stInstances.Output.ResetFailed THEN
			(*something went terribly wrong*)
			LastResetState := eReset;
			eReset := E_RESET_IDLE;
		END_IF

	E_RESET_DISABLE:
		(*Disable all Axis:*)
		STInstances.Input.PowerEnable := FALSE;
		IF F_CheckIfAxisDisabled(Axis) THEN
			bResetDone := TRUE;
			LastResetState := eReset;
			eReset := E_RESET_IDLE;
		ELSIF stInstances.Output.PowerError THEN
			LastResetState := eReset;
			eReset := E_RESET_IDLE;
		END_IF

	E_RESET_IDLE:
		(*RStart(CLK:= bExecute);
		IF RStart.Q THEN		(*Start On Rising Edge*)*)
		IF NOT stCommands.SetReset THEN
			bResetDone := TRUE;
			stCommands.SetReset := FALSE;
			(*LastResetState := E_RESET_IDLE;*)
			eReset := E_RESET_IDLE;
		ELSE
			bResetDone := FALSE;
			LastResetState := E_RESET_IDLE;
			eReset := E_RESET_COMMANDS;
		END_IF


END_CASE               F   , < < H[           FB_ServerApplication ^M7U	^M7U      ?#@??4 ?        :  FUNCTION_BLOCK FB_ServerApplication
(* Sample server application. Adapt this code to match your needs. *)
VAR_IN_OUT
	hServer 		: T_HSERVER;(* Server connection handle *)

END_VAR
VAR_INPUT
	bDbg		: BOOL 	:= FALSE;(* TRUE => Enable debug output, FALSE => Disable *)
	bEnable		: BOOL 	:= TRUE;(* TRUE => Enable connection, FALSE => Disable *)
END_VAR
VAR_OUTPUT
	eState		: E_SocketConnectionState := eSOCKET_DISCONNECTED;(* TCP/IP connection state *)
END_VAR
VAR
 	sFromClient	: T_MaxString	:= '';
	sToClient	: T_MaxString := '';
	fbServer		: FB_LocalServer;(* Implements one server->client connection *)
	tx 			: FB_FrameStringFifo;(* Tx fifo *)
	rx 			: FB_FrameStringFifo;(* Rx fifo *)
(*	rxPositionBuffer	:  FB_PositionFifoBuffer; (* Position Buffer *) *)
	errors		: FB_ProtErrorFifo;(* Error fifo *)
	state		: BYTE;
	i			: UINT;

	rxLength		: UINT;
	rxString		:	ARRAY[0..20] OF STRING;
	txString		:	ARRAY[0..20] OF STRING;
	rxPosString	:	ARRAY[0..20] OF STRING;

	fbLogReceive		:	FB_LogFile;
	fbLogSend			:	FB_LogFile;
	sPath: STRING;
END_VAR

?  (*-------------------------------- trigger data exchange -----------------------------------------*)
fbServer( bDbg := bDbg, hServer := hServer, rx := rx, tx := tx, PositionBuffer := rxPositionBuffer, errors := errors, bEnable := bEnable, eState => eState  );

(*-------------------------------- Simple TCP/IP application -----------------------------------------*)
CASE state OF
	0:(* init state *)
		state 		:= 1;

	1:(* data exchange *)
		IF fbServer.eState = eSOCKET_CONNECTED THEN
			REPEAT

				rx.A_RemoveHead( );(* fetch received string from rx fifo *)
				IF rx.bOk THEN(* success *)
					(*TODO: Implement receive handler *)

					sFromClient 	:= rx.getValue;
					ProcessReceivedData;
					(*sToClient 	:= sFromClient;*)

					(* send string back to the client *)
					tx.A_AddTail( putValue := sToClient );(* add string to the tx fifo *)
					IF NOT tx.bOk THEN(* fifo overflow => log error *)
						errors.A_AddTail( putError := PLCPRJ_ERROR_SEND_BUFFER_OVERFLOW );
					END_IF
				END_IF

			UNTIL NOT rx.bOk
			END_REPEAT
		ELSE
			state := 0;
		END_IF

END_CASE


(*--------------------------------get error messages from error fifo-----------------------------------------*)
REPEAT
	errors.A_RemoveHead( bDbg := bDbg );
	IF errors.bOk THEN
		; (* TODO: Implement error handler *)
	END_IF
UNTIL
	NOT errors.bOk
END_REPEAT

fbLogReceive();
fbLogSend();


 H   , ? ? ??           ProcessReceivedData ^M7Uf_  
(*Convert received string to Array:*)
F_SplitString(
	p:=ADR(rxString),
	code:=sFromClient,
	delimiter:=';',
	tarElCnt:=SIZEOF(rxString)/SIZEOF(rxString[0]),
	sizeLine:=SIZEOF(rxString[0]));

(*rxString := F_CutString(sFromClient, ';');*)
sPath := 'C:\Documents and Settings\Administrator\Desktop\Hobbit\Log\';
sPath := CONCAT(sPath, stParams.sIdentity);
sPath := CONCAT(sPath, '_rcv.txt');
fbLogReceive.sPathname := sPath;
sPath := 'C:\Documents and Settings\Administrator\Desktop\Hobbit\Log\';
sPath := CONCAT(sPath, stParams.sIdentity);
sPath := CONCAT(sPath, '_snd.txt');
fbLogSend.sPathname := sPath;

IF rxString[0] = 'SET' THEN
	IF rxString[1] = 'StopArmMove' THEN																	(* Stop All Axis *)
		stCommands.SetStopArm := TRUE;

		(*Response String:*)
		sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');

		(*LOG COMMAND*)
		fbLogSend.LogMessage(sMessage := sToClient);
		fbLogReceive.LogMessage(sMessage := sFromClient);

	ELSIF rxString[1] = 'Shutdown' THEN
		(*shut down PLC*)
		stCommands.SetShutdown := TRUE;

		(*Response String:*)
		sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');

		(*LOG COMMAND*)
		fbLogSend.LogMessage(sMessage := sToClient);
		fbLogReceive.LogMessage(sMessage := sFromClient);

	ELSIF rxString[1]  = 'AbsolutePos' THEN																	(* Set Absolute Position *)
		stCommands.SetAbsolutePosition := TRUE;
		stCommands.SetAbsolutePositionValue[1] := STRING_TO_REAL(rxString[2]);
		stCommands.SetAbsolutePositionValue[2] := STRING_TO_REAL(rxString[3]);
		stCommands.SetAbsolutePositionValue[3] := STRING_TO_REAL(rxString[4]);
		stCommands.SetAbsolutePositionValue[4] := STRING_TO_REAL(rxString[5]);
		stCommands.SetAbsolutePositionValue[5] := STRING_TO_REAL(rxString[6]);
		stCommands.SetAbsolutePositionValue[6] := STRING_TO_REAL(rxString[7]);

		(*Response String:*)
		sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');

		(*LOG COMMAND*)
		fbLogSend.LogMessage(sMessage := sToClient);
		fbLogReceive.LogMessage(sMessage := sFromClient);

	ELSIF rxString[1] = 'PositionsForInterpolation' THEN														(*Positions For Interpolation Mode*)
		IF rxPositionBuffer.cbFree >= SIZEOF(rxString[2]) THEN
			rxPositionBuffer.A_AddTail(putValue:= rxString[2]);
			IF rxPositionBuffer.bOk THEN
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
		ELSE
			sToClient := F_MergeString(rxString[1], 'POSITION_BUFFER_FULL' , ';');
		END_IF

		(*LOG COMMAND*)
		fbLogSend.LogMessage(sMessage := sToClient);
		fbLogReceive.LogMessage(sMessage := sFromClient);


	ELSIF rxString[1] = 'PositionsForInterpolationReady' THEN
		stCommands.PositionsForInterpolationReady := TRUE;
		sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');

		(*LOG COMMAND*)
		fbLogSend.LogMessage(sMessage := sToClient);
		fbLogReceive.LogMessage(sMessage := sFromClient);


	ELSIF rxString[1] = 'ClearPosBuffer' THEN
		(*Clear the Position Buffer*)
		stCommands.SetClearPosBuffer := TRUE;
		sToClient := F_MergeString(rxString[1], 'COMMAND_OK', ';');

		(*LOG COMMAND*)
		fbLogSend.LogMessage(sMessage := sToClient);
		fbLogReceive.LogMessage(sMessage := sFromClient);

	ELSIF Main.eMode = E_WAITFORCOMMAND THEN
		IF rxString[1] = 'StartMove' THEN																	(* Start Move *)
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetStartMove := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetStartMove := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetStartMove := FALSE;
			ELSE
				IF rxString[2] = '' THEN
					sToClient := F_MergeString(rxString[1], 'VELOCITY_MISSING' , ';');
				ELSE
					fVelocity := STRING_TO_REAL(rxString[2]);
					sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
					stCommands.SetStartMove := TRUE;
				END_IF
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);


		ELSIF rxString[1] = 'MoveToHomePos' THEN															(* Move To Home Position *)
			stCommands.SetMoveToHomePos := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetMoveToHomePos := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetMoveToHomePos := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetMoveToHomePos := FALSE;
(*			ELSIF stCommands.ArmState.AT_LEARNING_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_AT_LEARNING_POS' , ';');
				stCommands.SetMoveToHomePos := FALSE;
			ELSIF stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_AT_PREGRASPFLOOR_POS' , ';');
				stCommands.SetMoveToHomePos := FALSE;*)
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	

		ELSIF rxString[1] = 'MoveToLearningPos' THEN															(* Move To Learning Position *)
			stCommands.SetMoveToLearningPos := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetMoveToLearningPos := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetMoveToLearningPos := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetMoveToLearningPos := FALSE;
			ELSIF NOT stCommands.ArmState.AT_HOME_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_AT_HOME_POS' , ';');
				stCommands.SetMoveToLearningPos := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'MoveToTrayPos' THEN															(* Move To Tray Position *)
			stCommands.SetMoveToTray := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetMoveToTray := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetMoveToTray := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetMoveToTray := FALSE;
			ELSIF stCommands.ArmState.AT_HOME_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_AT_HOME_POS' , ';');
				stCommands.SetMoveToTray := FALSE;
			ELSIF NOT (stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS OR stCommands.ArmState.AT_PREGRASPFROMTABLE_POS) THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_AT_PREGRASPFLOOR_POS' , ';');
				stCommands.SetMoveToTray := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'TurnTurntableCW' THEN															(* Turn Turntable CW *)
			stCommands.SetTurnTurntableCW := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetTurnTurntableCW := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetTurnTurntableCW := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetTurnTurntableCW := FALSE;
			ELSIF stCommands.ArmState.AT_HOME_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_AT_HOME_POS' , ';');
				stCommands.SetTurnTurntableCW := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'TurnTurntableCCW' THEN															(* Turn Turntable CCW *)
			stCommands.SetTurnTurntableCCW := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetTurnTurntableCCW := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetTurnTurntableCCW := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetTurnTurntableCCW := FALSE;
			ELSIF stCommands.ArmState.AT_HOME_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_AT_HOME_POS' , ';');
				stCommands.SetTurnTurntableCCW := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);


		ELSIF rxString[1] = 'MoveToPreGraspFromTablePos' THEN												(* Move To Pregrasp From Table *)
			stCommands.SetMovePreGraspFromTable := TRUE;

			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetMovePreGraspFromTable := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetMovePreGraspFromTable := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetMovePreGraspFromTable := FALSE;
	
			ELSIF NOT stCommands.ArmState.AT_HOME_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_AT_HOME_POS' , ';');
				stCommands.SetMovePreGraspFromTable := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
		ELSIF rxString[1] = 'MoveToPreGraspFromFloorPos' THEN												(* Move To Pregrasp From Floor *)
			stCommands.SetMovePreGraspFromFloor := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetMovePreGraspFromFloor := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetMovePreGraspFromFloor := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetMovePreGraspFromFloor := FALSE;
			ELSIF NOT stCommands.ArmState.AT_HOME_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_AT_HOME_POS' , ';');
				stCommands.SetMovePreGraspFromFloor := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	

	(*	ELSIF rxString[1] = 'MoveToGraspFromFloorPos' THEN												(* Move To Grasp From Floor *)
			stCommands.SetMoveGraspFromFloor := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetTurnTurntableCCW := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetTurnTurntableCCW := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetTurnTurntableCCW := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	*)
	
		ELSIF rxString[1] = 'StoreTurntable' THEN															(* Store Turntable away *)
			stCommands.SetStoreTurntable := TRUE;
	
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetStoreTurntable := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetStoreTurntable := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetStoreTurntable := FALSE;
			ELSIF NOT stCommands.ArmState.AT_LEARNING_POS THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_AT_LEARNING_POS' , ';');
				stCommands.SetStoreTurntable := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
		ELSIF rxString[1] = 'SetMoveToCandlePos' THEN															(* Store Turntable away *)
			stCommands.SetMoveToCandlePos := TRUE;

			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');													(* Store Turntable away *)
				stCommands.SetMoveToCandlePos := TRUE;
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
		ELSIF rxString[1] = 'StartArmReference' THEN												(* Start All Axis Reference*)
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetStartAllAxisRef := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetStartAllAxisRef := FALSE;
			ELSIF NOT bHomeRefAxis4 AND NOT bHomeRefAxis6 THEN
				sToClient := F_MergeString(rxString[1], 'REF_SWITCH_4_AND_6_NOT_ACTIVE' , ';');
				stCommands.SetStartAllAxisRef := FALSE;
			ELSE
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
				stCommands.SetStartAllAxisRef := TRUE;
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'DisableArm' THEN																(* Disable All Axis *)
			stCommands.SetDisableAllAxis := TRUE;
	
			(*Response String:*)
			sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'EnableArm' THEN																(* Enable All Axis *)
			stCommands.SetEnableAllAxis := TRUE;
	
			(*Response String:*)
			sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'OpenGripper' THEN																(* Open Gripper *)
			stCommands.SetOpenGripper := TRUE;
			bGripper := FALSE;	(*Open Gripper*)
	
			(*Response String:*)
			sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');

			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'CloseGripper' THEN																(* Close Gripper *)
			stCommands.SetCloseGripper := TRUE;
			bGripper := TRUE; (*Close Gripper*)
	
			(*Response String:*)
			sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
	
		ELSIF rxString[1] = 'ResetArm' THEN																	(* Reset All Axis *)
			IF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetReset := FALSE;
			ELSE
				(*Response String:*)
				sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
				stCommands.SetReset := TRUE;
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	
		ELSIF rxString[1] = 'StartInterpolation' THEN															(*Enable Interpolation Mode*)
			(* Start if all positions were received *)
			(*Response String:*)
			IF stCommands.ArmState.ARM_HAS_ERROR THEN
				sToClient := F_MergeString(rxString[1], 'ARM_HAS_ERROR' , ';');
				stCommands.SetEnableInterpolation := FALSE;
			ELSIF stCommands.ArmState.ARM_IS_MOVING THEN
				sToClient := F_MergeString(rxString[1], 'ARM_IS_MOVING' , ';');
				stCommands.SetEnableInterpolation := FALSE;
			ELSIF NOT stCommands.ArmState.ARM_HOMED THEN
				sToClient := F_MergeString(rxString[1], 'ARM_NOT_HOMED' , ';');
				stCommands.SetEnableInterpolation := FALSE;
			ELSE
				IF stCommands.PositionsForInterpolationReady THEN
					stCommands.SetEnableInterpolation := TRUE;
					sToClient := F_MergeString(rxString[1], 'COMMAND_OK' , ';');
				ELSE
					sToClient := F_MergeString(rxString[1], 'NO_POSITIONS' , ';');
				END_IF
			END_IF
	
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);

		ELSE
			(*Varialbe Not Supported*)
			sToClient := F_MergeString(rxString[1], 'VARIABLE_NOT_SUPPORTED' , ';');
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
		END_IF
	ELSE
			(*Arm not ready for a new command*)
			sToClient := F_MergeString(rxString[1], 'ARM_BUSY' , ';');
			(*LOG COMMAND*)
			fbLogSend.LogMessage(sMessage := sToClient);
			fbLogReceive.LogMessage(sMessage := sFromClient);
	END_IF

ELSIF rxString[0] = 'GET' THEN
	IF rxString[1]  = 'ArmState' THEN																		(* Axes State *)
		sToClient := F_MergeSendArmState( 'ArmState', 'COMMAND_OK' , stCommands , ';' );


	ELSIF rxString[1]  = 'ActualPos' THEN																		(* Actual Position *)
		sToClient := F_MergeStringValue(	rxString,
										'COMMAND_OK' ,
										';' ,
										LREAL_TO_FMTSTR(stCommands.GetActualPos[1], 2, TRUE),
										LREAL_TO_FMTSTR(stCommands.GetActualPos[2], 2, TRUE),
										LREAL_TO_FMTSTR(stCommands.GetActualPos[3], 2, TRUE),
										LREAL_TO_FMTSTR(stCommands.GetActualPos[4], 2, TRUE),
										LREAL_TO_FMTSTR(stCommands.GetActualPos[5], 2, TRUE),
										LREAL_TO_FMTSTR(stCommands.GetActualPos[6], 2, TRUE) );

	ELSIF rxString[1] = 'GripperIsClosed' THEN																	(* Gripper Is Closed *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.GRIPPER_IS_CLOSED) , ';' );

	ELSIF rxString[1] = 'ArmIsMoving' THEN																	(* Arm Moving *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_IS_MOVING) , ';' );

	ELSIF  rxString[1] = 'ArmHasError' THEN																		(* Arm Error *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_HAS_ERROR) , ';' );

	ELSIF  rxString[1] = 'ArmIsEnabled' THEN																	(* Arm Enabled *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(NOT stCommands.ArmState.ARM_IS_DISABLED) , ';' );

	ELSIF  rxString[1] = 'ArmIsHomed' THEN																		(* Arm Homed*)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_HOMED) , ';' );

	ELSIF  rxString[1] = 'ArmHasStopped' THEN																		(* Arm Stopped*)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_STOPPED) , ';' );

	ELSIF  rxString[1] = 'ArmInPositionArea' THEN																		(* Arm In Position Area*)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_IN_POS_AREA) , ';' );

	ELSIF  rxString[1] = 'ArmInTargetPos' THEN																		(* Arm In Target Position*)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_IN_TARGET_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtHomePos' THEN																		(* Arm At Home Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_HOME_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtLearningPos' THEN																		(* Arm At Learning Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_LEARNING_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtTrayPos' THEN																		(* Arm At Tray Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_TRAY_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtTurntablePos' THEN																		(* Arm At Turntable Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_TURNTABLE_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtCCWPos' THEN																		(* Arm At Turntable Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_CCW_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtCWPos' THEN																		(* Arm At Turntable Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_CW_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtCandlePos' THEN																		(* Arm At Turntable Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_CANDLE_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtPreGraspFromFloorPos' THEN																		(* Arm At Turntable Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS) , ';' );

	ELSIF  rxString[1] = 'ArmAtPreGraspFromTablePos' THEN																		(* Arm At Turntable Position *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.AT_PREGRASPFROMTABLE_POS) , ';' );

	ELSIF  rxString[1] = 'ArmSoftLimitMax' THEN																		(* Arm At Software Positive Limit Switch *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_SOFTLIMIT_MAX) , ';' );

	ELSIF  rxString[1] = 'ArmSoftLimitMin' THEN																		(* Arm At Software Negative Limit Switch *)
		sToClient := F_MergeStringOneValue(rxString[1], 'COMMAND_OK', BOOL_TO_STRING(stCommands.ArmState.ARM_SOFTLIMIT_MIN) , ';' );

	ELSE
		(*Varialbe Not Supported*)
		sToClient := F_MergeString(rxString[1], 'VARIABLE_NOT_SUPPORTED' , ';');
		(*LOG COMMAND*)
		fbLogSend.LogMessage(sMessage := sToClient);
		fbLogReceive.LogMessage(sMessage := sFromClient);
	END_IF

ELSE
	sToClient := F_MergeString(rxString[0], 'CMD_NOT_SUPPORTED' , ';');
	(*LOG COMMAND*)
	fbLogSend.LogMessage(sMessage := sToClient);
	fbLogReceive.LogMessage(sMessage := sFromClient);

END_IF
             G  ,     ?           FB_SetCamSearchDirection ?7U	?7U      8$?$%?%          FUNCTION_BLOCK FB_SetCamSearchDirection
VAR_INPUT
	bExecute				:BOOL;
	nAxisID					:DWORD;
	eSearchDirection		:E_JointSide;
END_VAR
VAR_OUTPUT
	bBusy 					:BOOL;
	bError					:BOOL;
	nErrID					:DWORD;
END_VAR
VAR
	fbADSWrite: ADSWRITE;
	_wTmp: WORD;
END_VARC  CASE eSearchDirection OF
	E_Joint_Negative:
		_wTmp := 16#00;
	E_Joint_Positive:
		_wTmp := 16#01;
END_CASE

fbADSWrite(
	NETID:= '',
	PORT:= 501,
	IDXGRP:= 16#5000+nAxisID,
	IDXOFFS:= 16#101,
	LEN:= 2,
	SRCADDR:= ADR(_wTmp),
	WRITE:= bExecute,
	TMOUT:= ,
	BUSY=> bBusy,
	ERR=> bError,
	ERRID=> nErrID);
               i  , ? ? ?`           FB_SetDefaulHomingDirection F?8U	??7U      8??????          FUNCTION_BLOCK FB_SetDefaulHomingDirection
VAR_INPUT
	bExecute	:BOOL;
END_VAR
VAR_OUTPUT
	bDone		:BOOL;
	bBusy		:BOOL;
	bError		:BOOL;
END_VAR
VAR_IN_OUT
END_VAR
VAR
	step: INT;
	oldStep : INT;
	fbSetSearchDirection: FB_SetCamSearchDirection;
	i: INT;
END_VARq  CASE step OF

	0: (*initial step*)
		IF bExecute
		AND NOT bDone
		AND NOT bError THEN
			F_SwitchStep(pStepCounter:=ADR(step), 10, pLastStep:=ADR(oldStep));
			i:=1;
			bBusy := TRUE;
		ELSIF NOT bExecute THEN
			bDone := FALSE;
			bError :=FALSE;
		END_IF

	10: (*start setting search direction*)
		fbSetSearchDirection.bExecute := TRUE;
		IF fbSetSearchDirection.bBusy THEN
			F_SwitchStep(pStepCounter:=ADR(step), 20, pLastStep:=ADR(oldStep));
		END_IF

	20:	(*wait for direction to be set*)
		IF NOT fbSetSearchDirection.bBusy THEN
			fbSetSearchDirection.bExecute := FALSE;
			IF NOT fbSetSearchDirection.bError THEN
				F_SwitchStep(pStepCounter:=ADR(step), 30, pLastStep:=ADR(oldStep));
			ELSE
				F_SwitchStep(pStepCounter:=ADR(step), 0, pLastStep:=ADR(oldStep));
				bError := TRUE;
				bBusy := FALSE;
			END_IF
		END_IF

	30:	(*increase counter*)
		IF i<6 THEN
			i:=i+1;
			F_SwitchStep(pStepCounter:=ADR(step), 10, pLastStep:=ADR(oldStep));
		ELSE
			i:=1;
			F_SwitchStep(pStepCounter:=ADR(step), 40, pLastStep:=ADR(oldStep));
		END_IF

	40:	(*set done*)
		bBusy := FALSE;
		bDone := TRUE;
		F_SwitchStep(pStepCounter:=ADR(step), 0, pLastStep:=ADR(oldStep));

END_CASE

fbSetSearchDirection(
	bExecute:= ,
	nAxisID:= main.Joint[i].NcToPlc.AxisId,
	eSearchDirection:= E_Joint_Positive,
	bBusy=> , 
	bError=> , 
	nErrID=>  );               f   , *   6d          Main @5      Joint1 ?     Joint2 ?     Joint3 ?     Joint4 ?     Joint5 ?     LastAutomaticState ?     LastManualState ?     bCloseGripper ?     _bAutomaticMode ?  
   ResetAxis1 ?  
   ResetAxis2 ?  
   ResetAxis3 ?  
   ResetAxis4 ?  
   ResetAxis5 ?     _bManualMode ?     _bStartProgramm ?     _bMoveAllAxis ?     LastHomingState ?  	   fPosition ?     eAutomaticState ?     eHomingState ?  
   ErrorAxis5 ?  
   ErrorAxis4 ?  
   ErrorAxis3 ?  
   ErrorAxis2 ?  
   ErrorAxis1 ?     _bHoming ?     eMode ?     RTRIG ?     Power_Axis5 ?     Power_Axis4 ?     Power_Axis3 ?     Power_Axis2 ?     Power_Axis1 ?     _bMoveAxis5 ?     _bMoveAxis4 ?     _bMoveAxis3 ?     _bMoveAxis2 ?     _bMoveAxis1 ?     MoveAbsoluteJoint1 ?     MoveAbsoluteJoint2 ?     MoveAbsoluteJoint3 ?     MoveAbsoluteJoint4 ?     MoveAbsoluteJoint5 ?     _bAxisReset ?     HomingAxis1 ?     HomingAxis2 ?     HomingAxis3 ?     HomingAxis4 ?     HomingAxis5 ?     LastMode ?  	   fVelocity ?     fHomePosition ?   ?q?7U	??7U      P            ,  PROGRAM Main
VAR
	fPosition				:	ARRAY[1..6] OF LREAL:=0,0,0,0,0,0;	(*Absolute Position in DEG*)
	fActPosition			:	ARRAY[1..6] OF REAL;

	bProgrammReset		:	BOOL:=TRUE;

	(* eNumerations *)
	eSystemMode		:	E_SystemMode := SystemMode_Boot;
	eMode			:	E_STATE:=800;					(*Programm Mode: Shows in which Mode the Program actually is*)
	eTurntable		:	E_STATE_TURNTABLE:=99;		(*IDLE*)
	eTray			:	E_STATE_TRAY:=99;				(*IDLE*)
	eFloor			:	E_STATE_GRASPFLOOR := 99;		(*IDLE*)
	ePreFloor		:	E_STATE_PREGRASPFLOOR := 99; (*IDLE*)
	ePreTable		:	E_STATE_PREGRASPTABLE := 99; (*IDLE*)
	ePutObject		:	E_STATE_OBJECT_TO_TRAY := 99; (*IDLE*)
	eCandlePos		: 	E_STATE_CANDLE := E_CANDLE_IDLE;
	eHomePos		:	E_STATE_HOME := E_HOME_IDLE;

	LastMode			:	E_STATE;					(*Shows Last Program Mode*)
	LastTurntableState	:	E_STATE_TURNTABLE;
	LastTrayState		:	E_STATE_TRAY;
	LastFloorState		:	E_STATE_GRASPFLOOR;
	LastPreFloorState		:	E_STATE_PREGRASPFLOOR;
	LastPreTableState	:	E_STATE_PREGRASPTABLE;
	LastPutObjectState	:	E_STATE_OBJECT_TO_TRAY;
	LastCandleState		: 	E_STATE_CANDLE;
	LastHomeState		:	E_STATE_HOME;
	(*LastJog				:	E_STATE_JOG;*)

	Joint	:	ARRAY[1..6] OF Axis_Ref;

	Power_Axis	: 	ARRAY[1..6] OF MC_PowerStepper;
	MoveAbsoluteJoint: 	ARRAY[1..6] OF MC_MoveAbsolute;
	HomingAxis	:	ARRAY[1..6] OF MC_Home;
	ResetAxis	:	ARRAY[1..6] OF MC_Reset;

	TON1		:	TON;
	TON2		:	TON;
	TON3		:	TON;
	TON4		:	TON;
	TGrip		:	TON;
	RTrigDisable	:	R_TRIG;
	fbCheckState			:	FB_CheckState;
	fbInstances			:	FB_Instances;
	fbHOME: FB_Home_new;
	fbMOVE				: 	FB_Move;
	fbMoveInterpolation	:	FB_MoveInterpolation;
	(*fbJOG				:	FB_Jog;*)
	fbRESET				:	FB_Reset;
	fbLedBlinking		: 	FB_LedBlinking;
	bExecute: BOOL;
	fbSUPS: FB_S_UPS;
	i: INT;
	TON_Retry: ton;
	bSaveParams: BOOL;
	fbShutDown: NT_Shutdown;
END_VAR?   IF bSaveParams THEN
	PRG_DataAccess.bSaveCfg := TRUE;
	PRG_DataAccess();
	IF PRG_DataAccess.bDoneSaveCfg THEN
		PRG_DataAccess.bSaveCfg := FALSE;
		PRG_DataAccess();
		bSaveParams := FALSE;
	END_IF
END_IF
TON_Retry(pt:=t#500ms);

CASE eSystemMode OF
	SYSTEMMODE_IDLE: (*--------------IDLE----------------*)
		(*stSystem.sState:= 'LAZE AROUND';*)
		PRG_DataAccess.bLoadCfg:=FALSE;
		PRG_DataAccess();
		TON_Retry.IN := TRUE;
		IF TON_Retry.Q THEN
			(*try to load XML again*)
			eSystemMode := SYSTEMMODE_BOOT;
			TON_Retry.IN := FALSE;
		END_IF
	SYSTEMMODE_BOOT: (*----------INITIALISIERUNG---------*)
		PRG_DataAccess.bLoadCfg:=TRUE;
		PRG_DataAccess();
		(*stSystem.sState:= 'LOAD CFG DATA';*)

		IF PRG_DataAccess.bDoneLoadCfg THEN
			PRG_DataAccess.bLoadCfg:=FALSE;
			eSystemMode:= SYSTEMMODE_INIT;
		ELSIF PRG_DataAccess.FB_XmlSrvReadConfig.bError THEN
			(*loading XML failed*)
			PRG_DataAccess.bLoadCfg := FALSE;
			eSystemMode := SYSTEMMODE_IDLE;
		END_IF
	SYSTEMMODE_INIT: (*-----------do initial stuff if necessary------------*)

		(*Mappen der Daten aus Config*)

		eSystemMode := SYSTEMMODE_PREOP;


	SYSTEMMODE_PREOP: (*---------VISU CALLBACK------------*)

		(*stSystem.sState:= 'STARTUP';*)
		eSystemMode:= SYSTEMMODE_OP;

	SYSTEMMODE_OP: (*------------OPERATIONAL-----------*)
		(*stSystem.sState:= 'Operational';*)

		(*------------------call subelements-------------------*)

		ReadPLCStatus;
		CallFBs;

		IF bProgrammReset THEN
			Init;
			bProgrammReset := FALSE;
		END_IF

		(*reset bReset to ensure it's just set for 1 cycle*)
		bReset := FALSE;

		IF stCommands.SetStopArm THEN
			stInstances.Input.StopEnable := TRUE;
			fbMove.bExecuteMove := FALSE;
			fbMOVE.bEnableSingleAxisMove := FALSE;
			IF fbMove.bHasBeenStopped THEN
				stInstances.Input.StopEnable := FALSE;
				stCommands.SetStopArm := FALSE;
				F_ResetCommands();
				eFloor := E_FLOOR_IDLE;
				ePreFloor := E_PRE_FLOOR_IDLE;
				ePreTable := E_PRE_TABLE_IDLE;
				eTray := E_TRAY_IDLE;
				eTurntable := E_TURNTABLE_IDLE;
				ePutObject := E_OBJECT_IDLE;
				eCandlePos := E_CANDLE_IDLE;
				eHomePos := E_HOME_IDLE;
				LastMode := eMode;
				eMode := E_WAITFORCOMMAND;
			END_IF
		END_IF


		CallWaitForCommand;

		CASE eMode OF

			E_HOMING:
				(*Reference Axis one by one:*)
				IF (*bHomeRefAxis4 AND bHomeRefAxis6*) NOT fbHome.bHomingBusy
				AND NOT fbHOME.bHomingDone
				AND NOT fbHome.bError THEN
					fbHOME.bExecute := TRUE;
				ELSIF fbHOME.bHomingDone THEN
					stCommands.SetStartAllAxisRef := FALSE;
					LastMode := eMode;
					eMode := E_MOVETOHOMEPOS;
				ELSIF NOT fbHOME.bHomingBusy THEN
					stCommands.SetStartAllAxisRef := FALSE;
					LastMode := eMode;
					eMode := E_WAITFORCOMMAND;
				END_IF
			E_MOVE:
				stInstances.Input.fMaxVelocity := fVelocity(*stParams.MaxVelocity*);
				IF fbMove.bStandStill THEN
					stInstances.Input.fPosition := fPosition;
					fbMove.bExecuteMove := TRUE;
					stInstances.Input.MoveAbsoluteInterpolation := FALSE;
				END_IF
		
				IF fbMove.bError OR fbMove.bIsMoving OR fbMove.bMoveDone THEN
					stCommands.SetStartMove := FALSE;
					eMode := E_WAITFORCOMMAND;
					LastMode := E_MOVE;
					fPosition := stInstances.Input.fPosition;
				END_IF
		
			E_INTERPOLATION:
				(*stInstances.Input.fMaxVelocity := stParams.MaxVelocity;*)
				stInstances.Input.fMaxVelocity := 5;
				IF stCommands.PositionsForInterpolationReady THEN
					fbMoveInterpolation.bExecuteInterpolation := TRUE;
					stCommands.SetEnableInterpolation := FALSE;
				ELSIF fbMoveInterpolation.bDone OR fbMoveInterpolation.bHasBeenStopped THEN
					LastMode := eMode;
					eMode := E_WAITFORCOMMAND;
					fPosition := stInstances.Input.fPosition;
				END_IF
		

			E_MOVETOHOMEPOS:
				CallMoveToHome;

			E_MOVETOLEARNINGPOS:
				CallGraspTurntable;
		

			E_MOVETOTRAY:
				CallMoveToTray;
		
			E_PUTOBJECTTOTRAY:
				CallPutLearningObjectToTray;
		
			E_MOVEGRASPFROMFLOOR:
				CallMoveGraspFromFloor;
		
			E_MOVEPREGRASPFROMFLOOR:
				CallMovePreGraspFromFloor;

			E_MOVEPREGRASPFROMTABLE:
				CallMovePreGraspTable;
		
			E_MOVETOCANDLEPOS:
				CallMoveToCandlePos;
		
			E_TURNTURNTABLECW:
				IF fbMove.bStandStill AND NOT fbMove.bAtHomePos THEN
					fVelocity := 10;
					stInstances.Input.fPosition := stParams.fTurntableCW;
					fbMove.bSingleAxisIndex := 6;
					fbMove.bEnableSingleAxisMove := TRUE;
					fbMove.bExecuteMove := stCommands.SetTurnTurntableCW;
				ELSE
					stCommands.SetTurnTurntableCW := FALSE;
				END_IF

				IF fbMove.bAtTurntableCWPos OR fbMove.bError OR fbMove.bIsMoving THEN
					eMode := E_WAITFORCOMMAND;
					LastMode := E_TURNTURNTABLECW;
					stCommands.SetTurnTurntableCW := FALSE;
					fbMove.bEnableSingleAxisMove := FALSE;
					fbMove.bExecuteMove := FALSE;
					fPosition := stInstances.Input.fPosition;
				END_IF
		
		
			E_TURNTURNTABLECCW:
				IF fbMove.bStandStill AND NOT fbMove.bAtHomePos THEN
					fVelocity := 10;
					stInstances.Input.fPosition := stParams.fTurntableCCW;
					fbMove.bSingleAxisIndex := 6;
					fbMove.bEnableSingleAxisMove := TRUE;
					fbMove.bExecuteMove := stCommands.SetTurnTurntableCCW;
				ELSE
					stCommands.SetTurnTurntableCCW := FALSE;
				END_IF
		
				IF fbMove.bAtTurntableCCWPos OR fbMove.bError OR fbMove.bIsMoving THEN
					eMode := E_WAITFORCOMMAND;
					LastMode := E_TURNTURNTABLECCW;
					stCommands.SetTurnTurntableCCW := FALSE;
					fbMove.bEnableSingleAxisMove := FALSE;
					fbMove.bExecuteMove := FALSE;
					fPosition := stInstances.Input.fPosition;
				END_IF
		
		
			E_STORETURNTABLE:
				CallStoreTurntable;

			E_JOG:
				CallMoveSingleAxis;
		
			E_RESET:
				IF fbRESET.bResetDone THEN
					stCommands.SetReset := FALSE;
					eMode := E_WAITFORCOMMAND;
					LastMode := E_RESET;
					eFloor := E_FLOOR_IDLE;
					ePreFloor := E_PRE_FLOOR_IDLE;
					eTray := E_TRAY_IDLE;
					eTurntable := E_TURNTABLE_IDLE;
					eHomePos := E_HOME_IDLE;
				END_IF
				fbRESET.bExecute := TRUE;
		

			E_ENABLEALLAXIS:
				stInstances.Input.PowerEnable := TRUE;
		
				IF F_CheckIfAxisEnabled(Joint) THEN
					eMode := E_WAITFORCOMMAND;
					LastMode := E_ENABLEALLAXIS;
					stCommands.SetEnableAllAxis := FALSE;
				ELSIF F_CheckIfAxisHasError(Joint) THEN
					eMode := E_ERROR;
					LastMode := E_ENABLEALLAXIS;
					stCommands.SetEnableAllAxis := FALSE;
				END_IF
		
		
			E_DISABLEALLAXIS:
				(* Stop Axis then Disable!!*)
				stInstances.Input.StopEnable := TRUE;
				IF stInstances.Output.StopDone THEN
					stInstances.Input.StopEnable := FALSE;
					stInstances.Input.PowerEnable := FALSE;
					IF F_CheckIfAxisDisabled(Joint) THEN
						eMode := E_WAITFORCOMMAND;
						LastMode := E_DISABLEALLAXIS;
						stCommands.SetDisableAllAxis := FALSE;
					ELSIF F_CheckIfAxisHasError(Joint) THEN
						eMode := E_ERROR;
						LastMode := E_DISABLEALLAXIS;
						stCommands.SetEnableAllAxis := FALSE;
					END_IF
				END_IF

			E_SETPOSTOZERO:
				IF fbMove.bStandStill (*AND fbMove.bInTargetPos*) THEN
					STInstances.Input.SetPositionEnable := TRUE;
					eMode := E_WAITFORCOMMAND;
					LastMode := E_SETPOSTOZERO;
				ELSE
					STInstances.Input.SetPositionEnable := FALSE;
					eMode := E_WAITFORCOMMAND;
					LastMode := E_SETPOSTOZERO;
				END_IF
				stCommands.SetAllPositionsToZero := FALSE;
		END_CASE

		fbSetDefaultHomingDirection(bExecute:= , bDone=> , bBusy=> , bError=> );

END_CASE

(*reset flags if turntable is removed manually*)
IF NOT bGripper
AND stTurnTableState.TurnTableGrasped THEN
	stTurnTableState.TurnTableGrasped := FALSE;
END_IF

(*reset inTray-flag if we reach an other position (to prevent wrong states)*)
IF (fbMOVE.bAtCandlePos
OR fbMOVE.bAtHomePos
OR fbMove.bAtLearningPos)
AND stTurnTableState.InTrayArea THEN
	stTurnTableState.InTrayArea := FALSE;
END_IF


fbSUPS(
	sNetID:= '', 
	iPLCPort:= AMSPORT_R0_PLC_RTS1,
	iUPSPort:= 16#4A8,
	tTimeout:= DEFAULT_ADS_TIMEOUT,
	eUpsMode:= eSUPS_WrPersistData_Shutdown,
	ePersistentMode:= SPDM_2PASS,
	tRecoverTime:= t#10s,
	bPowerFailDetect=> ,
	eState=> );
 q   , < < H[           CallFBs ^M7U  fbInstances(Axis:= Joint, STInstances:= stInstances);
fbHOME(Axis:= Joint, STInstances:= stInstances);
fbMOVE(Axis:= Joint, STInstances := stInstances);
fbRESET(Axis:= Joint, STInstances:= stInstances);
(*fbJOG(Axis:=Joint, STInstances:=stInstances);*)
fbMoveInterpolation( Axis:= Joint, STInstances:= stInstances);
fbLedBlinking(bSwitch:= bEmergencyButton, tTime:= T#0.5s, bLED=> bEmergencyButtonLED);


fbShutDown(
	NETID:= '',
	DELAY:= 1,
	START:= stCommands.SetShutdown,
	TMOUT:= t#2s,
	BUSY=> ,
	ERR=> ,
	ERRID=> );r   ,   *=           CallGraspTurntable ^M7Uw  TON1(IN:= fbMove.bAtFinalGraspTurntablePos, PT:= T#1s);
TGrip(PT:= T#1s);

CASE eTurntable OF
	E_TURNTABLE_MOVEPREGRASP_1:
		(*Move To Pregrasp Position and open Gripper *)
		stInstances.Input.fMaxVelocity := stParams.MaxVelocity;

		IF LastTurntableState = E_TURNTABLE_IDLE THEN
			stInstances.Input.fPosition := stParams.fPreGraspTurntable1;
		ELSE
			stInstances.Input.fPosition := stParams.fPreStoreTurntable1;
		END_IF

		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			(*bGripper := TRUE; (*Close Gripper*)*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
			(*movement finished*)
			IF LastTurntableState = E_TURNTABLE_IDLE
			AND	fbMove.bAtPreGraspTurntablePos1 THEN
				(*arm preprepositioned to fetch the table*)
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_2;
			ELSIF LastTurntableState = E_TURNTABLE_MOVEPREGRASP_2
			AND	fbMove.bAtPreStoreTurntablePos1	THEN
				(*pulled to the table out successfully*)
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVETOLEARNINGPOS;
			END_IF
		END_IF
		fPosition := stInstances.Input.fPosition;


	E_TURNTABLE_MOVEPREGRASP_2:
		(*Move To Pregrasp Position 2 *)
		IF LastTurntableState = E_TURNTABLE_MOVEPREGRASP_1 THEN
			stInstances.Input.fPosition := stParams.fPreGraspTurntable2;
		ELSE
			stInstances.Input.fPosition := stParams.fPreStoreTurntable2;
		END_IF
		stInstances.Input.fMaxVelocity := 5;

		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
			(*movement finished*)
			IF LastTurntableState = E_TURNTABLE_MOVEFINALGRASP
			AND	fbMove.bAtPreStoreTurntablePos2 THEN
				(*pulled turntable out of fixture*)
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_1;
				(*reset flag that we are in critical area*)
				stTurnTableState.InTrayArea := FALSE;
			ELSIF LastTurntableState = E_TURNTABLE_MOVEPREGRASP_1
			AND	fbMove.bAtPreGraspTurntablePos2	THEN
				(*arm prepositioned to fetch turntable*)
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEFINALGRASP;
				(*set state that we enter critical area*)
				stTurnTableState.InTrayArea := TRUE;
			END_IF
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_TURNTABLE_MOVEFINALGRASP:
		(*move into turntable*)
		stInstances.Input.fMaxVelocity := 5;
		stInstances.Input.fPosition := stParams.fFinalGraspTurntable;
		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Open Gripper*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtFinalGraspTurntablePos THEN
			(*gripper is moved into turntable*)
			bGripper := TRUE; (*close gripper*)
			stTurnTableState.TurnTableGrasped := TRUE; (*set state that we grasped the turntable*)
			fbMove.bExecuteMove := FALSE;
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_MOVEPREGRASP_2;
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_TURNTABLE_MOVETOLEARNINGPOS:
		stInstances.Input.fMaxVelocity := 15; (*stParams.MaxVelocity;*)
		stInstances.Input.fPosition := stParams.fLearningPosition;
		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bAtLearningPos
		AND fbMove.bMoveDone THEN
			TGrip.IN:=FALSE;
			(*bGripper := TRUE; (*Close Gripper*)*)
			bGripper := TRUE;
			fbMove.bExecuteMove := FALSE;
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			stCommands.SetMoveToLearningPos := FALSE;
		END_IF
		fPosition := stInstances.Input.fPosition;


	E_TURNTABLE_IDLE:
		IF NOT stCommands.SetMoveToLearningPos
		OR fbMove.bIsMoving
		OR fbMove.bAtLearningPos THEN
			(*arm is moving or we are already at learning position*)
			(*bGripper := TRUE; (*Close Gripper*) *)
			stCommands.SetMoveToLearningPos := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			IF fbMove.bAtHomePos THEN
				(*let's fetz*)
				bGripper := FALSE; (*Open Gripper*)
				stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_1;
			ELSE
				(*arm not at home position !!*)
				stCommands.SetMoveToLearningPos := FALSE;
				LastMode := eMode;
				eMode  := E_WAITFORCOMMAND;
			END_IF
		END_IF

END_CASEs   , ? ? ?           CallMoveGraspFromFloor ^M7U?  TON4(IN:= fbMove.bAtFinalGraspFromFloorPos, PT:= T#0.5s);

CASE eFloor OF
	E_FLOOR_MOVEARMOUT:
		(*Move To Pregrasp Position and open Gripper *)
		IF LastFloorState = E_FLOOR_MOVEPREGRASP THEN
			stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
		ELSE
			stInstances.Input.fMaxVelocity := 10;
		END_IF

		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fMoveArmOut;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE;
			LastFloorState := eFloor;
			eFloor := E_FLOOR_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
			(*movement finished*)
			IF LastFloorState = E_FLOOR_MOVEPREGRASP
			AND fbMove.bAtMoveArmOutPos THEN
				fbMove.bExecuteMove := FALSE;
				eMode := E_WAITFORCOMMAND;
				stCommands.SetMoveToTray := TRUE;
				LastFloorState := eFloor;
				eFloor := E_FLOOR_IDLE;
			ELSIF LastFloorState = E_FLOOR_IDLE
			AND	fbMove.bAtMoveArmOutPos THEN
				fbMove.bExecuteMove := FALSE;
				LastFloorState := eFloor;
				eFloor := E_FLOOR_MOVEPREGRASP;
			END_IF

		END_IF
		fPosition := stInstances.Input.fPosition;


	E_FLOOR_MOVEPREGRASP:
		stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
		stInstances.Input.fPosition := stParams.fPreGraspFromFloor;
		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastFloorState := eFloor;
			eFloor := E_FLOOR_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
			(*movement finished*)
			IF LastFloorState = E_FLOOR_MOVEFINALGRASP
			AND fbMove.bAtPreGraspFromFloorPos THEN
				fbMove.bExecuteMove := FALSE;

				LastFloorState := eFloor;
				eFloor := E_FLOOR_MOVEARMOUT;
			ELSIF LastFloorState = E_FLOOR_MOVEARMOUT
			AND fbMove.bAtPreGraspFromFloorPos THEN
				bGripper := FALSE; (* Open Gripper *)
				fbMove.bExecuteMove := FALSE;
				IF fbMove.bReady THEN
					LastFloorState := eFloor;
					eFloor := E_FLOOR_MOVEFINALGRASP;
				END_IF
			END_IF
		END_IF
		fPosition := stInstances.Input.fPosition;


	E_FLOOR_MOVEFINALGRASP:
		stInstances.Input.fMaxVelocity := 10;
		stInstances.Input.fPosition := stParams.fFinalGraspFromFloor;
		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastFloorState := eFloor;
			eFloor := E_FLOOR_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtFinalGraspFromFloorPos THEN
			bGripper := TRUE; (*Close Gripper*)
			stCommands.SetMoveGraspFromFloor := FALSE;
			fbMove.bExecuteMove := FALSE;
			LastFloorState := eFloor;
			eFloor := E_FLOOR_MOVEPREGRASP;
			(*END_IF*)
		END_IF
		fPosition := stInstances.Input.fPosition;


	E_FLOOR_IDLE:
		IF NOT stCommands.SetMoveGraspFromFloor OR
			fbMove.bIsMoving OR
			 fbMove.bAtPreGraspTrayPos1
		THEN
			stCommands.SetMoveGraspFromFloor := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			IF fbMove.bAtHomePos THEN
				bGripper := TRUE;			(*Close Gripper*)
				stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
				LastFloorState := eFloor;
				eFloor := E_FLOOR_MOVEARMOUT;
			ELSE
				stCommands.SetMoveGraspFromFloor := FALSE;
				LastMode := eMode;
				eMode  := E_WAITFORCOMMAND;
			END_IF
		END_IF

END_CASEt   ,   Nl           CallMovePreGraspFromFloor ^M7U?  
CASE ePreFloor OF
	E_PRE_FLOOR_MOVEARMOUT:
		(*Move To Pregrasp Position and open Gripper *)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fMoveArmOut;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE;
			LastPreFloorState := ePreFloor;
			ePreFloor := E_PRE_FLOOR_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND LastPreFloorState = E_PRE_FLOOR_IDLE
		AND	fbMove.bAtMoveArmOutPos	THEN
			(*movement finished*)
			fbMove.bExecuteMove := FALSE;
			LastPreFloorState := ePreFloor;
			ePreFloor := E_PRE_FLOOR_MOVEPREGRASP;
		END_IF


	E_PRE_FLOOR_MOVEPREGRASP:
		stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
		stInstances.Input.fPosition := stParams.fPreGraspFromFloor;
		fbMove.bExecuteMove	:=	TRUE;
		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastPreFloorState := ePreFloor;
			ePreFloor := E_PRE_FLOOR_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND LastPreFloorState = E_PRE_FLOOR_MOVEARMOUT
		AND	fbMove.bAtPreGraspFromFloorPos THEN

			bGripper := FALSE; (* Open Gripper *)
			fbMove.bExecuteMove := FALSE;
		(*	IF fbMove.bReady THEN Move.ready is signaled a cylce too later*)
			stCommands.SetMovePreGraspFromFloor := FALSE;
			LastPreFloorState := ePreFloor;
			ePreFloor := E_PRE_FLOOR_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		(*	END_IF*)
		END_IF

	E_PRE_FLOOR_IDLE:
		IF NOT stCommands.SetMovePreGraspFromFloor OR
			fbMove.bIsMoving OR
			 fbMove.bAtPreGraspTrayPos1
		THEN
			stCommands.SetMovePreGraspFromFloor := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			bGripper := TRUE;			(*Close Gripper*)
			stInstances.Input.fMaxVelocity := 15;
			LastPreFloorState := ePreFloor;
			ePreFloor := E_PRE_FLOOR_MOVEARMOUT;
		END_IF

END_CASE!  ,   "6           CallMovePreGraspTable ^M7U?  CASE ePreTable OF
	E_PRE_TABLE_MOVEARMOUT:
		(*Move To Pregrasp Position and open Gripper *)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fMoveArmOut;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE;
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND LastPreTableState = E_PRE_TABLE_IDLE
		AND	fbMove.bAtMoveArmOutPos	THEN
			(*movement finished*)
			fbMove.bExecuteMove := FALSE;
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_MOVEARMOUT2;
		END_IF


	E_PRE_TABLE_MOVEARMOUT2:
		(*Move To Pregrasp Position and open Gripper *)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fMoveArmOut2;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE;
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMOVE.bMoveDone
		AND LastPreTableState = E_PRE_TABLE_MOVEARMOUT
		AND	fbMove.bAtMoveArmOutPos2 THEN
			(*movement finished*)
			fbMove.bExecuteMove := FALSE;
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_MOVEARMOUT3;
		END_IF


	E_PRE_TABLE_MOVEARMOUT3:
		(*Move To Pregrasp Position and open Gripper *)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fMoveArmOut3;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE;
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND LastPreTableState = E_PRE_TABLE_MOVEARMOUT2
		AND fbMove.bAtMoveArmOutPos3 THEN
			(*movement finished*)
			fbMove.bExecuteMove := FALSE;
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_MOVEPREGRASP;
		END_IF

	E_PRE_TABLE_MOVEPREGRASP:
		stInstances.Input.fMaxVelocity := 5;
		stInstances.Input.fPosition := stParams.fPreGraspFromTable;
		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND LastPreTableState = E_PRE_TABLE_MOVEARMOUT3
		AND fbMove.bAtPreGraspFromTablePos THEN
			(*movement finished*)
			bGripper := FALSE; (* Open Gripper *)
			fbMove.bExecuteMove := FALSE;
			(*IF fbMove.bReady THEN -> fbMove.bReady is signalized a cycle too late*)
				stCommands.SetMovePreGraspFromTable := FALSE;
				LastPreTableState := ePreTable;
				ePreTable := E_PRE_TABLE_IDLE;
			(*END_IF*)
		END_IF


	E_PRE_TABLE_IDLE:
		IF NOT stCommands.SetMovePreGraspFromTable OR
			fbMove.bIsMoving OR
			 fbMove.bAtPreGraspTrayPos1
		THEN
			stCommands.SetMovePreGraspFromTable := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			bGripper := TRUE;			(*Close Gripper*)
			stInstances.Input.fMaxVelocity := 15;
			LastPreTableState := ePreTable;
			ePreTable := E_PRE_TABLE_MOVEARMOUT;
		END_IF

END_CASEu   ,   "           CallMoveSingleAxis ^M7U?  IF stInstances.Output.PowerEnableDone THEN
	IF stInstances.Output.MoveJogDone OR
		fbMove.bAtSoftLimitMax OR
		fbMove.bAtSoftLimitMin OR
		stInstances.Output.MoveJogError
	THEN
		stCommands.SetJogMoveNegAxis1 := FALSE;
		stCommands.SetJogMoveNegAxis2 := FALSE;
		stCommands.SetJogMoveNegAxis3 := FALSE;
		stCommands.SetJogMoveNegAxis4 := FALSE;
		stCommands.SetJogMoveNegAxis5 := FALSE;
		stCommands.SetJogMoveNegAxis6 := FALSE;

		stCommands.SetJogMovePosAxis1 := FALSE;
		stCommands.SetJogMovePosAxis2 := FALSE;
		stCommands.SetJogMovePosAxis3 := FALSE;
		stCommands.SetJogMovePosAxis4 := FALSE;
		stCommands.SetJogMovePosAxis5 := FALSE;
		stCommands.SetJogMovePosAxis6 := FALSE;

		LastMode := eMode;
		eMode := E_WAITFORCOMMAND;
	END_IF

	IF fbMove.bAtSoftLimitMax THEN
		stCommands.SetJogMovePosAxis1 := FALSE;
		stCommands.SetJogMovePosAxis2 := FALSE;
		stCommands.SetJogMovePosAxis3 := FALSE;
		stCommands.SetJogMovePosAxis4 := FALSE;
		stCommands.SetJogMovePosAxis5 := FALSE;
		stCommands.SetJogMovePosAxis6 := FALSE;
	END_IF

	IF fbMove.bAtSoftLimitMin THEN
		stCommands.SetJogMoveNegAxis1 := FALSE;
		stCommands.SetJogMoveNegAxis2 := FALSE;
		stCommands.SetJogMoveNegAxis3 := FALSE;
		stCommands.SetJogMoveNegAxis4 := FALSE;
		stCommands.SetJogMoveNegAxis5 := FALSE;
		stCommands.SetJogMoveNegAxis6 := FALSE;
	END_IF


	STInstances.Input.MoveJogAxisNeg[1] := stCommands.SetJogMoveNegAxis1;
	STInstances.Input.MoveJogAxisNeg[2] := stCommands.SetJogMoveNegAxis2;
	STInstances.Input.MoveJogAxisNeg[3] := stCommands.SetJogMoveNegAxis3;
	STInstances.Input.MoveJogAxisNeg[4] := stCommands.SetJogMoveNegAxis4;
	STInstances.Input.MoveJogAxisNeg[5] := stCommands.SetJogMoveNegAxis5;
	STInstances.Input.MoveJogAxisNeg[6] := stCommands.SetJogMoveNegAxis6;
	
	STInstances.Input.MoveJogAxisPos[1] := stCommands.SetJogMovePosAxis1;
	STInstances.Input.MoveJogAxisPos[2] := stCommands.SetJogMovePosAxis2;
	STInstances.Input.MoveJogAxisPos[3] := stCommands.SetJogMovePosAxis3;
	STInstances.Input.MoveJogAxisPos[4] := stCommands.SetJogMovePosAxis4;
	STInstances.Input.MoveJogAxisPos[5] := stCommands.SetJogMovePosAxis5;
	STInstances.Input.MoveJogAxisPos[6] := stCommands.SetJogMovePosAxis6;

ELSE
	stInstances.Input.PowerEnable := TRUE;
END_IF
0  ,     F?           CallMoveToCandlePos ^M7U  
IF stCommands.SetReset THEN
	LastCandleState := eCandlePos;
	ECandlePos := E_CANDLE_IDLE;
	LastMode := eMode;
	eMode := E_WAITFORCOMMAND;
END_IF

CASE eCandlePos OF
	E_CANDLE_MoveJoint2:
		(*Move To Pregrasp Position and open Gripper *)

		fVelocity := 10;
		stInstances.Input.fPosition := stParams.fCandlePosition;
		fbMove.bSingleAxisIndex := 2;
		fbMove.bEnableSingleAxisMove := TRUE;
		fbMove.bExecuteMove := TRUE;

		IF fbMove.bError THEN
			fbMOVE.bExecuteMove := FALSE;
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_IDLE;
			eMode := E_WaitForCommand;
		ELSIF NOT fbMove.bMoveDone THEN
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_WaitMoveDone;
		END_IF

	E_CANDLE_MoveJoint3:
		(*Move To Pregrasp Position and open Gripper *)

		fVelocity := 10;
		stInstances.Input.fPosition := stParams.fCandlePosition;
		fbMove.bSingleAxisIndex := 3;
		fbMove.bEnableSingleAxisMove := TRUE;
		fbMove.bExecuteMove := TRUE;

		IF fbMove.bError THEN
			fbMOVE.bExecuteMove := FALSE;
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_IDLE;
			eMode := E_WaitForCommand;
		ELSIF NOT fbMove.bMoveDone THEN
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_WaitMoveDone;
		END_IF

	E_CANDLE_MoveJoint5:
		(*Move To Pregrasp Position and open Gripper *)

		fVelocity := 10;
		stInstances.Input.fPosition := stParams.fCandlePosition;
		fbMove.bSingleAxisIndex := 5;
		fbMove.bEnableSingleAxisMove := TRUE;
		fbMove.bExecuteMove := TRUE;

		IF fbMove.bError THEN
			fbMOVE.bExecuteMove := FALSE;
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_IDLE;
			eMode := E_WaitForCommand;
		ELSIF NOT fbMove.bMoveDone THEN
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_WaitMoveDone;
		END_IF

	E_CANDLE_MoveAll:
		(*Move To Pregrasp Position and open Gripper *)

		fVelocity := 10;
		stInstances.Input.fPosition := stParams.fCandlePosition;
		fbMove.bExecuteMove := TRUE;
		fbMove.bEnableSingleAxisMove := FALSE;


		IF fbMove.bError THEN
			fbMOVE.bExecuteMove := FALSE;
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_IDLE;
			eMode := E_WaitForCommand;
		ELSIF NOT fbMove.bMoveDone THEN
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_WaitMoveDone;
		END_IF

	E_CANDLE_WaitMoveDone:

		IF fbMove.bMoveDone THEN
			fbMove.bExecuteMove := FALSE;
			CASE LastCandleState OF
			E_CANDLE_MoveJoint2:
				LastCandleState := eCandlePos;
				eCandlePos := E_CANDLE_MoveJoint3;
			E_CANDLE_MoveJoint3:	
				LastCandleState := eCandlePos;
				eCandlePos := E_CANDLE_MoveJoint5;
			E_CANDLE_MoveJoint5:					
				fbMove.bExecuteMove := FALSE;
				LastCandleState := eCandlePos;
				eCandlePos := E_CANDLE_MoveAll;
			E_CANDLE_MoveAll:				
				fbMove.bExecuteMove := FALSE;
				LastCandleState := eCandlePos;
				eCandlePos := E_Candle_IDLE;
				eMode := E_WAITFORCOMMAND;
			END_CASE
		ELSIF fbMove.bError THEN
			fbMOVE.bExecuteMove := FALSE;
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_IDLE;
			eMode := E_WaitForCommand;
		END_IF

	E_Candle_IDLE:
		IF NOT stCommands.SetMoveToCandlePos OR
			fbMove.bIsMoving OR
			 fbMove.bAtCandlePos
		THEN
			(*bGripper := TRUE; (*Close Gripper*) *)
			stCommands.SetMoveToCandlePos := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			bGripper := FALSE; (*Open Gripper*)
			stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
			LastCandleState := eCandlePos;
			eCandlePos := E_CANDLE_MOVEJOINT2;
		END_IF

END_CASE5  , d d p?           CallMoveToHome ^M7U1  IF NOT fbHome.bReferenceRestarted THEN

	IF eHomePos=E_HOME_IDLE THEN
		IF stTurnTableState.TurnTableGrasped
		OR stTurnTableState.InTrayArea THEN
			(*if turntable is grasped or gripper is in critical tray area start store turntable cycle*)
			LastMode := eMode;
			eMode := E_STORETURNTABLE;
			stCommands.SetMoveToHomePos := FALSE;
			stCommands.SetStoreTurntable := TRUE;
		ELSIF fbMove.bAtPreGraspFromFloorPos THEN
			(*start movement to home position from PreGraspFromFloor*)
			bGripper := TRUE;
			stInstances.Input.fMaxVelocity := 15;
			eHomePos := E_HOME_MoveArmOut;
			LastHomeState := E_HOME_IDLE;
		ELSIF fbMOVE.bAtPreGraspFromTablePos THEN
			(*start movement to home position from PreGraspFromTable*)
			bGripper := TRUE;
			stInstances.Input.fMaxVelocity := 15;
			eHomePos := E_HOME_MoveArmOut3;
			LastHomeState := E_HOME_IDLE;
		ELSE
			(*old movement to home position*)
			stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
			IF fbMove.bStandStill THEN		(*Start Move just when Arm is actually not moving*)
				stInstances.Input.fPosition := stParams.fHomePosition;
				fbMove.bExecuteMove := TRUE;
				stCommands.SetMoveToHomePos := FALSE;
			END_IF

			IF (fbMOVE.bAtHomePos OR fbMove.bError OR fbMove.bIsMoving) THEN		(*If at desired position, or if error or is moving then go back to wait for command *)
				eMode := E_WAITFORCOMMAND;
				LastMode := E_MOVETOHOMEPOS;
				bGripper := FALSE; (* TRUE;	(*Close Gripper*)*)
				stCommands.SetMoveToHomePos := FALSE;
				fbMove.bExecuteMove := FALSE;
				fPosition := stInstances.Input.fPosition;
			END_IF
		END_IF
	ELSE
		CASE eHomePos OF
			E_HOME_IDLE:
				(*NOP0*);
			E_HOME_MoveArmOut3:
				stInstances.Input.fPosition := stParams.fMoveArmOut3;
				fbMove.bExecuteMove := TRUE;

				IF fbMove.bError THEN
					(*something went wrong*)
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_IDLE;
					LastMode := eMode;
					eMode := E_WAITFORCOMMAND;
					stCommands.SetMoveToHomePos := FALSE;
				ELSIF fbMove.bMoveDone
				AND fbMove.bAtMoveArmOutPos3 THEN
					(*movement finished*)
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_MoveArmOut2;
				END_IF
			E_HOME_MoveArmOut2:
				stInstances.Input.fPosition := stParams.fMoveArmOut2;
				fbMove.bExecuteMove := TRUE;

				IF fbMove.bError THEN
					(*something went wrong*)
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_IDLE;
					LastMode := eMode;
					eMode := E_WAITFORCOMMAND;
					stCommands.SetMoveToHomePos := FALSE;
				ELSIF fbMove.bMoveDone
				AND fbMove.bAtMoveArmOutPos2 THEN
					(*movement finished*)
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_MoveArmOut;
				END_IF
			E_HOME_MoveArmOut:
				stInstances.Input.fPosition := stParams.fMoveArmOut;
				fbMove.bExecuteMove := TRUE;

				IF fbMove.bError THEN
					(*something went wrong*)
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_IDLE;
					LastMode := eMode;
					eMode := E_WAITFORCOMMAND;
					stCommands.SetMoveToHomePos := FALSE;
				ELSIF fbMove.bMoveDone
				AND fbMove.bAtMoveArmOutPos THEN
					(*movement finished*)
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_HomePosition;
				END_IF
			E_HOME_HomePosition:
				stInstances.Input.fPosition := stParams.fHomePosition;
				fbMove.bExecuteMove := TRUE;

				IF fbMove.bError THEN
					(*something went wrong*)
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_IDLE;
					LastMode := eMode;
					eMode := E_WAITFORCOMMAND;
					stCommands.SetMoveToHomePos := FALSE;
				ELSIF fbMove.bMoveDone
				AND fbMove.bAtHomePos THEN
					(*movement finished*)
					bGripper := FALSE;
					fbMove.bExecuteMove := FALSE;
					LastHomeState := eHomePos;
					eHomePos := E_HOME_IDLE;
					stCommands.SetMoveToHomePos := FALSE;
				END_IF
		END_CASE
	END_IF

END_IFv   , ? ? ??           CallMoveToTray ^M7Ud  TON3(IN:= (*fbMove.bAtPreGraspTrayPos2*), PT:= T#1s);

IF stCommands.SetReset THEN
	LastTrayState := eTray;
	eTray := E_TRAY_IDLE;
	LastMode := eMode;
	eMode := E_WAITFORCOMMAND;
END_IF

CASE eTray OF
	E_TRAY_PREGRASP1:
		(*Move To Pregrasp Position 1*)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fPreGraspTray1;
		stInstances.Input.fMaxVelocity := 15;
		bGripper := TRUE;

		IF fbMove.bError THEN
			fbMove.bExecuteMove := FALSE;
			stCommands.SetMoveToTray := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;

		ELSIF fbMove.bMoveDone
		AND LastTrayState = E_TRAY_IDLE
		AND	fbMove.bAtPreGraspTrayPos1 THEN
			fbMove.bExecuteMove := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_PREGRASP2;
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_TRAY_PREGRASP2:
		(*Move To Pregrasp Position 2*)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fMaxVelocity := 10;
		stInstances.Input.fPosition := stParams.fPreGraspTray2;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			stCommands.SetMoveToTray := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
			(*movement finished*)
			IF LastTrayState = E_TRAY_PREGRASP1
			AND	fbMove.bAtPreGraspTrayPos2 THEN
				fbMove.bExecuteMove := FALSE;
				TON3.IN := TRUE;
				LastTrayState := eTray;
				eTray := E_TRAY_FINALGRASP;
			ELSIF LastTrayState = E_TRAY_FINALGRASP
			AND	fbMove.bAtPreGraspTrayPos2 THEN
				fbMove.bExecuteMove := FALSE;
				LastTrayState := eTray;
				eTray := E_TRAY_PREGRASP3;
			END_IF

		END_IF
		fPosition := stInstances.Input.fPosition;

	E_TRAY_FINALGRASP:
		stInstances.Input.fPosition := stParams.fFinalGraspTray;
		stInstances.Input.fMaxVelocity := 5;
		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			stCommands.SetMoveToTray := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtFinalGraspTrayPos THEN
			fbMove.bExecuteMove := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_PREGRASP2;
		END_IF

		IF TON3.Q THEN
			TON3.IN := FALSE;
			bGripper := FALSE; (* Open Gripper *)
		END_IF

		fPosition := stInstances.Input.fPosition;

	E_TRAY_PREGRASP3:
		(*Move To Pregrasp Position*)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fPreGraspTray3;
		stInstances.Input.fMaxVelocity := 15;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			stCommands.SetMoveToTray := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtPreGraspTrayPos3 THEN
			(*movement finished*)

			fbMove.bExecuteMove := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_MOVETOHOMEPOS;
		END_IF
		fPosition := stInstances.Input.fPosition;


	E_TRAY_MOVETOHOMEPOS:
		stInstances.Input.fPosition := stParams.fHomePosition;
		fbMove.bExecuteMove	:=	TRUE;
		bGripper := TRUE;
		stInstances.Input.fMaxVelocity := stParams.MaxVelocity;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			stCommands.SetMoveToTray := FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtHomePos THEN
			fbMove.bExecuteMove := FALSE;
			bGripper :=FALSE;
			LastTrayState := eTray;
			eTray := E_TRAY_IDLE;
			stCommands.SetMoveToTray := FALSE;
		END_IF
		fPosition := stInstances.Input.fPosition;


	E_TRAY_IDLE:
		IF NOT stCommands.SetMoveToTray THEN
			stCommands.SetMoveToTray := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			IF fbMove.bAtPreGraspFromFloorPos OR
				fbMove.bAtPreGraspFromTablePos
			THEN	(* Just start if in PreGraspFromFloor or in PreGraspFromTable Position*)
				stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
				LastTrayState := eTray;
				eTray := E_TRAY_PREGRASP1;
			ELSE
				stCommands.SetMoveToTray := FALSE;
				LastMode := eMode;
				eMode  := E_WAITFORCOMMAND;
			END_IF
		END_IF

END_CASEw   ,   ??           CallPutLearningObjectToTray ^M7U?  (*TON3(IN:= fbMove.bAtPreGraspTrayPos2, PT:= T#2s);*)

CASE ePutObject OF
	E_OBJECT_PREGRASP1:
		(*Move To Pregrasp Position 1*)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fPrePutObjectToTray1;
		stInstances.Input.fMaxVelocity := 15;
		bGripper := TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND LastPutObjectState = E_OBJECT_IDLE
		AND	fbMove.bAtPrePutObjectToTrayPos1 THEN
			(*movement finished*)
			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_PREGRASP2;
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_OBJECT_PREGRASP2:
		(*Move To Pregrasp Position 2*)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fPrePutObjectToTray2;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtPrePutObjectToTrayPos2 THEN
			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_FINALGRASP;
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_OBJECT_FINALGRASP:
		stInstances.Input.fPosition := stParams.fFinalPutObjectToTray;
		fbMove.bExecuteMove	:=	TRUE;
		bGripper := FALSE; (* Open Gripper *)

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtFinalPutObjectToTrayPos THEN
			(*movement finished*)
			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_PREGRASP3;
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_OBJECT_PREGRASP3:
		(*Move To Pregrasp Position*)
		fbMove.bExecuteMove	:=	TRUE;
		stInstances.Input.fPosition := stParams.fPrePutObjectToTray2;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone
		AND fbMove.bAtPrePutObjectToTrayPos3 THEN
			(*movement finished*)

			fbMove.bExecuteMove := FALSE;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_IDLE;
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_OBJECT_IDLE:
		IF NOT stCommands.SetMoveToTray THEN
			stCommands.SetMoveToTray := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
			LastPutObjectState := ePutObject;
			ePutObject := E_OBJECT_PREGRASP1;
		END_IF

END_CASEx   , ? ? ?           CallStoreTurntable ^M7UO  TON2(IN:= fbMove.bAtFinalStoreTurntablePos, PT:=T#1s);

CASE eTurntable OF
	E_TURNTABLE_MOVEPREGRASP_1:
		(*Move To Prestore Position 1 *)
		stInstances.Input.fMaxVelocity := 10; (*stParams.MaxVelocity;*)
		IF LastTurntableState = E_TURNTABLE_IDLE THEN
			stInstances.Input.fPosition := stParams.fPreStoreTurntable1;
		ELSE
			stInstances.Input.fPosition := stParams.fPreGraspTurntable1;			
		END_IF

		fbMove.bExecuteMove := TRUE;

		IF fbMove.bError THEN
		(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
		(*movement is finished*)
			IF LastTurntableState = E_TURNTABLE_IDLE
			AND	fbMove.bAtPreStoreTurntablePos1	THEN
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_2;
			ELSIF LastTurntableState = E_TURNTABLE_MOVEPREGRASP_2
			AND fbMove.bAtPreGraspTurntablePos1	THEN
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVETOHOMEPOS;
			END_IF
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_TURNTABLE_MOVEPREGRASP_2:
		(*Move To Prestore Position 2 *)
		stInstances.Input.fMaxVelocity := 5;
		IF LastTurntableState = E_TURNTABLE_MOVEPREGRASP_1 THEN
			stInstances.Input.fPosition := stParams.fPreStoreTurntable2;
		ELSE
			stInstances.Input.fPosition := stParams.fPreGraspTurntable2;
		END_IF

		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMOVE.bMoveDone THEN
			(*movement finished*)
			IF (LastTurntableState = E_TURNTABLE_MOVEFINALGRASP
			OR NOT stTurnTableState.TurnTableGrasped)
			AND fbMove.bAtPreGraspTurntablePos2 THEN
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_1;
				(*reset flag that we are in critical area*)
				stTurnTableState.InTrayArea := FALSE;
			ELSIF LastTurntableState = E_TURNTABLE_MOVEPREGRASP_1
			AND fbMove.bAtPreStoreTurntablePos2	THEN
				(*prepared to put turntable into the tray*)
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEFINALGRASP;
				(*set flag that we are going to enter critical area*)
				stTurnTableState.InTrayArea := TRUE;
			END_IF
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_TURNTABLE_MOVEFINALGRASP:
		stInstances.Input.fMaxVelocity := 3;
		stInstances.Input.fPosition := stParams.fFinalStoreTurntable;

		fbMove.bExecuteMove	:=	TRUE;

		IF fbMove.bError THEN
			(*something went wrong*)
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
			(*movement finished*)
			IF fbMove.bAtFinalStoreTurntablePos THEN
				bGripper := FALSE; (*Open Gripper*)
				stTurnTableState.TurnTableGrasped := FALSE; (*reset flag that we grasped turntable*)
				fbMove.bExecuteMove := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_2;
			END_IF
		END_IF
		fPosition := stInstances.Input.fPosition;

	E_TURNTABLE_MOVETOHOMEPOS:
		stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
		stInstances.Input.fPosition := stParams.fHomePosition;
		fbMove.bExecuteMove	:=	TRUE;
		(*bGripper := TRUE;		(*Close Gripper*)*)

		IF fbMove.bError THEN
			fbMove.bExecuteMove := FALSE;
			bGripper := TRUE; (*Close Gripper*)
			LastTurntableState := eTurntable;
			eTurntable := E_TURNTABLE_IDLE;
			LastMode := eMode;
			eMode := E_WAITFORCOMMAND;
		ELSIF fbMove.bMoveDone THEN
			IF fbMove.bAtHomePos AND fbMove.bStandStill THEN
				fbMove.bExecuteMove := FALSE;
				bGripper := FALSE;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_IDLE;
				stCommands.SetStoreTurntable := FALSE;
			END_IF
		END_IF
		fPosition := stInstances.Input.fPosition;


	E_TURNTABLE_IDLE:
		IF NOT stCommands.SetStoreTurntable
		OR fbMove.bIsMoving THEN
			stCommands.SetStoreTurntable := FALSE;
			LastMode := eMode;
			eMode  := E_WAITFORCOMMAND;
		ELSE
			IF fbMove.bAtLearningPos THEN
			(*Just start if in learning position*)
				bGripper := TRUE; (*Close Gripper*)
				stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_1;
			ELSIF stTurnTableState.TurnTableGrasped THEN
				(*we still have the turntable in gripper*)
				IF stTurnTableState.InTrayArea THEN
					(*we nearly have the turntable stored -> start movement to finalStorePos*)
					LastTurntableState := eTurntable;
					eTurntable := E_TURNTABLE_MOVEFINALGRASP;
				ELSE
					(*we are completely free -> do a normal store movement*)
					stInstances.Input.fMaxVelocity := stParams.MaxVelocity;
					LastTurntableState := eTurntable;
					eTurntable := E_TURNTABLE_MOVEPREGRASP_1;
				END_IF
			ELSIF stTurnTableState.InTrayArea THEN
				(*we are in tray area but have turntable not grasped -> retract from tray*)
				LastTurntableState := eTurntable;
				eTurntable := E_TURNTABLE_MOVEPREGRASP_2;
			ELSE
				stCommands.SetStoreTurntable := FALSE;
				LastMode := eMode;
				eMode  := E_WAITFORCOMMAND;
			END_IF
		END_IF

END_CASEy   , x x ??           CallWaitForCommand ^M7Ua  (*Implementation of Commands*)

IF stCommands.SetAbsolutePosition THEN
	(*Command:	Set Absolute Position*)
	stCommands.SetAbsolutePosition:=FALSE;
	fPosition := stCommands.SetAbsolutePositionValue;

ELSIF stCommands.SetStartAllAxisRef THEN
	(*Command: Start Homing All Axis*)
	LastMode := E_WAITFORCOMMAND;
	eMode := E_HOMING;

ELSIF stCommands.SetReset THEN
	(*Command: Reset*)
	LastMode := E_WAITFORCOMMAND;
	eMode := E_RESET;

ELSIF stCommands.SetEnableAllAxis THEN
	(*Command: Enable All Axis*)
	LastMode := E_WAITFORCOMMAND;
	eMode := E_ENABLEALLAXIS;

ELSIF stCommands.SetCloseGripper THEN
	(*Command: Close Gripper*)
	bGripper := TRUE;
	stCommands.SetCloseGripper := FALSE;

ELSIF stCommands.SetOpenGripper THEN
	(*Command: Open Gripper*)
	bGripper := FALSE;
	stCommands.SetOpenGripper := FALSE;

ELSIF stCommands.SetClearPosBuffer THEN
	(*Command: Clear Position Buffer*)
	REPEAT
		rxPositionBuffer.A_RemoveHead();
	UNTIL
		NOT rxPositionBuffer.bOk
	END_REPEAT
	stCommands.SetClearPosBuffer := FALSE;
	stCommands.PositionsForInterpolationReady := FALSE;

(*
ELSIF stCommands.SetEnableInterpolation THEN
	LastMode := eMode;
	eMode := E_INTERPOLATION;
*)

ELSIF stCommands.SetDisableAllAxis THEN
	(*Command: Disable All Axis*)
	LastMode := E_WAITFORCOMMAND;
	eMode := E_DISABLEALLAXIS;

ELSIF stCommands.SetJogMoveNegAxis1 OR
	stCommands.SetJogMovePosAxis1 OR
	stCommands.SetJogMoveNegAxis2 OR
	stCommands.SetJogMovePosAxis2 OR
	stCommands.SetJogMoveNegAxis3 OR
	stCommands.SetJogMovePosAxis3 OR
	stCommands.SetJogMoveNegAxis4 OR
	stCommands.SetJogMovePosAxis4 OR
	stCommands.SetJogMoveNegAxis5 OR
	stCommands.SetJogMovePosAxis5 OR
	stCommands.SetJogMoveNegAxis6 OR
	stCommands.SetJogMovePosAxis6
THEN
	LastMode := E_WAITFORCOMMAND;
	eMode := E_JOG;

ELSIF stCommands.SetAllPositionsToZero THEN
	LastMode := E_WAITFORCOMMAND;
	eMode := E_SETPOSTOZERO;

END_IF




IF F_CheckIfAllAxisHomed(Joint) AND NOT F_CheckIfAxisHasError(Joint) THEN

	IF eTurntable <> E_TURNTABLE_IDLE
	OR eTray <> E_TRAY_IDLE
	OR eFloor <> E_FLOOR_IDLE
	OR ePreFloor <> E_PRE_FLOOR_IDLE
	OR ePreTable <> E_PRE_TABLE_IDLE
	OR ePutObject <> E_OBJECT_IDLE
	OR eCandlePos <> E_CANDLE_IDLE THEN
		(*don't react on any commands as long as
		state machines are not idle*)
		;
	ELSIF  stCommands.SetStartMove THEN
		(*Command: Start Move:*)
		LastMode := E_WAITFORCOMMAND;
		eMode := E_MOVE;

	ELSIF stCommands.SetMoveToHomePos THEN
		(*Command: Move to home position*)
		LastMode := E_WAITFORCOMMAND;
		eMode := E_MOVETOHOMEPOS;

	ELSIF stCommands.SetMoveToCandlePos THEN
		LastMode := E_WAITFORCOMMAND;
		eMode := E_MOVETOCANDLEPOS;

	ELSIF stCommands.SetMoveToLearningPos THEN
		(*Command: MoveToLearningPos*)
		LastMode := E_WAITFORCOMMAND;
		eMode := E_MOVETOLEARNINGPOS;

	ELSIF stCommands.SetStoreTurntable THEN
		(*Command: MoveToLearningPos*)
		LastMode := E_WAITFORCOMMAND;
		eMode := E_STORETURNTABLE;

	ELSIF stCommands.SetMoveToTray THEN
		(*Command: MoveToTray*)
		LastMode := E_WAITFORCOMMAND;
		eMode := E_MOVETOTRAY;

	ELSIF stCommands.SetPutLearningObjectToTray THEN
		(*Command: MoveToTray*)
		LastMode := E_WAITFORCOMMAND;
		eMode := E_PUTOBJECTTOTRAY;

	ELSIF stCommands.SetTurnTurntableCW THEN
		LastMode := eMode;
		eMode := E_TURNTURNTABLECW;

	ELSIF stCommands.SetTurnTurntableCCW THEN
		LastMode := eMode;
		eMode := E_TURNTURNTABLECCW;

	ELSIF stCommands.SetEnableInterpolation THEN
		LastMode := eMode;
		eMode := E_INTERPOLATION;

	ELSIF stCommands.SetMoveGraspFromFloor THEN
		LastMode := eMode;
		eMode := E_MOVEGRASPFROMFLOOR;

	ELSIF stCommands.SetMovePreGraspFromFloor THEN
		LastMode := eMode;
		eMode := E_MOVEPREGRASPFROMFLOOR;

	ELSIF stCommands.SetMovePreGraspFromTable THEN
		LastMode := eMode;
		eMode := E_MOVEPREGRASPFROMTABLE;

	ELSE
		fbHome.bExecute := FALSE;
		fbMove.bExecuteMove := FALSE;
		fbMoveInterpolation.bExecuteInterpolation := FALSE;
		fbReset.bExecute := FALSE;
		STInstances.Input.SetPositionEnable := FALSE;
	END_IF
ELSE
	fbHome.bExecute := FALSE;
	STInstances.Input.SetPositionEnable := FALSE;
END_IF
Y  ,   ??           Init ^M7U)  	bGripper := FALSE;

IF stCommands.ArmState.AT_HOME_POS THEN
	fPosition := stParams.fHomePosition;
ELSIF stCommands.ArmState.AT_LEARNING_POS THEN
	fPosition := stParams.fLearningPosition;
ELSIF stCommands.ArmState.AT_TURNTABLE_POS THEN
	fPosition := stParams.fFinalGraspTurntable;
ELSIF stCommands.ArmState.AT_TRAY_POS THEN
	fPosition := stParams.fFinalPutObjectToTray;
ELSIF stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS THEN
	fPosition := stParams.fPreGraspFromFloor;
ELSIF stCommands.ArmState.AT_PREGRASPFROMTABLE_POS THEN
	fPosition := stParams.fPreGraspFromTable;
(*ELSIF stCommands.ArmState.AT_CCW_POS THEN
ELSIF stCommands.ArmState.AT_CW_POS THEN*)
ELSIF stCommands.ArmState.AT_CANDLE_POS THEN
	fPosition := stParams.fCandlePosition;
END_IF

stInstances.Input.fPosition := fPosition;z   , x x ??           ReadPLCStatus ^M7U  Joint[1].ReadStatus;
Joint[2].ReadStatus;
Joint[3].ReadStatus;
Joint[4].ReadStatus;
Joint[5].ReadStatus;
Joint[6].ReadStatus;

(*ACTUAL POSITION*)
stCommands.GetActualPos := stInstances.Output.ActualPosition;

(* Actual Arm State: *)
fbCheckState(Axis:= Joint);
             I  , ? ? ?m           PRG_DataAccess ^M7U	^M7U      
berncut           PROGRAM PRG_DataAccess		(*Unterprogramm zum Lesen und Schreiben der XML Konfig Datei*)
VAR_INPUT
	bLoadCfg, bSaveCfg:	BOOL;
END_VAR
VAR
	FB_XmlSrvReadConfig: 	FB_XmlSrvRead;
	FB_XmlSrvWriteConfig: 	FB_XmlSrvWrite;
(*	FB_EventSetReadXML: 	FB_EventSet;
	FB_EventSetSaveXML: 	FB_EventSet;*)
	FE_LoadBusy: F_TRIG;
	FE_Save	:F_TRIG;
	(*FB_BlinkBak: FB_Blinktakt;*)
	FB_XmlSrvWriteConfigBak: FB_XmlSrvWrite;
	pathname: STRING(100);
END_VAR
VAR_OUTPUT
	bDoneLoadCfg:	BOOL;
	bDoneSaveCfg: BOOL;
END_VARN  (*-----------Lesen und schreiben der XML-Konfiguration-----------------*)

pathname := 'C:\Documents and Settings\Administrator\Desktop\HOBBIT\RobotSpecific\parameters.xml';

FB_XmlSrvWriteConfig(
	sNetId:= '',
	ePath:= PATH_GENERIC,
	nMode:= XMLSRV_ADDMISSING,
	pSymAddr:= ADR(stParams) ,
	cbSymSize:= SIZEOF(stParams),
	sFilePath:=pathname ,
	sXPath:= '/Config',
	bExecute:= bSaveCfg,
	tTimeout:=t#5s ,
	bBusy=> ,
	bError=> ,
	nErrId=> );

(*Bakup automatisch sichern*)

(*FB_BlinkBak(bEnable:=TRUE , PT:=t#15m , Q=> );*)

FB_XmlSrvWriteConfigBak(
	sNetId:= '',
	ePath:= PATH_GENERIC,
	nMode:= XMLSRV_ADDMISSING,
	pSymAddr:= ADR(stParams) ,
	cbSymSize:= SIZEOF(stParams),
	sFilePath:='C:\PLC\Config.bak' ,
	sXPath:= '/Config',
	bExecute:= (*FB_BlinkBak.Q*),
	tTimeout:=t#5s ,
	bBusy=> ,
	bError=> ,
	nErrId=> );


FB_XmlSrvReadConfig(
	sNetId:= ,
	ePath:=PATH_GENERIC ,
	nMode:= XMLSRV_SKIPMISSING  ,
	pSymAddr:=ADR(stParams) ,
	cbSymSize:=SIZEOF(stParams) ,
	sFilePath:=pathname ,
	sXPath:= '/Config' ,
	bExecute:=bLoadCfg,
	tTimeout:=t#5s ,
	bBusy=> ,
	bError=> ,
	nErrId=> );

FE_Save(clk:=FB_XmlSrvWriteConfig.bBusy);
IF FE_Save.Q AND NOT FB_XmlSrvWriteConfig.bError THEN
	bDoneSaveCfg := TRUE;
ELSIF NOT bSaveCfg THEN
	bDoneSaveCfg := FALSE;
END_IF

FE_LoadBusy(clk:=FB_XmlSrvReadConfig.bBusy);
IF FE_LoadBusy.Q THEN
	IF NOT FB_XmlSrvReadConfig.bError THEN
		bDoneLoadCfg:=TRUE;
	END_IF
ELSE
	bDoneLoadCfg:=FALSE;
END_IF
(*-------------------------------------St?rungen------------------------------*)
(*FB_EventSetReadXML(
	bSetEvent:= FB_XmlSrvReadConfig.bError,
	tErrTrueTime:= ,
	bQuitEvent:= ,
	bQuitReq:=FALSE ,
	iEventClass:=8 ,
	EventId:= 8,
	SourceId:= 1,
	sNetID:= ,
	sVarFormat:= ,
	sValue:= DWORD_TO_STRING(FB_XmlSrvReadConfig.nErrId),
	bRmEvtSet=> );

FB_EventSetSaveXML(
	bSetEvent:= FB_XmlSrvWriteConfig.bError,
	tErrTrueTime:= ,
	bQuitEvent:= ,
	bQuitReq:=FALSE ,
	iEventClass:=8 ,
	EventId:= 9,
	SourceId:= 1,
	sNetID:= ,
	sVarFormat:= ,
	sValue:= DWORD_TO_STRING(FB_XmlSrvWriteConfig.nErrId),
	bRmEvtSet=> );*)               e   , x x ?        
   SCODE_CODE ^M7U	^M7U       ??          ?   FUNCTION SCODE_CODE : DWORD
(* Helper function: returns the lower word of error code *)
VAR_INPUT
	sc		: DWORD;
END_VAR


   SCODE_CODE := 16#FFFF AND sc;               {   , ? ? ??        O   TCPIPClient @      sRemoteHost ?     sRemotePort ?     eConnectionState ?   ?^M7U	^M7U      s e fat         ?  PROGRAM TCPIPClient
(* Programm to send information to the XPC
   when Status or Variables change *)
VAR
	fbApplication	: FB_ClientApplication := ( sRemoteHost:= REMOTE_IP_ADDRESS, nRemotePort:= CLIENTPORT, bDbg :=TRUE(* TRUE = enable debug output, FALSE = disable *)  );
	bEnable			: BOOL := TRUE;(* TRUE => enable client data exchange, FALSE => disable *)
	fbCloseAll		: FB_SocketCloseAll;
	bCloseAll		: BOOL := TRUE;
END_VAR  IF bCloseAll THEN (*On PLC reset or program download close all old (opened) connections *)
	bCloseAll := FALSE;
	fbCloseAll( bExecute:= TRUE );
ELSE
	fbCloseAll( bExecute:= FALSE );
END_IF

IF NOT fbCloseAll.bBusy THEN
	fbApplication( bEnable := bEnable );
END_IF               |   , ? ? ??           TCPIPServer H^M7U	^M7U      

	onct          PROGRAM TCPIPServer
(* Programm to send information to the XPC
   when requested *)
VAR
	sLocalHost	: STRING(15) 	:= LOCAL_IP_ADDRESS;(* Server address *)
	nLocalPort	: UDINT 		:= SERVERPORT;(* Server port number *)
	bEnable		: BOOL 		:= TRUE;(* TRUE => enable server data exchange, FALSE => disable *)
	hServer		: T_HSERVER;(* Server connection handle *)
	fbApplication	: FB_ServerApplication := ( bDbg:=FALSE );(* Application (connection) instances *)

	fbCloseAll	: FB_SocketCloseAll;
	bCloseAll	: BOOL := TRUE;

END_VAR?  IF bCloseAll THEN(*On PLC reset or program download close all old (opened) connections *)
	bCloseAll := FALSE;
	rxPositionBuffer.A_Reset();

	(* Initialize server handle *)
	F_CreateServerHnd( 	sSrvNetID 	:= '',
						sLocalHost 	:= sLocalHost,
						nLocalPort	:= nLocalPort,
						nMode 		:= CONNECT_MODE_ENABLEDBG,(* Enable debug messages *)
						bEnable 		:= TRUE,(* TRUE = leave listener socket open, FALSE = close listener socket after last connection ist closed too *)
						hServer 		:= hServer );


	fbCloseAll( bExecute:= TRUE );
ELSE
	fbCloseAll( bExecute:= FALSE );
END_IF


IF NOT fbCloseAll.bBusy THEN
	fbApplication( hServer:= hServer, bEnable:= bEnable);
END_IF
                )  , ? ? ?           ARMSTATE ^M7U
    @    ^M7UQ   d                                                                                                          
    @          ? = } (   ???     ???                                             @                           ???        @
                       @                                                                                                           
    @          ? = } (     ???     ???                                            Actual Arm State @                          ???       MS Sans Serif @                       @                                                                                                           
    @         F mE,  ???     ???                                             @                      4    ???        @
                       @                                                                             	   16#00FF00                          
    @        ? n ? ?   ?       ???                                 !   STCommands.ArmState.ARM_HAS_ERROR        @                      5    ???        @
                       @                                                                                                           
    @         n ? ? ? ?     ???     ???                                            Arm Has Error @                      6    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        ? ? ? ?   ?       ???                                    stcommands.ArmState.ARM_HOMED        @                      7    ???        @
                       @                                                                                                           
    @         ? ? ? ? ?     ???     ???                                            Arm Homed @                      8    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        ? ? ?   ?       ???                                    stcommands.ArmState.ARM_STOPPED        @                      9    ???        @
                       @                                                                                                           
    @         ? ? ? ?     ???     ???                                            Arm Has Stopped @                      :    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        ? "K6  ?       ???                                 #   stcommands.ArmState.ARM_IN_POS_AREA        @                      ;    ???        @
                       @                                                                                                           
    @         "? K? 6    ???     ???                                            Arm In Position Area @                      <    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        ? ^?r  ?       ???                                 %   stcommands.ArmState.ARM_IN_TARGET_POS        @                      =    ???        @
                       @                                                                                                           
    @         ^? ?? r    ???     ???                                            Arm In Target Position @                      >    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        ? ???  ?       ???                                 #   stcommands.ArmState.ARM_IS_DISABLED        @                      ?    ???        @
                       @                                                                                                           
    @         ?? ?? ?    ???     ???                                            Arm Disabled @                      @    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        &n O? :?   ?       ???                                 !   stcommands.ArmState.ARM_IS_MOVING        @                      A    ???        @
                       @                                                                                                           
    @        Jn ? ??     ???     ???                                            Arm Is Moving @                      B    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        &? O? :?   ?       ???                                 %   stcommands.ArmState.ARM_SOFTLIMIT_MAX        @                      C    ???        @
                       @                                                                                                           
    @        J? ? ??     ???     ???                                         $   Positive Software 
Limit Switch @                      D    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        &? O:?   ?       ???                                 %   stcommands.ArmState.ARM_SOFTLIMIT_MIN        @                      E    ???        @
                       @                                                                             	   16#00FF00                          
    @        &"OK:6  ?       ???                                    stcommands.ArmState.AT_HOME_POS        @                      G    ???        @
                       @                                                                                                           
    @        J"K?6    ???     ???                                            Arm At Home Position @                      H    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        &^O?:r  ?       ???                                 #   stcommands.ArmState.AT_LEARNING_POS        @                      I    ???        @
                       @                                                                                                           
    @        J^??r    ???     ???                                            Arm At Learning Position @                      J    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        &?O?:?  ?       ???                                 %   stcommands.ArmState.GRIPPER_IS_CLOSED        @                      K    ???        @
                       @                                                                                                           
    @        J????    ???     ???                                            Gripper Is Closed @                      L    ???       MS Sans Serif @                       @                                                                                                           
    @        J? ??     ???     ???                                         $   Negative Software 
Limit Switch @                      M    ???       MS Sans Serif @                       @                                                                             	   16#00FF00                          
    @        ? ???  ?       ???                                 !   stcommands.ArmState.AT_CANDLE_POS        @                      O    ???        @
                       @                                                                                                           
    @         ?? ?? ?    ???     ???                                            Arm in Candle Position @                      P    ???       MS Sans Serif @                       @             ?   ??   ?   ??   ? ? ? ???     ?   ??   ?   ??   ? ? ? ???                  (  , B B ??           COMMANDS ^M7U
    @    ^M7Uk   d   I                                                                         	   16#00FF00                          
    @        
 6? }_ Y  ?       ???                                 %   Main.fbmove.bAtFinalGraspFromFloorPos        @                      [    ???        @
                       @                                                                                                           
    @        
 &? m_ I  ???     ???                                             @                      W    ???        @
                       @                                                                             	   16#00FF00                          
    @        ? Z _? 	}   ?       ???                                    Main.fbmove.bAtLearningPos        @                      S    ???        @
                       @                                                                             	   16#00FF00                          
    @        ? ? _? 	?   ?       ???                                    stcommands.SetMoveToTray        @                      R    ???        @
                       @                                                                             	   16#00FF00                          
    @        
 ? ? ? _ ?   ?       ???                                    stcommands.SetStoreTurntable        @                      Q    ???        @
                       @                                                                             	   16#00FF00                          
    @        ? ? _-		  ?       ???                                    Main.fbmove.bAtTurntableCCWPos        @                      P    ???        @
                       @                                                                                                           
    @        ? &_m	I  ???     ???                                             @                      K    ???        @
                       @                                                                             	   16#00FF00                          
    @        ??c??  ?       ???                                 #   stCommands.ArmState.ARM_IS_DISABLED        @                      I    ???        @
                       @                                                                             	   16#00FF00                          
    @        v?I??  ?       ???                                    stCommands.SetReset        @                      +    ???        @
                       @                                                                             	   16#00FF00                          
    @        ?|c???  ?       ???                                 '   NOT stCommands.ArmState.ARM_IS_DISABLED        @                          ???        @
                       @                                                                             	   16#00FF00                          
    @        v|I???  ?       ???                                 %   stCommands.ArmState.ARM_IN_TARGET_POS        @                          ???        @
                       @                                                                             	   16#00FF00                          
    @        ?mc?  ?       ???                                    Main.fbhome.bHomingBusy        @                          ???        @
                       @                                                                             	   16#00FF00                          
    @        
 
 ? Q _ -   ?       ???                                 )   NOT stCommands.ArmState.GRIPPER_IS_CLOSED        @                          ???        @
                       @                                                                                                         
    @        ??+???    @                     Start Move @???     ???             @        ???        @
    stcommands.SetStartMove                 @       ?                                                                                                     
    @          ? G _ -     @                 "   Open Gripper @???     ???             @        ???        @
    stCommands.SetOpenGripper                 @       ?                                                                                                     
    @        ?&cY?    @                 %   Start Reference @???     ???             @        ???        @
    stcommands.SetStartAllAxisRef                 @       ?                                                                                                     
    @        ??E???    @                 %   Enable All Axis @???     ???             @    
    ???        @
    stcommands.SetEnableAllAxis                 @       ?                                                                                                     
    @        ??E	??    @                 &   Disable All Axis @???     ???             @        ???        @
    stcommands.SetDisableAllAxis                 @       ?                                                                                                       
    @        v
 Ii??    ?@     ???                                             @                          ???        @
                       @                                                                                                     -91   91
    @        ?F 5e ?U   ???     ???                                        Main.fPosition[1]   %.2f @                          ???        @
                      @                                                                                                         
    @        ??+	??    @                    Reset @???     ???             @        ???        @
    stcommands.SetReset                 @       ?                                                                                                       
    @        ?F ?e ?U      ?@     ???                                         
   Axis 1 @                          ???        @
                       @                                                                                                           
    @        ?n ?? ?}     ???     ???                                         
   Axis 2 @                          ???        @
                       @                                                                                                           
    @        ?? ?? ??     ???     ???                                         
   Axis 3 @                          ???        @
                       @                                                                                                           
    @        ?? ?? ??     ???     ???                                         
   Axis 4 @                          ???        @
                       @                                                                                                           
    @        ?? ???     ???     ???                                         
   Axis 5 @                          ???        @
                       @                                                                                                           
    @        ??-?    ???     ???                                         
   Axis 6 @                          ???        @
                       @                                                                                                           
    @        v
 I3 ?     ???     ???                                            New Position @                           ???       MS Sans Serif @                       @                                                                                                     -90   90
    @        ?n 5? ?}   ???     ???                                        Main.fPosition[2]   %.2f @                      !    ???        @
                      @                                                                                                     -35   120
    @        ?? 5? ??   ???     ???                                        Main.fPosition[3]   %.2f @                      "    ???        @
                      @                                                                                                     -170   170
    @        ?? 5? ??   ???     ???                                        Main.fPosition[4]   %.2f @                      #    ???        @
                      @                                                                                                     -50   130
    @        ?? 5??   ???     ???                                        Main.fPosition[5]   %.2f @                      $    ???        @
                      @                                                                                                     -170   170
    @        ?5-?  ???     ???                                        Main.fPosition[6]   %.2f @                      %    ???        @
                      @                                                                             	   16#00FF00                          
    @        
 Z ? ? _ }   ?       ???                                    Main.fbmove.bAtHomePos        @                      ,    ???        @
                       @                                                                                                         
    @         d ? ? _ }     @                 -   Move To 
Home Position @???     ???             @    -    ???        @
    stcommands.SetMoveToHomePos                 @       ?                                                                                                     
    @        ? d U? 	}     @                 0   Move To
Learning Position @???     ???             @    .    ???        @
    stcommands.SetMoveToLearningPos                 @       ?                                                                                                     
    @         ? ? ? _ ?     @                 %   Store Turntable @???     ???             @    1    ???        @
    stcommands.SetStoreTurntable                 @       ?                                                                         	   16#00FF00                          
    @        
 ? ? -_ 	  ?       ???                                    Main.fbmove.bAtTurntableCWPos        @                      2    ???        @
                       @                                                                                                         
    @         ? ? #_ 	    @                 '   Turn Turntable CW @???     ???             @    3    ???        @
    stcommands.SetTurnTurntableCW                 @       ?                                                                                                     
    @        ? ? U#		    @                 (   Turn Turntable CCW @???     ???             @    4    ???        @
    stcommands.SetTurnTurntableCCW                 @       ?                                                                         	   16#00FF00                          
    @        lc??  ?       ???                                    stcommands.ArmState.ARM_STOPPED        @                      5    ???        @
                       @                                                                                                         
    @        v&Y??    @                    Stop Arm @???     ???             @    6    ???        @
    stcommands.SetStopArm                 @       ?                                                                                                       
    @        ? &_m	I  ???     ???                                             @                      7    ???        @
                       @                                                                                                         
    @        ? 0Uc	I    @                 *   Enable Interpolation @???     ???             @    8    ???        @
 !   stcommands.SetEnableInterpolation                 @       ?                                                                         	   16#00FF00                          
    @        ? 
 _Q 	-   ?       ???                                 %   stCommands.ArmState.GRIPPER_IS_CLOSED        @                      9    ???        @
                       @                                                                                                         
    @        ?  UG 	-     @                 #   Close Gripper @???     ???             @    :    ???        @
    stCommands.SetCloseGripper                 @       ?                                                                                                       
    @        ?
 c7??    ??     ???                                             @                      ;    ???        @
                       @                                                                                                     -90   90
    @        ?F Oe U   ???     ???                                        Main.Joint[1].NcToPlc.ActPos   %.2f @                      <    ???        @
                      @                                                                                                           
    @        ?F ?e ?U     ???     ???                                         
   Axis 1 @                      =    ???        @
                       @                                                                                                           
    @        ?n ?? ?}     ???     ???                                         
   Axis 2 @                      >    ???        @
                       @                                                                                                           
    @        ?? ?? ??     ???     ???                                         
   Axis 3 @                      ?    ???        @
                       @                                                                                                           
    @        ?? ?? ??     ???     ???                                         
   Axis 4 @                      @    ???        @
                       @                                                                                                           
    @        ?? ???     ???     ???                                         
   Axis 5 @                      A    ???        @
                       @                                                                                                           
    @        ??-?    ???     ???                                         
   Axis 6 @                      B    ???        @
                       @                                                                                                           
    @        ?
 c3 ?     ???     ???                                            Actual Position @                      C    ???       MS Sans Serif @                       @                                                                                                     -90   90
    @        ?n O? }   ???     ???                                        Main.Joint[2].NcToPlc.ActPos   %.2f @                      D    ???        @
                      @                                                                                                     -90   90
    @        ?? O? ?   ???     ???                                        Main.Joint[3].NcToPlc.ActPos   %.2f @                      E    ???        @
                      @                                                                                                     -90   90
    @        ?? O? ?   ???     ???                                        Main.Joint[4].NcToPlc.ActPos   %.2f @                      F    ???        @
                      @                                                                                                     -90   90
    @        ?? O?   ???     ???                                        Main.Joint[5].NcToPlc.ActPos   %.2f @                      G    ???        @
                      @                                                                                                     -90   90
    @        ?O-  ???     ???                                        Main.Joint[6].NcToPlc.ActPos   %.2f @                      H    ???        @
                      @                                                                                                         
    @        ? ? U? 	?     @                 "   Move To Tray @???     ???             @    O    ???        @
    stcommands.SetMoveToTray                 @       ?                                                                                                     
    @         @? s_ Y    @                 -   Move 
Grasp From Floor @???     ???             @    T    ???        @
     stcommands.SetMoveGraspFromFloor                 @       ?                                                                                                     
    @         0? c_ I    @                 0   Set All Positions to
Zero @???     ???             @    U    ???        @
         stCommands.SetAllPositionsToZero             @       ?                                                                                                       
    @        ?6?U?E    ???     ???                                            Velocity @                      \    ???        @
                       @                                                                                                     0   20
    @        ?65U?E  ???     ???                                     
   .fVelocity   %.2f @                      ]    ???        @
                      @                                                                             	   16#00FF00                          
    @        ? 6_}	Y  ?       ???                                 #   Main.fbmove.bAtPreGraspFromFloorPos        @                      ^    ???        @
                       @                                                                                                         
    @        ? @Us	Y    @                 1   Move Pre 
Grasp From Floor @???     ???             @    _    ???        @
 #   stcommands.SetMovePreGraspFromFloor                 @       ?                                                                         	   16#00FF00                          
    @        
 ?? ?_ ?  ?       ???                                 &   Main.fbmove.bAtFinalPutObjectToTrayPos        @                      b    ???        @
                       @                                                                                                         
    @         ?? ?_ ?    @                 2   Put Learning Object
To Tray @???     ???             @    c    ???        @
 %   stcommands.SetPutLearningObjectToTray                 @       ?                                                                         	   16#00FF00                          
    @        ? ?_?	?  ?       ???                                 #   Main.fbmove.bAtPreGraspFromTablePos        @                      f    ???        @
                       @                                                                                                         
    @        ? ?U?	?    @                 1   Move Pre 
Grasp From Table @???     ???             @    g    ???        @
 #   stcommands.SetMovePreGraspFromTable                 @       ?                                                                         	   16#00FF00                          
    @        
 ?? _ ?  ?       ???                                    Main.fbmove.bAtCandlePos        @                      h    ???        @
                       @                                                                                                         
    @         ?? _ ?    @                 -   Move to Candle position @???     ???             @    i    ???        @
    stcommands.SetMoveToCandlePos                 @       ?         ?   ??   ?   ??   ? ? ? ???     ?   ??   ?   ??   ? ? ? ???                  ?   , < < H[           PROGRAMSTATES ^M7U
    @    ^M7U   d                                                                                                          
    @        x ? EU^'  ???     ???                                             @                          ???        @
                       @                                                                                                           
    @        x ? E? ^?   ???     ???                                             @                          ???        @
                       @                                                                                                           
    @        x 
 E? ^U   ???     ???                                             @                          ???        @
                       @                                                                                                           
    @        ,? ? ??   ???     ???                                         TCPIPClient.fbApplication.eState   %s @                          ???        @
                      @                                                                                                           
    @        , [ ?7   ???     ???                                     
   Main.eMode   %s @                          ???        @
                      @                                                                                                           
    @        ,K?'  ???     ???                                         TCPIPServer.fbApplication.eState   %s @                          ???        @
                      @                                                                                                           
    @        x  #[ ? 7     ???     ???                                            Main STATE @                          ???       MS Sans Serif @                       @                                                                                                           
    @        x ? -? ? ?     ???     ???                                            Client STATE @                          ???       MS Sans Serif @                       @                                                                                                           
    @        x -K? '    ???     ???                                            Server STATE @                          ???       MS Sans Serif @                       @                                                                                                           
    @        ? P -? ? s     ???     ???                                            Last STATE @                          ???       MS Sans Serif @                       @                                                                                                           
    @        ,d ? ?x   ???     ???                                        Main.LastMode   %s @                          ???        @
                      @                                                                                                           
    @        @?????  ???     ???                                        .REMOTE_IP_ADDRESS   %s @                          ???        @
                      @                                                                                                           
    @        @r????  ???     ???                                        .LOCAL_IP_ADDRESS   %s @                          ???        @
                       @                                                                                                           
    @        ? |#?? ?    ???     ???                                            IP Address CX: @                          ???       MS Sans Serif @                       @                                                                                                           
    @        ? ?#?? ?    ???     ???                                            IP Address XPC: @                          ???       MS Sans Serif @                       @             ?   ??   ?   ??   ? ? ? ???     ?   ??   ?   ??   ? ? ? ???                  ????, ? ? ??         &   TcUtilities.lib 10.1.13 21:12:12 @??P!   TcBase.lib 14.5.09 13:14:08 @ ?J#   TcSystem.lib 16.1.14 20:38:48 @8'?R%   TcBaseMath.lib 27.7.04 13:07:56 @?7A!   TcMath.lib 23.9.04 16:15:30 @??RA    TcMC2.lib 14.7.14 12:34:46 @F??S"   STANDARD.LIB 5.6.98 13:03:02 @f?w5   TcpIp.lib*2.2.15 14:33:03 @n?T(   TcSocketHelper.lib*2.2.15 14:33:03 @n?T3   Hella.Automation.Tools-00.lib*2.2.15 14:33:03 @n?T!   TcSUPS.lib 22.3.10 14:59:22 @?i?K'   TcXmlDataSrv.lib*29.9.06 16:13:00 @?)E/  	  ARG_TO_CSVFIELD @?      ADSDATATYPEID       E_AmsLoggerMode    	   E_ArgType       E_DbgContext       E_DbgDirection       E_EnumCmdType       E_FileRBufferCmd       E_HashPrefixTypes       E_MIB_IF_Type       E_NumGroupTypes       E_PersistentMode       E_PrefixFlagParam       E_RegValueType       E_RouteTransportType    
   E_SBCSType       E_ScopeServerState       E_TimeZoneID       E_TypeFieldParam       E_UTILITIES_ERRORCODES       OTSTRUCT       PROFILERSTRUCT       REMOTEPC       REMOTEPCINFOSTRUCT       ST_AmsFindFileSystemEntry       ST_AmsGetTimeZoneInformation       ST_AmsLoggerReq       ST_AmsRouteEntry       ST_AmsRouteEntryHead       ST_AmsRouterInfoEntry       ST_AmsRouteSystemEntry       ST_AmsStartProcessReq       ST_AmsSymbolInfoEntry       ST_DeviceIdentification       ST_DeviceIdentificationEx       ST_FileAttributes       ST_FileRBufferHead       ST_FindFileEntry       ST_FormatParameters       ST_HKeySrvRead       ST_HKeySrvWrite       ST_IP_ADAPTER_INFO       ST_IP_ADDR_STRING       ST_IPAdapterHwAddr       ST_IPAdapterInfo       ST_SBCSTable    #   ST_ScopeServerRecordModeDescription       ST_TcRouterStatusInfo       ST_TimeZoneInformation       SYMINFO_BUFFER       SYMINFOSTRUCT       T_Arg    
   T_FILETIME       T_FIX16    
   T_FloatRec       T_HashTableEntry       T_HHASHTABLE       T_HLINKEDLIST       T_HUGE_INTEGER       T_LARGE_INTEGER       T_LinkedListEntry       T_UHUGE_INTEGER       T_ULARGE_INTEGER    
   TIMESTRUCT                  BCD_TO_DEC @           BE128_TO_HOST @          BE16_TO_HOST @          BE32_TO_HOST @          BE64_TO_HOST @          BYTEARR_TO_MAXSTRING @          CSVFIELD_TO_ARG @          CSVFIELD_TO_STRING @          DATA_TO_HEXSTR @          DCF77_TIME @          DCF77_TIME_EX @          DEC_TO_BCD @           DEG_TO_RAD @           DINT_TO_DECSTR @          DT_TO_FILETIME @          DT_TO_SYSTEMTIME @           DWORD_TO_BINSTR @          DWORD_TO_DECSTR @          DWORD_TO_HEXSTR @          DWORD_TO_LREALEX @          DWORD_TO_OCTSTR @          F_ARGCMP @          F_ARGCPY @          F_ARGIsZero @          F_BIGTYPE @          F_BOOL @          F_BYTE @           F_BYTE_TO_CRC16_CCITT @          F_CheckSum16 @           F_CRC16_CCITT @           F_CreateHashTableHnd @          F_CreateLinkedListHnd @          F_DATA_TO_CRC16_CCITT @          F_DINT @           F_DWORD @           F_FormatArgToStr @          F_GetDayOfMonthEx @          F_GetDayOfWeek @          F_GetDOYOfYearMonthDay @          F_GetFloatRec @          F_GetMaxMonthDays @          F_GetMonthOfDOY @          F_GetVersionTcUtilities @           F_GetWeekOfTheYear @          F_HUGE @          F_INT @           F_LARGE @          F_LREAL @           F_LTrim @          F_REAL @           F_RTrim @          F_SINT @           F_STRING @           F_SwapReal @           F_SwapRealEx @          F_ToLCase @          F_ToUCase @          F_TranslateFileTimeBias @          F_UDINT @           F_UHUGE @          F_UINT @           F_ULARGE @          F_USINT @           F_WORD @           F_YearIsLeapYear @          FB_AddRouteEntry @          FB_AmsLogger @          FB_BasicPID @           FB_BufferedTextFileWriter @       '   FB_BufferedTextFileWriter.A_Reset @          FB_ConnectScopeServer @          FB_CSVMemBufferReader @          FB_CSVMemBufferWriter @          FB_DbgOutputCtrl @          FB_DbgOutputCtrl.A_Log @          FB_DbgOutputCtrl.A_LogHex @          FB_DbgOutputCtrl.A_Reset @          FB_DisconnectScopeServer @          FB_EnumFindFileEntry @          FB_EnumFindFileList @          FB_EnumRouteEntry @          FB_EnumStringNumbers @          FB_FileRingBuffer @       !   FB_FileRingBuffer.A_AddTail @          FB_FileRingBuffer.A_Close @           FB_FileRingBuffer.A_Create @       !   FB_FileRingBuffer.A_GetHead @          FB_FileRingBuffer.A_Open @       $   FB_FileRingBuffer.A_RemoveHead @          FB_FileRingBuffer.A_Reset @       &   FB_FileTimeToTzSpecificLocalTime @       .   FB_FileTimeToTzSpecificLocalTime.A_Reset @          FB_FormatString @           FB_GetAdaptersInfo @           FB_GetDeviceIdentification @       "   FB_GetDeviceIdentificationEx @          FB_GetHostAddrByName @          FB_GetHostName @          FB_GetLocalAmsNetId @          FB_GetRouterStatusInfo @          FB_GetTimeZoneInformation @          FB_HashTableCtrl @          FB_HashTableCtrl.A_Add @       !   FB_HashTableCtrl.A_GetFirst @       )   FB_HashTableCtrl.A_GetIndexAtPosPtr @           FB_HashTableCtrl.A_GetNext @          FB_HashTableCtrl.A_Lookup @          FB_HashTableCtrl.A_Remove @       "   FB_HashTableCtrl.A_RemoveAll @       $   FB_HashTableCtrl.A_RemoveFirst @          FB_HashTableCtrl.A_Reset @          FB_LinkedListCtrl @       &   FB_LinkedListCtrl.A_AddHeadValue @       &   FB_LinkedListCtrl.A_AddTailValue @       "   FB_LinkedListCtrl.A_FindNext @       "   FB_LinkedListCtrl.A_FindPrev @       !   FB_LinkedListCtrl.A_GetHead @       *   FB_LinkedListCtrl.A_GetIndexAtPosPtr @       !   FB_LinkedListCtrl.A_GetNext @       !   FB_LinkedListCtrl.A_GetPrev @       !   FB_LinkedListCtrl.A_GetTail @       )   FB_LinkedListCtrl.A_RemoveHeadValue @       )   FB_LinkedListCtrl.A_RemoveTailValue @       -   FB_LinkedListCtrl.A_RemoveValueAtPosPtr @          FB_LinkedListCtrl.A_Reset @       *   FB_LinkedListCtrl.A_SetValueAtPosPtr @          FB_LocalSystemTime @          FB_MemBufferMerge @          FB_MemBufferSplit @          FB_MemRingBuffer @           FB_MemRingBuffer.A_AddTail @           FB_MemRingBuffer.A_GetHead @       #   FB_MemRingBuffer.A_RemoveHead @          FB_MemRingBuffer.A_Reset @          FB_MemRingBufferEx @       "   FB_MemRingBufferEx.A_AddTail @       #   FB_MemRingBufferEx.A_FreeHead @       &   FB_MemRingBufferEx.A_GetFreeSize @       "   FB_MemRingBufferEx.A_GetHead @           FB_MemRingBufferEx.A_Reset @          FB_MemStackBuffer @          FB_MemStackBuffer.A_Pop @          FB_MemStackBuffer.A_Push @          FB_MemStackBuffer.A_Reset @          FB_MemStackBuffer.A_Top @          FB_RegQueryValue @           FB_RegSetValue @           FB_RemoveRouteEntry @           FB_ResetScopeServerControl @          FB_SaveScopeServerData @          FB_ScopeServerControl @          FB_SetTimeZoneInformation @          FB_StartScopeServer @          FB_StopScopeServer @          FB_StringRingBuffer @       #   FB_StringRingBuffer.A_AddTail @       #   FB_StringRingBuffer.A_GetHead @       &   FB_StringRingBuffer.A_RemoveHead @       !   FB_StringRingBuffer.A_Reset @       (   FB_SystemTimeToTzSpecificLocalTime @       0   FB_SystemTimeToTzSpecificLocalTime.A_Reset @          FB_TextFileRingBuffer @       %   FB_TextFileRingBuffer.A_AddTail @       #   FB_TextFileRingBuffer.A_Close @       "   FB_TextFileRingBuffer.A_Open @       #   FB_TextFileRingBuffer.A_Reset @       (   FB_TranslateLocalTimeToUtcByZoneID @       0   FB_TranslateLocalTimeToUtcByZoneID.A_Reset @       (   FB_TranslateUtcToLocalTimeByZoneID @       0   FB_TranslateUtcToLocalTimeByZoneID.A_Reset @       &   FB_TzSpecificLocalTimeToFileTime @       .   FB_TzSpecificLocalTimeToFileTime.A_Reset @       (   FB_TzSpecificLocalTimeToSystemTime @       0   FB_TzSpecificLocalTimeToSystemTime.A_Reset @          FB_WritePersistentData @          FILETIME_TO_DT @          FILETIME_TO_SYSTEMTIME @          FIX16_TO_LREAL @          FIX16_TO_WORD @          FIX16Add @          FIX16Align @          FIX16Div @          FIX16Mul @          FIX16Sub @          GetRemotePCInfo @           HEXSTR_TO_DATA @          HOST_TO_BE128 @          HOST_TO_BE16 @          HOST_TO_BE32 @          HOST_TO_BE64 @          INT64_TO_LREAL @          Int64Add64 @          Int64Add64Ex @          Int64Cmp64 @          Int64Div64Ex @          Int64IsZero @          Int64Negate @          Int64Not @          Int64Sub64 @          IsFinite @          LARGE_INTEGER @          LARGE_TO_ULARGE @          LREAL_TO_FIX16 @          LREAL_TO_FMTSTR @          LREAL_TO_INT64 @          LREAL_TO_UINT64 @          MAXSTRING_TO_BYTEARR @          NT_AbortShutdown @           NT_GetTime @           NT_Reboot @           NT_SetLocalTime @          NT_SetTimeToRTCTime @           NT_Shutdown @           NT_StartProcess @           OTSTRUCT_TO_TIME @           PBOOL_TO_BOOL @          PBYTE_TO_BYTE @          PDATE_TO_DATE @          PDINT_TO_DINT @          PDT_TO_DT @          PDWORD_TO_DWORD @          PHUGE_TO_HUGE @          PINT_TO_INT @          PLARGE_TO_LARGE @          PLC_ReadSymInfo @           PLC_ReadSymInfoByName @           PLC_ReadSymInfoByNameEx @           PLC_Reset @           PLC_Start @           PLC_Stop @           PLREAL_TO_LREAL @          PMAXSTRING_TO_MAXSTRING @          PREAL_TO_REAL @          Profiler @           PSINT_TO_SINT @          PSTRING_TO_STRING @          PTIME_TO_TIME @          PTOD_TO_TOD @          PUDINT_TO_UDINT @          PUHUGE_TO_UHUGE @          PUINT64_TO_UINT64 @          PUINT_TO_UINT @          PULARGE_TO_ULARGE @          PUSINT_TO_USINT @          PWORD_TO_WORD @          RAD_TO_DEG @           ROUTETRANSPORT_TO_STRING @       	   RTC @          RTC_EX @          RTC_EX2 @          ScopeASCIIExport @           ScopeExit @          ScopeGetRecordLen @           ScopeGetState @           ScopeLoadFile @           ScopeManualTrigger @           ScopeSaveAs @          ScopeSetOffline @           ScopeSetOnline @           ScopeSetRecordLen @           ScopeViewExport @           STRING_TO_CSVFIELD @          STRING_TO_SYSTEMTIME @          STRING_TO_UINT64 @          SYSTEMTIME_TO_DT @           SYSTEMTIME_TO_FILETIME @          SYSTEMTIME_TO_STRING @          TC_Config @          TC_CpuUsage @           TC_Restart @           TC_Stop @           TC_SysLatency @           TIME_TO_OTSTRUCT @           UInt32x32To64 @          UINT64_TO_LREAL @          UINT64_TO_STRING @          UInt64Add64 @          UInt64Add64Ex @          UInt64And @          UInt64Cmp64 @          UInt64Div16Ex @          UInt64Div64 @          UInt64Div64Ex @          UInt64isZero @          UInt64Limit @          UInt64Max @          UInt64Min @          UInt64Mod64 @          UInt64Mul64 @          UInt64Mul64Ex @          UInt64Not @          UInt64Or @          UInt64Rol @          UInt64Ror @          UInt64Shl @          UInt64Shr @          UInt64Sub64 @          UInt64Xor @          ULARGE_INTEGER @          ULARGE_TO_LARGE @          WORD_TO_FIX16 @          WritePersistentData @           ?	  Global_Variables @V      DEFAULT_CSV_FIELD_DOUBLE_QUOTE"@     MAX_REMOTE_PCS"@     FLOATREC_MIN_PRECISION"@     SYSTEMSERVICE_ADDREMOTE"@     SYSTEMSERVICE_IPHELPERAPI"@     SBCS_TABLES"@     FMTERR_UNACCEPTEDPARAMETER"@     MAX_ROUTE_TRANSPORT"@     MAX_LOCAL_ADAPTERS"@     ARGTYPE_IS_FLOAT_TYPE"@     FORMAT_HASH_PREFIX_STRING"@     FMTERR_PRECISIONVALUE"@     FMTERR_TYPEFIELDVALUE"@     ROUTE_FLAG_NOOVERRIDE"@     GLOBAL_FORMAT_HASH_PREFIX_TYPE"@     DATE_AND_TIME_SECPERWEEK"@     FMTERR_ARGTYPEINVALID"@     FMTERR_NOERROR"@     SYSTEMTIME_DATE_AND_TIME_MAX"@     FLOATREC_MAX_PRECISION"@     MAX_ROUTE_NAME_LEN"@     SYSTEMSERVICE_FFILEFIND"@     SYSTEMTIME_TICKSPERSEC"@     FMTERR_WIDTHPRECISIONVALPOS"@     EMPTY_ROUTE_ENTRY"@     MIN_SBCS_TABLE"@     MIN_ROUTE_TRANSPORT"@     FMTERR_DESTBUFFOVERFLOW"@     HKEY_MAX_BINARY_DATA_SIZE"@     MAX_BASIC_HASHTABLE_CHAINSIZE"@     ROUTE_FLAG_TEMPORARY"@     AMSLOGGER_IGR_GENERAL"@     MAX_ADAPTER_ADDRESS_LENGTH"@     IPHELPERAPI_IPADDRBYHOSTNAME"@     WEST_EUROPE_TZI"@     DBG_OUTPUT_FILE"@  
   ImQn_TABLE"@     FMTERR_FLAGPOSITION"@     MAX_SBCS_TABLE"@     FORMAT_MAX_ARGS"@     FLOATREC_EXP_IS_INF"@     FORMAT_TYPE_SUPPORT_MASK"@     FORMAT_DEFAULT_PRECISION"@     FORMAT_DECASC_CODES"@     DEFAULT_CSV_RECORD_SEP_LF"@     MAX_ADAPTER_NAME_LENGTH"@     ARGTYPE_IS_UNSIGNED_TYPE"@     SYSTEMTIME_TICKSPERMSEC"@     MAX_ROUTE_ADDR_LEN"@     IPHELPERAPI_ADAPTERSINFO"@     FMTERR_PRECISIONDOTPOSITION"@     DBG_OUTPUT_LOG"@     MAX_ADAPTER_DESCRIPTION_LENGTH"@     DATE_AND_TIME_SECPERDAY"@     GLOBAL_DCF77_SEQUENCE_CHECK"@     FLOATREC_EXP_IS_NAN"@     SYSTEMTIME_TICKSPERDAY"@     SYSTEMSERVICE_IPHOSTNAME"@     MAX_AVERAGE_MEASURES"@     SYSTEMSERVICE_ENUMREMOTE"@     DBG_OUTPUT_VISU"@     FLOATREC_MAX_DIGITS"@     DBG_OUTPUT_NONE"@     GLOBAL_CRC_CCITT_TABLE"@     DEFAULT_CSV_FIELD_SEP"@     AMSPORT_AMSLOGGER"@     EMPTY_ARG_VALUE"@     SYSTEMSERVICE_DELREMOTE"@     SYSTEMTIME_MAX_YEARSDAY"@     ARGTYPE_IS_STRING_TYPE"@     SYSTEMTIME_DATE_AND_TIME_MIN"@     DEFAULT_CSV_RECORD_SEP_CR"@     ARGTYPE_IS_SIGNED_TYPE"@     FMTERR_INSUFFICIENTARGS"@     SYSTEMTIME_MAX_MONTHDAYS"@     FMTERR_PERCENTSIGNPOSITION"@     GLOBAL_DCF77_PULSE_SPLIT"@     FORMAT_PREFIX_SUPPORT_MASK"@     FMTERR_WIDTHVALUE"@     FMTERR_ASTERISKPOSITION"@     ARGTYPE_IS_BIT_TYPE"@     SYSTEMTIME_DATEDELTA_OFFSET"@     FORMAT_HEXASC_CODES"@     GLOBAL_SBCS_TABLE"@     ROUTE_FLAG_DYNAMIC"@     AMSLOGGER_IOF_MODE"@   @     z   FW_AdsClearEvents @      FW_NoOfByte       FW_SystemInfoType       FW_SystemTaskInfoType    
   FW_TcEvent                   FW_AdsLogDINT @           FW_AdsLogEvent @           FW_AdsLogLREAL @           FW_AdsLogSTR @           FW_AdsRdWrt @           FW_AdsRdWrtInd @           FW_AdsRdWrtRes @           FW_AdsRead @           FW_AdsReadDeviceInfo @           FW_AdsReadInd @           FW_AdsReadRes @           FW_AdsReadState @           FW_AdsWrite @           FW_AdsWriteControl @           FW_AdsWriteInd @           FW_AdsWriteRes @           FW_DRand @           FW_GetCpuAccount @           FW_GetCpuCounter @           FW_GetCurTaskIndex @           FW_GetSystemTime @           FW_GetVersionTcBase @           FW_LptSignal @           FW_MemCmp @           FW_MemCpy @           FW_MemMove @           FW_MemSet @           FW_PortRead @          FW_PortWrite @           Q   ?  ADSCLEAREVENTS @       E_IOAccessSize    
   E_OpenPath       E_SeekOrigin       E_TcEventClass       E_TcEventClearModes       E_TcEventPriority       E_TcEventStreamType       ExpressionResult       PVOID       SFCActionType       SFCStepType       ST_AdsBaDevApiHead       ST_AdsBaDevApiIoCtlModifier       ST_AdsBaDevApiReq       ST_AdsRdWrtListHead       ST_AdsRdWrtListPara       ST_AdsReadWriteListEntry    
   ST_AmsAddr       ST_StructMemberAlignmentProbe       SYSTEMINFOTYPE       SYSTEMTASKINFOTYPE    
   T_AmsNetId       T_AmsNetIdArr    	   T_AmsPort    
   T_IPv4Addr       T_IPv4AddrArr       T_MaxString       T_U64KAFFINITY       TcEvent       UXINT       XINT       XWORD                   ADSLOGDINT @           ADSLOGEVENT @           ADSLOGLREAL @           ADSLOGSTR @           ADSRDDEVINFO @           ADSRDSTATE @           ADSRDWRT @           ADSRDWRTEX @           ADSRDWRTIND @           ADSRDWRTRES @           ADSREAD @           ADSREADEX @           ADSREADIND @           ADSREADRES @           ADSWRITE @           ADSWRITEIND @           ADSWRITERES @           ADSWRTCTL @           AnalyzeExpression @          AnalyzeExpressionCombined @          AnalyzeExpressionTable @          AppendErrorString @          BAVERSION_TO_DWORD @          CLEARBIT32 @           CSETBIT32 @           DRAND @           F_CompareFwVersion @          F_CreateAmsNetId @           F_CreateIPv4Addr @           F_GetStructMemberAlignment @          F_GetVersionTcSystem @           F_IOPortRead @          F_IOPortWrite @          F_ScanAmsNetIds @          F_ScanIPv4AddrIds @          F_SplitPathName @          F_ToASC @          F_ToCHR @          FB_AdsReadWriteList @          FB_BaDeviceIoControl @          FB_BaGenGetVersion @          FB_CreateDir @          FB_EOF @           FB_FileClose @           FB_FileDelete @           FB_FileGets @           FB_FileOpen @           FB_FilePuts @           FB_FileRead @           FB_FileRename @           FB_FileSeek @           FB_FileTell @           FB_FileWrite @           FB_PcWatchdog @          FB_RemoveDir @          FB_SimpleAdsLogEvent @          FILECLOSE @           FILEOPEN @           FILEREAD @           FILESEEK @           FILEWRITE @           FW_CallGenericFb @          FW_CallGenericFun @          GETBIT32 @           GETCPUACCOUNT @           GETCPUCOUNTER @           GETCURTASKINDEX @           GETSYSTEMTIME @           GETTASKTIME @          LPTSIGNAL @           MEMCMP @           MEMCPY @           MEMMOVE @           MEMSET @           ROL32 @           ROR32 @           SETBIT32 @           SFCActionControl @           SHL32 @           SHR32 @           p  Global_Variables @?      SYSTEMSERVICE_FRENAME"@     ADSIGRP_IOIMAGE_RWOB"@     SYSTEMSERVICE_CHANGENETID"@     FOPEN_MODEREAD"@     SYSTEMSERVICE_CLOSEHANDLE"@     SYSTEMSERVICE_FTELL"@     BOOTDATAFLAGS_RETAIN_REQUESTED"@     ADSSTATE_RESET"@     SYSTEMSERVICE_FGETS"@     SYSTEMSERVICE_FEOF"@     AMSPORT_R0_CAMTOOL"@     PI"@     SYSTEMSERVICE_FSEEK"@     ADSSTATE_RECONFIG"@     FILE_OPENCREATE"@     ADSIGRP_SYMNAME"@     ADSSTATE_START"@     ADSSTATE_LOADCFG"@     ADSSTATE_IDLE"@     BOOTDATAFLAGS_RETAIN_LOADED"@     FILE_SEEKEND"@     ADSIGRP_SYM_HNDBYNAME"@     TCEVENTFLAG_FMTSELF"@     ADSIGRP_SYMNOTE"@     SYSTEMSERVICE_FWRITE"@  "   SYSTEMSERVICE_REG_HKEYLOCALMACHINE"@     ADSLOG_MSGTYPE_STRING"@     TCEVENT_FMTPRGSIZE"@     ADSIGRP_SYM_VERSION"@     ADSIGRP_IOIMAGE_RWIOB"@     ADSIGRP_IOIMAGE_ROSIZE"@     ADSIGRP_SYM_INFOBYNAMEEX"@     ADSIGRP_IOIMAGE_RISIZE"@     AMSPORT_R0_NCSAF"@     TCEVENTSTATE_INVALID"@     TCEVENTSTATE_CONFIRMED"@     FILE_OPENREAD"@     AMSPORT_LOGGER"@     BOOTDATAFLAGS_PERSISTENT_LOADED"@     TCEVENTSTATE_RESETCON"@     SYSTEMSERVICE_FDELETE"@  	   FILE_READ"@     MAX_STRING_LENGTH"@     ADSSTATE_SHUTDOWN"@     AMSPORT_R0_CAM"@     ADSIGRP_SYMVAL"@     ADSIGRP_IOIMAGE_RWIX"@     ADSIGRP_SYM_DOWNLOAD"@     ADSIGRP_SYM_UPLOAD"@     TIMESERVICE_RTCTIMEDIFF"@     AMSPORT_R0_NC"@     ADSIGRP_IOIMAGE_RWIB"@     SYSTEMSTATEFLAGS_BSOD"@     AMSPORT_R0_CNC"@     BOOTDATAFLAGS_RETAIN_INVALID"@     ADSIOFFS_DEVDATA_ADSSTATE"@     ADSSTATE_SAVECFG"@     SYSTEMSERVICE_OPENWRITE"@     SYSTEMSERVICE_FCLOSE"@     ADSSTATE_CONFIG"@     ADSSTATE_POWERGOOD"@     ADSIGRP_SYM_INFOBYNAME"@     SYSTEMSERVICE_FSCANF"@     ADSIGRP_SYM_UPLOADINFO"@     SYSTEMSERVICE_OPENREAD"@     SYSTEMSERVICE_RMDIR"@     TIMESERVICE_ADJUSTTIMETORTC"@     TCEVENTFLAG_SRCID"@     SYSTEMSERVICE_FPUTS"@     FOPEN_MODEAPPEND"@     ADSLOG_MSGTYPE_MSGBOX"@     ADSSTATE_SUSPEND"@  
   FILE_WRITE"@     SYSTEMSERVICE_SENDEMAIL"@     SYSTEMSTATEFLAGS_RTVIOLATION"@     AMSPORT_R0_ISG"@     AMSPORT_R0_PLC_RTS4"@     TIMESERVICE_DATEANDTIME"@     AMSPORT_R0_PLC_RTS3"@     TCEVENTFLAG_AUTOFMTALL"@     AMSPORT_R0_PLC_RTS2"@     AMSPORT_R0_PLC_RTS1"@     TCEVENTFLAG_MSGBOX"@     FILE_OPENWRITE"@      BOOTDATAFLAGS_PERSISTENT_INVALID"@     ADSSTATE_RUN"@     ADSSTATE_POWERFAILURE"@     ADSLOG_MSGTYPE_ERROR"@     FOPEN_MODEWRITE"@     ADSLOG_MSGTYPE_WARN"@     AMSPORT_EVENTLOG"@     AMSPORT_R3_SCOPESERVER"@     SYSTEMSERVICE_STARTPROCESS"@     ADSLOG_MSGTYPE_LOG"@     ADSLOG_MSGTYPE_HINT"@     FOPEN_MODEPLUS"@     AMSPORT_R0_NCSVB"@     TCEVENTFLAG_LOG"@     SYSTEMSERVICE_MKDIR"@     ADSLOG_MSGTYPE_RESOURCE"@     TIMESERVICE_SYSTEMTIMES"@     ADSIGRP_IOIMAGE_CLEARI"@     AMSPORT_R0_LINE"@     ADSIGRP_IOIMAGE_CLEARO"@     FOPEN_MODETEXT"@     SYSTEMSERVICE_FPRINTF"@     ADSSTATE_RESUME"@     TCEVENTSTATE_RESET"@     AMSPORT_R0_IO"@     TCEVENTFLAG_PRIOCLASS"@     ADSSTATE_INVALID"@     DEFAULT_ADS_TIMEOUT"@     ADSIGRP_SYM_VALBYNAME"@     TCEVENTSTATE_SIGNALED"@     ADSIGRP_DEVICE_DATA"@     ADSSTATE_ERROR"@     AMSPORT_R0_RTIME"@     ADSIGRP_SYM_RELEASEHND"@     TIMESERVICE_TIMEZONINFORMATION"@     SYSTEMSERVICE_CREATEFILE"@     TCEVENT_SRCNAMESIZE"@     ADSSTATE_INIT"@     SYSTEMSERVICE_FREAD"@     ADSIGRP_SYMTAB"@     SYSTEMSERVICE_TIMESERVICES"@     ADSSTATE_STOP"@     FILE_SEEKBEGIN"@     ADSSTATE_MAXSTATES"@     SYSTEMSERVICE_OPENCREATE"@     SYSTEMSERVICE_FOPEN"@     AMSPORT_R3_SYSSERV"@     ADSIOFFS_DEVDATA_DEVSTATE"@     FOPEN_MODEBINARY"@     AMSPORT_R0_PLC"@     ADSIGRP_SYM_VALBYHND"@     ADSIGRP_IOIMAGE_RWOX"@   @        FW_Floor @                  FW_LrealFrac @          FW_LrealModP @          FW_LrealTrunc @                  F_GetVersionTcMath @                   FLOOR @       
   FRAC @       
   LMOD @          LTRUNC @          MODABS @          MODTURNS @           ~   ?
  _F_AxisState @_      _E_ParameterType       _E_ReadWriteParameterMode       _E_TcMC_STATES       _E_TcNC_CmdState        _E_TcNC_CmdTypeNewTargPosAndVelo       _E_TcNC_PosSetType       _E_TcNC_ServoState       _E_TcNC_SlaveTypes       _E_TcNC_StartPosType       _E_TcNC_TargPosType       _E_TouchProbeState       _InternalAxisRefData       _ST_FunctionBlockResults       _ST_NCADS_Axis       _ST_NCADS_AxisFunctions       _ST_NCADS_AxisParameter       _ST_NCADS_AxisState       _ST_NCADS_IDXOFFS_AxisFunctions       _ST_NCADS_IDXOFFS_AxisParameter       _ST_NCADS_IDXOFFS_AxisState        _ST_NCADS_IDXOFFS_TableFunctions        _ST_NCADS_IDXOFFS_TableParameter       _ST_NCADS_Table       _ST_NCADS_TableFunctions       _ST_NCADS_TableParameter       _ST_ParaStruct       _ST_TcNC_Compensation2       _ST_TcNC_CoupleSlave       _ST_TcNC_CoupleSlaveMultiMaster        _ST_TcNC_CoupleSlaveMultiMaster2       _ST_TcNC_DecoupleSlave       _ST_TcNc_OperationModes       _ST_TcNC_PhasingRequest        _ST_TcNC_SetEncoderSaclingFactor       _ST_TcNC_SetPos       _ST_TcNC_SetPosOnTheFly       _ST_TcNC_StopInfoRequest       _ST_TcNC_StopInfoResponse       _ST_TcNc_TouchProbeActivation       _ST_TcNc_TouchProbeDeactivation        _ST_TcNc_TouchProbeStatusRequest    !   _ST_TcNc_TouchProbeStatusResponse    !   _ST_TcNC_UnversalAxisStartRequest    "   _ST_TcNC_UnversalAxisStartResponse       AXIS_REF       E_AxisErrorCodes       E_AxisPositionCorrectionMode       E_DestallDetectMode       E_DestallMode       E_DisableMode    	   E_JogMode       E_PhasingType       E_PositionType    
   E_ReadMode       E_SetScalingFactorMode       E_SignalEdge       E_SignalSource       E_SuperpositionAbortOption       E_SuperpositionMode       E_TouchProbe       E_TouchProbeMode       E_WorkDirection       MC_AxisParameter       MC_AxisStates       MC_BufferMode       MC_Direction       MC_HomingMode       MC_TouchProbeRecordedData       NCTOPLC_AXIS_REF       PLCTONC_AXIS_REF       ST_AdsAddress       ST_AxisComponents       ST_AxisOpModes       ST_AxisParameterSet       ST_AxisStatus       ST_BacklashCompensationOptions       ST_DriveAddress       ST_GearInDynOptions       ST_GearInMultiMasterOptions       ST_GearInOptions       ST_GearOutOptions       ST_HomingOptions       ST_McOutputs       ST_MoveOptions       ST_NcApplicationRequest       ST_PhasingOptions       ST_PositionCompensationOptions    #   ST_PositionCompensationTableElement    %   ST_PositionCompensationTableParameter       ST_PowerStepperStruct       ST_SetEncoderScalingOptions       ST_SetPositionOptions       ST_SuperpositionOptions       ST_TableCharacValues       TRIGGER_REF                  _F_GetIndexGroup @          _F_NcCycleCounterUpdated @          _F_ReadStatus @          _F_TcMC_DWORD_TO_HEXSTR @          _F_TcMC_Round @          _F_UpdateNcCycleCounter @          _FB_MoveUniversalGeneric @       /   _FB_MoveUniversalGeneric.ActCalcDiffCmdNo @       .   _FB_MoveUniversalGeneric.ActErrorMessage @       8   _FB_MoveUniversalGeneric.ActMonitorContinousMotion @       7   _FB_MoveUniversalGeneric.ActMonitorDiscreteMotion @       -   _FB_MoveUniversalGeneric.ActMonitorStop @       0   _FB_MoveUniversalGeneric.ActNcCycleCounter @          _FB_PhasingGeneric @       '   _FB_PositionCorrectionTableLookup @       B   _FB_PositionCorrectionTableLookup.ActIsCompensationDirection @          _FB_ReadWriteParameter @          _FBAXIS_REF @          _FBAXIS_REF.ReadStatus @          _MC_HaltPhasing @          _MC_PhasingAbsolute @          _MC_PhasingRelative @          _TcMC_ADSRDWRT @          _TcMC_ADSREAD @          _TcMC_ADSWRITE @          _TCMCGLOBAL @           _TCMCGLOBAL.ReadDeviceInfo @          F_AxisCamDataQueued @          F_AxisCamScalingPending @          F_AxisCamTableQueued @          F_AxisControlLoopClosed @          F_AxisExternalLatchValid @           F_AxisGotNewTargetPosition @          F_AxisHasBeenStopped @          F_AxisHasExtSetPointGen @          F_AxisHasJob @          F_AxisInErrorState @          F_AxisInPositionWindow @          F_AxisInProtectedMode @          F_AxisInPTPmode @          F_AxisIoDataIsInvalid @          F_AxisIsAtTargetPosition @          F_AxisIsCalibrated @          F_AxisIsCalibrating @          F_AxisIsCompensating @          F_AxisIsCoupled @          F_AxisIsMoving @          F_AxisIsMovingBackwards @          F_AxisIsMovingEndless @          F_AxisIsMovingForward @          F_AxisIsNotMoving @          F_AxisIsReady @          F_AxisJobPending @           F_AxisMotionCommandsLocked @          F_AxisPhasingActive @       #   F_AxisReachedConstantVelocity @          F_GetVersion_TcMC2 @          MC_AbortSuperposition @          MC_AbortTrigger @          MC_AbortTrigger_V2_00 @          MC_BacklashCompensation @          MC_ExtSetPointGenDisable @          MC_ExtSetPointGenEnable @          MC_ExtSetPointGenFeed @          MC_GearIn @          MC_GearIn.ActGearInDyn @          MC_GearIn.WriteGearRatio @          MC_GearInDyn @          MC_GearInMultiMaster @       1   MC_GearInMultiMaster.ActGearInMultiMasterV1 @       1   MC_GearInMultiMaster.ActGearInMultiMasterV2 @          MC_GearOut @          MC_Halt @          MC_Home @          MC_Jog @          MC_Jog.ActCheckJogEnd @          MC_Jog.ActJogMove @          MC_MoveAbsolute @          MC_MoveAdditive @          MC_MoveContinuousAbsolute @          MC_MoveContinuousRelative @          MC_MoveModulo @       %   MC_MoveModulo.MC_MoveModuloCall @          MC_MoveRelative @          MC_MoveSuperImposed @          MC_MoveVelocity @          MC_OverrideFilter @       "   MC_PositionCorrectionLimiter @          MC_Power @          MC_PowerStepper @          MC_ReadActualPosition @          MC_ReadActualVelocity @          MC_ReadApplicationRequest @          MC_ReadAxisComponents @          MC_ReadAxisError @          MC_ReadBoolParameter @          MC_ReadDriveAddress @          MC_ReadParameter @          MC_ReadParameterSet @       2   MC_ReadParameterSet.ActGetSizeOfParameterSet @          MC_ReadStatus @          MC_ReadStopInfo @          MC_Reset @       $   MC_SetAcceptBlockedDriveSignal @           MC_SetEncoderScalingFactor @          MC_SetOverride @          MC_SetPosition @          MC_Stop @          MC_Stop.ActStop @       '   MC_TableBasedPositionCompensation @          MC_TouchProbe @       )   MC_TouchProbe.ActTouchProbeActivate @       0   MC_TouchProbe.ActTouchProbeMonitorActivity @       2   MC_TouchProbe.ActTouchProbeMonitorLatchValid @       0   MC_TouchProbe.ActTouchProbeMonitorPlcEvent @       ,   MC_TouchProbe.ActTouchProbeStartupInit @          MC_TouchProbe_V2_00 @       /   MC_TouchProbe_V2_00.ActTouchProbeActivate @       6   MC_TouchProbe_V2_00.ActTouchProbeMonitorActivity @       1   MC_TouchProbe_V2_00.ActTouchProbeMonitoring @       8   MC_TouchProbe_V2_00.ActTouchProbeMonitorLatchValid @       6   MC_TouchProbe_V2_00.ActTouchProbeMonitorPlcEvent @       2   MC_TouchProbe_V2_00.ActTouchProbeStartupInit @          MC_WriteBoolParameter @          MC_WriteParameter @          I   TcMC_GlobalConstants @      DEFAULT_HOME_POSITION H  
   TcMcGlobal H   H        CONCAT @                	   CTD @        	   CTU @        
   CTUD @           DELETE @           F_TRIG @        
   FIND @           INSERT @        
   LEFT @        	   LEN @        	   MID @           R_TRIG @           REPLACE @           RIGHT @           RS @        
   SEMA @           SR @        	   TOF @        	   TON @           TP @              Global Variables 0 @     @     ?   F_GetVersionTcpIp @      E_WinsockError       ST_SockAddr       ST_TcIpConnSvrResponse       ST_TcIpConnSvrUdpBuffer    	   T_HSOCKET                  FB_SocketAccept @          FB_SocketClose @          FB_SocketCloseAll @          FB_SocketConnect @          FB_SocketListen @          FB_SocketReceive @          FB_SocketSend @       %   FB_SocketUdpAddMulticastAddress @          FB_SocketUdpCreate @       &   FB_SocketUdpDropMulticastAddress @          FB_SocketUdpReceiveFrom @          FB_SocketUdpSendTo @             Global_Variables @          ?   F_CreateServerHnd @      E_ConnEstablishState       E_SocketAcceptMode       E_SocketConnectionState    	   T_HSERVER       T_ThrottleTimes                   F_GetVersionTcSocketHelper @          FB_ClientServerConnection @          FB_ServerClientConnection @          FB_SocketListenEx @          FB_SocketReceiveEx @          FB_ThrottleTimer @          FB_ThrottleTimer.MaxSpeed @          FB_ThrottleTimer.MinSpeed @          FB_ThrottleTimer.SlowDown @          FB_ThrottleTimer.SpeedUp @          HSOCKET_TO_STRING @             Globale_Variablen @          ?   F_BitArr_To_USINT @   	   LogStruct    	   ST_Result       	   BitButton    	   MotorHand       MotorHandSH       Sortimat       ZYLINDER       ZylinderHand       ZylinderHandSt              F_DEG_TO_RAD @          F_InBorders_INT @          F_InBorders_REAL @          F_Inch_To_mm @          F_LinFkt @          F_mm_To_Inch @          F_RAD_TO_DEG @          F_ResultReset @          F_RoundREAL @          F_SplitString @          F_USINT_To_BitArr @          FB_Blink @          FB_CheckParity @          FB_ClearFileByTime @       '   FB_ClearFileByTime._CallInstances @       &   FB_ClearFileByTime.ClearOldStuff @          FB_LogFile @          FB_LogFile.CallInstances @          FB_LogFile.LogMessage @          FB_RTC @          FB_StopUhr @          FB_Takt @          FB_Timer @             Globale_Variablen @          I   F_GetVersionTcSUPS @      E_S_UPS_Mode       E_S_UPS_State                  FB_NT_QuickShutdown @          FB_S_UPS @             Global_Variables @          V   F_GetVersionTcXmlDataSrv @      ST_SymAddrInfo       ST_XmlSrvOpenEntry                  FB_SymNameByAddr @          FB_XmlSrvRead @          FB_XmlSrvReadByName @          FB_XmlSrvWrite @          FB_XmlSrvWriteByName @             Global_Variables @          Internal_Global_Variables @                       , x x ??           2                ????????????????  
             ????                 ????, Z Z fy                      POUs
               _Main               Functionblocks
                 FB_CheckState                    FB_Home                     FB_Home_new  J                  FB_Instances                     FB_LedBlinking                     FB_Move                     FB_MoveInterpolation                     FB_ReadPositionFromFifo                     FB_ReadPositionWindowValue                     FB_Reset     ????           	   Functions	                Check Arm State                 F_CheckAxisHaveStopped                     F_CheckIfAllAxisHomed                     F_CheckIfAxisDisabled                     F_CheckIfAxisEnabled                     F_CheckIfAxisHasError                     F_CheckIfAxisHasJob                     F_CheckIfAxisIsMoving                     F_CheckIfAxisNotMoving                     F_CheckIfHomingIsBusy                     F_CheckIfInPosArea                      F_CheckIfInTargetPos  !                   F_CheckIfNextPosIsOutOfPosArea  "                   F_CheckIfSoftLimitMax  #                   F_CheckIfSoftLimitMin  $   ????               Check Positions                 F_CheckAtFinalGraspFromFloorPos  %                   F_CheckAtFinalGraspTrayPosition  &                $   F_CheckAtFinalGraspTurntablePosition  '                %   F_CheckAtFinalPutObjecttoTrayPosition  (                $   F_CheckAtFinalStoreTurntablePosition  )                   F_CheckAtHomePosition  *                   F_CheckAtLearningPosition  +                   F_CheckAtMoveArmOutPos  ,                   F_CheckAtMoveArmOutPos2  %                  F_CheckAtMoveArmOutPos3  $                  F_CheckAtPreGraspFromFloorPos  -                   F_CheckAtPreGraspFromTablePos  #                  F_CheckAtPregraspTrayPos1  .                   F_CheckAtPregraspTrayPos2  /                   F_CheckAtPregraspTrayPos3  0                #   F_CheckAtPregraspTurntablePosition1  1                #   F_CheckAtPregraspTurntablePosition2  2                   F_CheckAtPrePutObjectToTrayPos1  3                   F_CheckAtPrePutObjectToTrayPos2  4                   F_CheckAtPrePutObjectToTrayPos3  5                #   F_CheckAtPreStoreTurntablePosition1  6                #   F_CheckAtPreStoreTurntablePosition2  7                   F_CheckAtTurntableCCWPosition  8                   F_CheckAtTurntableCWPosition  9   ????              Homing                 FB_HomeJoint  =                  FB_SetCamSearchDirection  G                  FB_SetDefaulHomingDirection  i  ????              ReferenceDialog                 F_CheckJointSide  8                  F_CheckStartRef  `                  F_ResetDialog  g                  F_ResetStartRefDialog  e                  F_SetRefDialog  b                  F_SetStartRefDialog  c  ????              StateMachine                 F_SwitchStep  ?  ????                F_CalcVelocity  :                   F_ResetCommands  ;                   F_SetHomingDisable  <                   F_SetHomingEnable  =   ????????              TCP               _TCPIPClient               Functionblocks                FB_ClientApplication                ProcessReceiveData  A                  ProcessSendData  B   >                   FB_LocalClient  C   ????           	   Functions                 F_MergeSendData  D   ????????              _TCPIPServer               Functionblocks                 FB_LocalServer  E                  FB_ServerApplication                ProcessReceivedData  H   F   ????           	   Functions                 F_CutString  I                   F_MergeSendArmState  J                   F_MergeString  K                   F_MergeStringOneValue  L                   F_MergeStringValue  M   ????              Helper POUs Server                 F_ADSLOGERROR  N                   F_ADSLOGSTRING  O                  FB_FrameStringFifo             	   A_AddTail  T                  A_RemoveHead  U                  A_Reset  V   P                  FB_PositionFifoBuffer             	   A_AddTail  [                  A_RemoveHead  \                  A_Reset  ]   W                   FB_ProtErrorFifo             	   A_AddTail  b                  A_RemoveHead  c                  A_Reset  d   ^                
   SCODE_CODE  e   ????????????                F_CheckAtCandlePos  *                  F_CheckAxisInPositionWindow  0                  F_CheckIfInTargetPosEx  /                  FB_MoveToZero  4                 Main                CallFBs  q                  CallGraspTurntable  r                  CallMoveGraspFromFloor  s                  CallMovePreGraspFromFloor  t                  CallMovePreGraspTable  !                 CallMoveSingleAxis  u                  CallMoveToCandlePos  0                 CallMoveToHome  5                 CallMoveToTray  v                  CallPutLearningObjectToTray  w                  CallStoreTurntable  x                  CallWaitForCommand  y                  Init  Y                 ReadPLCStatus  z   f                   PRG_DataAccess  I                  TCPIPClient  {                   TCPIPServer  |   ????           
   Data types            
   ArmControl              
   E_Decision  h                  E_HomingJoint  >                  E_JointSide  7                  E_STATE  }                   E_STATE_CANDLE  ,                  E_STATE_GRASPFLOOR  ~                   E_STATE_HOME  6                 E_STATE_HOMING                     E_STATE_INTERPOLATION  ?                   E_STATE_JOG  ?                   E_STATE_MOVE  ?                   E_STATE_OBJECT_TO_TRAY  ?                   E_STATE_PREGRASPFLOOR  ?                   E_STATE_PREGRASPTABLE                     E_STATE_RESET  ?                   E_STATE_TRAY  ?                   E_STATE_TURNTABLE  ?                   E_STATE_VELOCITY  ?                   ST_ArmState  ?                   ST_Instances  ?                   ST_Instances_Input  ?                   ST_Instances_Output  ?                   ST_ReferenceDialog  Z                  ST_StartReferenceDialog  X                  ST_TurnTableState  K  ????              TCPIP                Client                 E_CLIENT_STATE  ?   ????               Server              
   E_RX_STATE  ?                   E_SERVER_STATE  ?   ????                ST_Commands  ?   ????                E_SystemMode  V                  ST_Parameters  H  ????              Visualizations                 ArmState  )                 Commands  (                  ProgramSTATEs  ?   ????              Global Variables                 Globale_Constant                   Globale_Variablen                     TwinCAT_Configuration                     Variablen_Konfiguration     ????                                                             K?Q                         	   localhost            P      	   localhost            P      	   localhost            P     i?xR    ǿ?+