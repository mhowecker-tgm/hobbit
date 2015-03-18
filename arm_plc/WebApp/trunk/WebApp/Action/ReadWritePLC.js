(function () {

    var NETID = ""; // Empty string for local machine;
    var PORT = "801"; // PLC Runtime
    //var SERVICE_URL = "http://localhost/RobotArmWebApp/TcAdsWebService.dll"; // HTTP path to the TcAdsWebService;
    var SERVICE_URL = "http://192.168.2.190/RobotArmWebApp/TcAdsWebService.dll"; // HTTP path to the TcAdsWebService;

    var client = new TcAdsWebService.Client(SERVICE_URL, null, null);

    var general_timeout = 500;

    var readLoopID1 = null;
    var readLoopID2 = null;
    var connected = false;
    
    var readLoopDelay = 500;

    var readSymbolValuesData = null;

    // Array of symbol names to read;
    var handlesVarNamesRead = [
        // Arm States:
        ".stCommands.ArmState.ARM_IN_POS_AREA",
        ".stCommands.ArmState.ARM_IN_TARGET_POS",
        ".stCommands.ArmState.ARM_SOFTLIMIT_MAX",
        ".stCommands.ArmState.ARM_SOFTLIMIT_MIN",
        ".stCommands.ArmState.ARM_HAS_ERROR",
        ".stCommands.ArmState.ARM_IS_DISABLED",
        ".stCommands.ArmState.ARM_HOMED",
        ".stCommands.ArmState.ARM_IS_MOVING",
        ".stCommands.ArmState.ARM_STOPPED",
        ".stCommands.ArmState.GRIPPER_IS_CLOSED",
        //".stCommands.PositionsForInterpolaationReady",

        // Position States:
        ".stCommands.ArmState.AT_HOME_POS",
        ".stCommands.ArmState.AT_LEARNING_POS",
        ".stCommands.ArmState.AT_TRAY_POS",
        ".stCommands.ArmState.AT_TURNTABLE_POS",
        ".stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS",
        ".stCommands.ArmState.AT_PREGRASPFROMTABLE_POS",
        ".stCommands.ArmState.AT_CCW_POS",
        ".stCommands.ArmState.AT_CW_POS",
		".stCommands.ArmState.AT_CANDLE_POS",

        // Actual Position:
        ".stCommands.GetActualPos",

        // Emergency Button: (�ffner)
        ".bEmergencyButton",
        
        // Home Reference Switches
        ".bHomeRefAxis1",
        ".bHomeRefAxis2",
        ".bHomeRefAxis3",
        ".bHomeRefAxis4",
        ".bHomeRefAxis5",
        ".bHomeRefAxis6",
        
        //referende Dialog
        ".stReferenceDialog.bActive",
        ".stReferenceDialog.nJoint"
    ];
    
    var SizeOfReadValues =  27*1 + 6*8 + 1*2; // 28 BOOL + 6 LREAL Values + 1 INT Value
        
    
    var handlesVarNamesWrite = [
        // Arm Commands
        //".stCommands.SetAbsolutePosition",
        //".stCommands.SetAbsolutePositionValue",
        //".stCommands.SetStartMove",
        //".stCommands.SetMoveVelocity",
        ".stCommands.SetMoveToHomePos",
        ".stCommands.SetMoveToLearningPos",
        ".stCommands.SetTurnTurntableCW",
        ".stCommands.SetTurnTurntableCCW",
        ".stCommands.SetStoreTurntable",
        ".stCommands.SetMoveToTray",
        ".stCommands.SetMovePreGraspFromFloor",
        ".stCommands.SetMovePreGraspFromTable",
        ".stCommands.SetStartAllAxisRef",
        ".stCommands.SetDisableAllAxis",
        ".stCommands.SetEnableAllAxis",
        ".stCommands.SetOpenGripper",
        ".stCommands.SetCloseGripper",
        ".stCommands.SetReset",
        ".stCommands.SetStopArm",
        ".stCommands.SetClearPosBuffer",
        ".stCommands.SetAllPositionsToZero",
        ".stCommands.SetJogMovePosAxis1",
        ".stCommands.SetJogMoveNegAxis1",
        ".stCommands.SetJogMovePosAxis2",
        ".stCommands.SetJogMoveNegAxis2",
        ".stCommands.SetJogMovePosAxis3",
        ".stCommands.SetJogMoveNegAxis3",
        ".stCommands.SetJogMovePosAxis4",
        ".stCommands.SetJogMoveNegAxis4",
        ".stCommands.SetJogMovePosAxis5",
        ".stCommands.SetJogMoveNegAxis5",
        ".stCommands.SetJogMovePosAxis6",
        ".stCommands.SetJogMoveNegAxis6",
	".stCommands.SetMoveToCandlePos",
    ];

    // Symbo handle variables;
    // Arm States
    var hArmInPosArea = null;
    var hArmInTargetPos = null;
    var hArmSoftLimitMax = null;
    var hArmSoftLimitMin = null;
    var hArmHasError = null;
    var hArmIsDisabled = null;
    var hArmHomed = null;
    var hArmIsMoving = null;
    var hArmStopped = null;
    var hGripperIsClosed = null;
    // Position States
    var hAtHomePos = null;
    var hAtLearningPos = null;
    var hAtTrayPos = null;
    var hAtTurntablePos = null;
    var hAtPreGraspFloor = null;
    var hAtPreGraspTable = null;
    var hAtCCWPos = null;
    var hAtCWPos = null;
	var hAtCandlePos = null;
    // Actual Position:
    var hActPos = null;
    // Emergency Button:
    var hEmergency = null;
    // Reference Switches:
    var hHomeRefAxis1 = null;
    var hHomeRefAxis2 = null;
    var hHomeRefAxis3 = null;
    var hHomeRefAxis4 = null;
    var hHomeRefAxis5 = null;
    var hHomeRefAxis6 = null;
    var hReferenceDialogActive = null;
    var hReferenceDialogNumber = null;    
    // Commands
    var handlesWrite = [];



    //  Read Variables, Global defined for use in other functions
    var bArmInPosArea = null;
    var bArmInTargetPos = null;
    var bArmSoftLimitMax = null;
    var bArmSoftLimitMin = null;
    var bArmHasError = null;
    var bArmIsDisabled = null;
    var bArmHomed = null;
    var bArmIsMoving = null;
    var bArmStopped = null;
    var bGripperIsClosed = null;
    var bAtHomePos =  null;
    var bAtLearningPos = null;
    var bAtTrayPos = null;
    var bAtTurntablePos = null;
    var bAtPreGraspFloor = null;
    var bAtPreGraspTable = null;
    var bAtCCWPos = null;
    var bAtCWPos = null;
    var bAtCandlePos = null;
    var fActPos = [];
    var bSOSButton = null;
    var bHomeRefAxis1 = null;
    var bHomeRefAxis2 = null;
    var bHomeRefAxis3 = null;
    var bHomeRefAxis4 = null;
    var bHomeRefAxis5 = null;
    var bHomeRefAxis6 = null;
    var bReferenceDialogActive = null;
    var nJointNo = null;

    // Base64 encoded binary data strings for write requests;
    var switchTrueBase64 = null;
    var switchFalseBase64 = null;

    var handlesVarNamesLength = (handlesVarNamesRead.length + handlesVarNamesWrite.length);
    var retryCounter;

    // Occurs if the window has loaded;
    window.onload = (function () {

        //create variable handles
        //CreateVarHandles();
            
        //start read-state loop
        readLoopID2 = window.setInterval(ReadLoop2, readLoopDelay);
    });


    var CreateVarHandles = (function(){
        
        // Create sumcommando for reading twincat symbol handles by symbol name;
        var handleswriter = new TcAdsWebService.DataWriter();

        //var handlesVarNamesLength = (handlesVarNamesRead.length + handlesVarNamesWrite.length);
        
        // Write general information for each symbol handle to the TcAdsWebService.DataWriter object;
        for (var i = 0; i < handlesVarNamesRead.length; i++) {
            handleswriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolHandleByName);
            handleswriter.writeDINT(0);
            handleswriter.writeDINT(4); // Expected size; A handle has a size of 4 byte;
            handleswriter.writeDINT(handlesVarNamesRead[i].length); // The length of the symbol name string;
        }
        for (var i = 0; i < handlesVarNamesWrite.length; i++) {
            handleswriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolHandleByName);
            handleswriter.writeDINT(0);
            handleswriter.writeDINT(4); // Expected size; A handle has a size of 4 byte;
            handleswriter.writeDINT(handlesVarNamesWrite[i].length); // The length of the symbol name string;
        }


        // Write symbol names after the general information to the TcAdsWebService.DataWriter object;
        for (var i = 0; i < handlesVarNamesRead.length; i++) {
            handleswriter.writeString(handlesVarNamesRead[i]);
        }
        for (var i = 0; i < handlesVarNamesWrite.length; i++) {
            handleswriter.writeString(handlesVarNamesWrite[i]);
        }
                        
        // Send the list-read-write command to the TcAdsWebService by use of the readwrite function of the TcAdsWebService.Client object;
        client.readwrite(
            NETID,
            PORT,
            0xF082, 	// IndexGroup = ADS list-read-write command; Used to request handles for twincat symbols;
            handlesVarNamesLength, // IndexOffset = Count of requested symbol handles;
            (handlesVarNamesLength * 4) + (handlesVarNamesLength * 8), // Length of requested data + 4 byte errorcode and 4 byte length per twincat symbol;
            handleswriter.getBase64EncodedData(),
            RequestHandlesCallback,
            null,
            general_timeout,
            RequestHandlesTimeoutCallback,
            true);
    });

    // Occurs if the readwrite for the sumcommando has finished;
    var RequestHandlesCallback = (function (e, s) {

        if (e && e.isBusy) {
            // HANDLE PROGRESS TASKS HERE;
            var message = "<img src='WebApp/Design/Img/RequestingHandle.png'>";//"Requesting Handles...";//
            // Arm States:
            document.getElementById("_ArmInPosArea").innerHTML = message;
            document.getElementById("_ArmInTargetPos").innerHTML = message;
            document.getElementById("_ArmSoftLimitMax").innerHTML = message;
            document.getElementById("_ArmSoftLimitMin").innerHTML = message;
            document.getElementById("_ArmHasError").innerHTML = message;
            document.getElementById("_ArmIsDisabled").innerHTML = message;
            document.getElementById("_ArmHomed").innerHTML = message;
            document.getElementById("_ArmIsMoving").innerHTML = message;
            document.getElementById("_ArmStopped").innerHTML = message;
            document.getElementById("_AtHomePos").innerHTML = message;
            document.getElementById("_AtLearningPos").innerHTML = message;
            document.getElementById("_AtTrayPos").innerHTML = message;
            document.getElementById("_AtTurntablePos").innerHTML = message;
            document.getElementById("_AtPreGraspFloor").innerHTML = message;
            document.getElementById("_AtPreGraspTable").innerHTML = message;
            document.getElementById("_AtCCWPos").innerHTML = message;
            document.getElementById("_AtCWPos").innerHTML = message;
            document.getElementById("_AtCandlePos").innerHTML = message;
            document.getElementById("_ActPos1").innerHTML = message;
            document.getElementById("_ActPos2").innerHTML = message;
            document.getElementById("_ActPos3").innerHTML = message;
            document.getElementById("_ActPos4").innerHTML = message;
            document.getElementById("_ActPos5").innerHTML = message;
            document.getElementById("_ActPos6").innerHTML = message;
            document.getElementById("_SOSButton").innerHTML = message;
            document.getElementById("_HomeRefAxis1").innerHTML = message;
            document.getElementById("_HomeRefAxis2").innerHTML = message;
            document.getElementById("_HomeRefAxis3").innerHTML = message;
            document.getElementById("_HomeRefAxis4").innerHTML = message;
            document.getElementById("_HomeRefAxis5").innerHTML = message;
            document.getElementById("_HomeRefAxis6").innerHTML = message;
            return;
        }

        if (e && !e.hasError) {

            // Get TcAdsWebService.DataReader object from TcAdsWebService.Response object;
            var reader = e.reader;

            // Read error code and length for each handle;
            for (var i = 0; i < handlesVarNamesLength; i++) {

                var err = reader.readDWORD();
                var len = reader.readDWORD();

                if (err != 0) {
                    document.getElementById("div_log").innerHTML = "Handle error!";
                    return;
                }

            }

            // Read handles from TcAdsWebService.DataReader object;
            // Arm States
            hArmInPosArea = reader.readDWORD();
            hArmInTargetPos = reader.readDWORD();
            hArmSoftLimitMax = reader.readDWORD();
            hArmSoftLimitMin = reader.readDWORD();
            hArmHasError = reader.readDWORD();
            hArmIsDisabled = reader.readDWORD();
            hArmHomed = reader.readDWORD();
            hArmIsMoving = reader.readDWORD();
            hArmStopped = reader.readDWORD();
            hGripperIsClosed = reader.readDWORD();
            // Position States:
            hAtHomePos = reader.readDWORD();
            hAtLearningPos = reader.readDWORD();
            hAtTrayPos = reader.readDWORD();
            hAtTurntablePos = reader.readDWORD();
            hAtPreGraspFloor = reader.readDWORD();
            hAtPreGraspTable = reader.readDWORD();
            hAtCCWPos = reader.readDWORD();
            hAtCWPos = reader.readDWORD();
			hAtCandlePos = reader.readDWORD();
            // Actual Position:
            hActPos = reader.readDWORD();
            // Emergency Button:
            hSOSButton = reader.readDWORD();
            // Reference Switches:
            hHomeRefAxis1 = reader.readDWORD();
            hHomeRefAxis2 = reader.readDWORD();
            hHomeRefAxis3 = reader.readDWORD();
            hHomeRefAxis4 = reader.readDWORD();
            hHomeRefAxis5 = reader.readDWORD();
            hHomeRefAxis6 = reader.readDWORD();
            hReferenceDialogActive = reader.readDWORD();
            hReferenceDialogNumber = reader.readDWORD();
            // Commands
            for (var i = 0; i < handlesVarNamesWrite.length; i++) {
                handlesWrite[i] = reader.readDWORD();        
            }


            // Create sum commando to read symbol values based on the handle
            var readSymbolValuesWriter = new TcAdsWebService.DataWriter();

            //  ".stCommands.ArmState.ARM_IN_POS_AREA"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmInPosArea); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_IN_TARGET_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmInTargetPos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_SOFTLIMIT_MAX"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmSoftLimitMax); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_SOFTLIMIT_MIN"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmSoftLimitMin); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_HAS_ERROR"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmHasError); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_IS_DISABLED"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmIsDisabled); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_HOMED"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmHomed); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_IS_MOVING"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmIsMoving); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.ARM_STOPPED"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hArmStopped); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.GRIPPER_IS_CLOSED"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hGripperIsClosed); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.AT_HOME_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtHomePos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read

            //  ".stCommands.ArmState.AT_LEARNING_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtLearningPos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            //  ".stCommands.ArmState.AT_TRAY_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtTrayPos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            //  ".stCommands.ArmState.AT_TURNTABLE_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtTurntablePos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            //  ".stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtPreGraspFloor); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
               
            //  ".stCommands.ArmState.AT_PREGRASPFROMTABLE_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtPreGraspTable); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
          
            //  ".stCommands.ArmState.AT_CCW_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtCCWPos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            //  ".stCommands.ArmState.AT_CW_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtCWPos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
			
            //  ".stCommands.ArmState.AT_CANDLE_POS"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hAtCandlePos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            //  ".stCommands.GetActualPos"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hActPos); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(48); // size to read
               
            
            //  ".bEmergencyButton"
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hSOSButton); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read            

            //  Reference Switches:
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hHomeRefAxis1); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hHomeRefAxis2); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hHomeRefAxis3); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hHomeRefAxis4); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hHomeRefAxis5); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hHomeRefAxis6); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
            
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hReferenceDialogActive); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(1); // size to read
                        
            readSymbolValuesWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolValueByHandle); // IndexGroup
            readSymbolValuesWriter.writeDINT(hReferenceDialogNumber); // IndexOffset = The target handle
            readSymbolValuesWriter.writeDINT(2); // size to read
                                    
            // Get Base64 encoded data from TcAdsWebService.DataWriter;
            readSymbolValuesData = readSymbolValuesWriter.getBase64EncodedData();

            // Start cyclic reading of symbol values;
            readLoopID1 = window.setInterval(ReadLoop1, readLoopDelay);

        } else {

            if (e.error.getTypeString() == "TcAdsWebService.ResquestError") {
                // HANDLE TcAdsWebService.ResquestError HERE;
                document.getElementById("div_log").innerHTML = "Error: StatusText = " + e.error.statusText + " Status: " + e.error.status;
            }
            else if (e.error.getTypeString() == "TcAdsWebService.Error") {
                // HANDLE TcAdsWebService.Error HERE;
                document.getElementById("div_log").innerHTML = "Error: ErrorMessage = " + e.error.errorMessage + " ErrorCode: " + e.error.errorCode;
            }

        }

    });

    // Occurs if the readwrite for the sumcommando to request symbol handles runs into timeout;
    var RequestHandlesTimeoutCallback = (function () {
        // HANDLE TIMEOUT HERE;
        var delay=500; //500milliseconds
        setTimeout(function(){
            //retry after delay (for maximum 10 times)
            document.getElementById("div_log").innerHTML = "Request handles timeout!";
            if (retryCounter<10)
            {
                retryCounter++;
                EstablishConnection();
            }
        }, delay);
    });

    // Interval callback for cyclic reading;
    var ReadLoop2 = (function () {
        // Read ADS state
        client.readState(
            NETID,
            PORT,
            ReadStateCallback,
            null,
            general_timeout,
            ReadStateTimeoutCallback,
            true);
    
    });
    
    var ReadLoop1 = (function () {
        // Send the read-read-write command to the TcAdsWebService by use of the readwrite function of the TcAdsWebService.Client object;
        client.readwrite(
            NETID,
            PORT,
            0xF080, // 0xF080 = Read command;
            handlesVarNamesRead.length, // IndexOffset = Variables count;
            (SizeOfReadValues + (handlesVarNamesRead.length * 4)), // Length of requested data + 4 byte errorcode per variable;
            readSymbolValuesData,
            ReadCallback,
            null,
            general_timeout,
            ReadTimeoutCallback,
            true);


    });
    
    var ShowComErr = (function (text, show){   
        if (show)
            document.getElementById("ComErr").style.display= 'block';
        else
            document.getElementById("ComErr").style.display= 'none';
            
        document.getElementById("_AdsError").innerHTML = "Communication ERROR: " + text;
    });
    var UpdatePLCState = (function (state){
       document.getElementById("iFooter").innerHTML = "Current PLC State: " + state; 
    });
    
    // Occurs if the readState command has finished;
    var ReadStateCallback = (function (e, s) {

        if (e && e.isBusy) {
            // HANDLE PROGRESS TASKS HERE;
            // Exit callback function because request is still busy;
            return;
        }

        if (e && !e.hasError) {

            var reader = e.reader;
            var AdsStates = TcAdsWebService.AdsState;

            // Read error codes from begin of TcAdsWebService.DataReader object;
            var AdsStateNum =  reader.readBYTE();
            //console.log(AdsState);
            var DeviceStateNum = reader.readBYTE();
            
            switch (AdsStateNum) {
                case (AdsStates.RESET):
                    document.getElementById("_AdsState").innerHTML = "RESET";
                    ShowComErr("RESET", true); 
                    UpdatePLCState("RESET");
                    connected = false;
                    break;
                case (AdsStates.INIT):
                    document.getElementById("_AdsState").innerHTML = "INIT";
                    ShowComErr("INIT", true);
                    UpdatePLCState("INIT");
                    connected = false;
                    break;
                case (AdsStates.START):
                    document.getElementById("_AdsState").innerHTML = "START";
                    ShowComErr("START", false);
                    UpdatePLCState("START");
                    break;
                case (AdsStates.RUN):
                    document.getElementById("_AdsState").innerHTML = "RUN";
                    ShowComErr("RUN", false);
                    UpdatePLCState("RUN");
                    if (!connected)
                    {
                        CreateVarHandles();
                        connected = true;
                    }
                    break;
                case (AdsStates.STOP):
                    document.getElementById("_AdsState").innerHTML = "STOP";
                    ShowComErr("STOP", true);
                    UpdatePLCState("STOP");
                    connected = false;
                    break;
                case (AdsStates.CONFIG):
                    document.getElementById("_AdsState").innerHTML = "CONFIG";
                    ShowComErr("CONFIG", true); 
                    UpdatePLCState("CONFIG");
                    connected = false;
                    break;  
            }
            
        } else {                    
            //document.getElementById("ComErr").style.display= 'block'; 
            connected = false;

            if (e.error.getTypeString() == "TcAdsWebService.ResquestError") {
                // HANDLE TcAdsWebService.ResquestError HERE;
                document.getElementById("div_log").innerHTML = "Error: StatusText = " + e.error.statusText + " Status: " + e.error.status;
                ShowComErr(e.error.statusText + " " + e.error.status, true);
            }
            else if (e.error.getTypeString() == "TcAdsWebService.Error") {
                // HANDLE TcAdsWebService.Error HERE;
                document.getElementById("div_log").innerHTML = "Error: ErrorMessage = " + e.error.errorMessage + " ErrorCode: " + e.error.errorCode;
                ShowComErr(e.error.errorMessage + " " + e.error.errorCode, true);
            }
            else
                ShowComErr("UNKNOWN", true);
        }
        if (!connected && readLoopID1!==null)
        {
            window.clearInterval(readLoopID1);
        }
        
    });
    
    // Occurs if the readState command runs into timeout;
    var ReadStateTimeoutCallback = (function () {
        // HANDLE TIMEOUT HERE;
        document.getElementById("div_log").innerHTML = "Read State timeout!";
    });
    
    
    
    // Occurs if the read-read-write command has finished;
    var ReadCallback = (function (e, s) {

        if (e && e.isBusy) {
            // HANDLE PROGRESS TASKS HERE;
            // Exit callback function because request is still busy;
            return;
        }

        if (e && !e.hasError) {

            var reader = e.reader;

            // Read error codes from begin of TcAdsWebService.DataReader object;
            for (var i = 0; i < handlesVarNamesRead.length; i++) {
                var err = reader.readDWORD();
                if (err != 0) {
                    document.getElementById("div_log").innerHTML = "Symbol error!";
                    return;
                }
            }

            //  ".stCommands.ArmState.ARM_IN_POS_AREA" // BOOL
            bArmInPosArea = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_IN_TARGET_POS" // BOOL
            bArmInTargetPos = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_SOFTLIMIT_MAX" // BOOL
            bArmSoftLimitMax = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_SOFTLIMIT_MIN" // BOOL
            bArmSoftLimitMin = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_HAS_ERROR" // BOOL
            bArmHasError = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_IS_DISABLED" // BOOL
            bArmIsDisabled = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_HOMED" // BOOL
            bArmHomed = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_IS_MOVING" // BOOL
            bArmIsMoving = reader.readBOOL();

            //  ".stCommands.ArmState.ARM_STOPPED" // BOOL
            bArmStopped = reader.readBOOL();

            //  ".stCommands.ArmState.GRIPPER_IS_CLOSED" // BOOL
            bGripperIsClosed = reader.readBOOL();
            
            //  ".stCommands.ArmState.AT_HOME_POS"
            bAtHomePos =  reader.readBOOL();
            
            //  ".stCommands.ArmState.AT_LEARNING_POS"
            bAtLearningPos = reader.readBOOL();

            //  ".stCommands.ArmState.AT_TRAY_POS"
            bAtTrayPos = reader.readBOOL();

            //  ".stCommands.ArmState.AT_TURNTABLE_POS"
            bAtTurntablePos = reader.readBOOL();

            //  ".stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS"
            bAtPreGraspFloor = reader.readBOOL();

            //  ".stCommands.ArmState.AT_PREGRASPFROMFLOOR_POS"
            bAtPreGraspTable = reader.readBOOL();

            //  ".stCommands.ArmState.AT_CCW_POS"
            bAtCCWPos = reader.readBOOL();

            //  ".stCommands.ArmState.AT_CW_POS"
            bAtCWPos = reader.readBOOL();

            //  ".stCommands.ArmState.AT_CANDLE_POS"
            bAtCandlePos = reader.readBOOL();
            
            //  ".stCommands.GetActPos"            
            for (var i = 0; i < 6; i++) {
                fActPos[i]=reader.readLREAL();    
            }
            
            document.getElementById("_ActPos1").innerHTML = fActPos[0].toFixed(2);
            document.getElementById("_ActPos2").innerHTML = fActPos[1].toFixed(2);
            document.getElementById("_ActPos3").innerHTML = fActPos[2].toFixed(2);
            document.getElementById("_ActPos4").innerHTML = fActPos[3].toFixed(2);
            document.getElementById("_ActPos5").innerHTML = fActPos[4].toFixed(2);
            document.getElementById("_ActPos6").innerHTML = fActPos[5].toFixed(2);
            
            
            //  ".bEmergencyButton"
            bSOSButton = reader.readBOOL();  

            //  Reference Switches:
            bHomeRefAxis1 = reader.readBOOL();
            bHomeRefAxis2 = reader.readBOOL();
            bHomeRefAxis3 = reader.readBOOL();
            bHomeRefAxis4 = reader.readBOOL();
            bHomeRefAxis5 = reader.readBOOL();
            bHomeRefAxis6 = reader.readBOOL();
            
            //  ".stReferenceDialog.bActive"
            bReferenceDialogActive = reader.readBOOL();
            nJointNo = reader.readINT();

            // Variables for Picture location:
            var PicMoving = "<img src='WebApp/Design/Img/Moving.png'>";
            var PicStopped = "<img src='WebApp/Design/Img/Stopped.png'>";
            var PicError = "<img src='WebApp/Design/Img/Error.png'>";
            var PicHomed = "<img src='WebApp/Design/Img/Homed-green.png'>";
            var PicNotHomed = "<img src='WebApp/Design/Img/Homed-red.png'>";
            var PicTrue = "<img src='WebApp/Design/Img/True.png'>";
            var PicFalse = "<img src='WebApp/Design/Img/False.png'>";              
            var PicButtonRed = "WebApp/Design/Img/button-red.png";
            var PicButtonBlue = "WebApp/Design/Img/button-blue.png";
            var PicJoint1 = "<img src='WebApp/Design/img/Joint1.png' width='100%'>";
            var PicJoint2 = "<img src='WebApp/Design/img/Joint2.png' width='100%'>";
            var PicJoint3 = "<img src='WebApp/Design/img/Joint3.png' width='100%'>";
            var PicJoint4 = "<img src='WebApp/Design/img/Joint4.png' width='100%'>";
            var PicJoint5 = "<img src='WebApp/Design/img/Joint5.png' width='100%'>";
            var PicJoint6 = "<img src='WebApp/Design/img/Joint6.png' width='100%'>";
            
            // Write data to the "Actual Arm States" interface;
            if (bArmInPosArea) {
                document.getElementById("_ArmInPosArea").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmInPosArea").innerHTML = PicFalse;
            }

            if (bArmInTargetPos){
                document.getElementById("_ArmInTargetPos").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmInTargetPos").innerHTML = PicFalse;
            }

            if (bArmSoftLimitMax) {
                document.getElementById("_ArmSoftLimitMax").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmSoftLimitMax").innerHTML = PicFalse;
            }

            if (bArmSoftLimitMin) {
                document.getElementById("_ArmSoftLimitMin").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmSoftLimitMin").innerHTML = PicFalse;
            }

            if (bArmHasError) {
                document.getElementById("_ArmHasError").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmHasError").innerHTML = PicFalse;
            }

            if (bArmIsDisabled) {
                document.getElementById("_ArmIsDisabled").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmIsDisabled").innerHTML = PicFalse;
            }

            if (bArmHomed) {
                document.getElementById("_ArmHomed").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmHomed").innerHTML = PicFalse;
            }

            if (bArmIsMoving) {
                document.getElementById("_ArmIsMoving").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmIsMoving").innerHTML = PicFalse;
            }

            if (bArmStopped) {
                document.getElementById("_ArmStopped").innerHTML = PicTrue;
            } else {
                document.getElementById("_ArmStopped").innerHTML = PicFalse;
            }

            if (bGripperIsClosed) {
                document.getElementById("_GripperIsClosed").innerHTML = PicTrue;
            } else {
                document.getElementById("_GripperIsClosed").innerHTML = PicFalse;
            }
            
            if (bAtHomePos) {
                document.getElementById("_AtHomePos").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtHomePos").innerHTML = PicFalse;
            }
            
            if (bAtLearningPos) {
                document.getElementById("_AtLearningPos").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtLearningPos").innerHTML = PicFalse;
            }
            
            if (bAtPreGraspFloor) {
                document.getElementById("_AtPreGraspFloor").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtPreGraspFloor").innerHTML = PicFalse;
            }
            
            if (bAtPreGraspTable) {
                document.getElementById("_AtPreGraspTable").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtPreGraspTable").innerHTML = PicFalse;
            }
			
            if (bAtTrayPos) {
                document.getElementById("_AtTrayPos").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtTrayPos").innerHTML = PicFalse;
            }
            
            if (bAtTurntablePos) {
                document.getElementById("_AtTurntablePos").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtTurntablePos").innerHTML = PicFalse;
            }
            
            if (bAtCCWPos) {
                document.getElementById("_AtCCWPos").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtCCWPos").innerHTML = PicFalse;
            }
            
            if (bAtCWPos) {
                document.getElementById("_AtCWPos").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtCWPos").innerHTML = PicFalse;
            }
            if (bAtCandlePos) {
                document.getElementById("_AtCandlePos").innerHTML = PicTrue;
            } else {
                document.getElementById("_AtCandlePos").innerHTML = PicFalse;
            }
            
            if (!bSOSButton) {
                document.getElementById("_SOSButton").innerHTML = PicTrue;
            } else {
                document.getElementById("_SOSButton").innerHTML = PicFalse;
            }
            
            if (bHomeRefAxis1) {
                document.getElementById("_HomeRefAxis1").innerHTML = PicTrue;
                document.getElementById("__HomeRefAxis1").innerHTML = PicTrue;
            } else {
                document.getElementById("_HomeRefAxis1").innerHTML = PicFalse;
                document.getElementById("__HomeRefAxis1").innerHTML = PicFalse;
            }
            
            if (bHomeRefAxis2) {
                document.getElementById("_HomeRefAxis2").innerHTML = PicTrue;
                document.getElementById("__HomeRefAxis2").innerHTML = PicTrue;
            } else {
                document.getElementById("_HomeRefAxis2").innerHTML = PicFalse;
                document.getElementById("__HomeRefAxis2").innerHTML = PicFalse;
            }
            
            if (bHomeRefAxis3) {
                document.getElementById("_HomeRefAxis3").innerHTML = PicTrue;
                document.getElementById("__HomeRefAxis3").innerHTML = PicTrue;
            } else {
                document.getElementById("_HomeRefAxis3").innerHTML = PicFalse;
                document.getElementById("__HomeRefAxis3").innerHTML = PicFalse;
            }
            
            if (bHomeRefAxis4) {
                document.getElementById("_HomeRefAxis4").innerHTML = PicTrue;
                document.getElementById("__HomeRefAxis4").innerHTML = PicTrue;
            } else {
                document.getElementById("_HomeRefAxis4").innerHTML = PicFalse;
                document.getElementById("__HomeRefAxis4").innerHTML = PicFalse;
            }
            
            if (bHomeRefAxis5) {
                document.getElementById("_HomeRefAxis5").innerHTML = PicTrue;
                document.getElementById("__HomeRefAxis5").innerHTML = PicTrue;
            } else {
                document.getElementById("_HomeRefAxis5").innerHTML = PicFalse;
                document.getElementById("__HomeRefAxis5").innerHTML = PicFalse;
            }
            
            if (bHomeRefAxis6) {
                document.getElementById("_HomeRefAxis6").innerHTML = PicTrue;
                document.getElementById("__HomeRefAxis6").innerHTML = PicTrue;
            } else {
                document.getElementById("_HomeRefAxis6").innerHTML = PicFalse;
                document.getElementById("__HomeRefAxis6").innerHTML = PicFalse;
            }
            
            // Write data to the "Command" interface ui;
            // Start Move To Home Position:
            if (bAtHomePos) {
                //__AtHomePos.innerHTML = PicTrue;
                document.getElementById("__AtHomePos").innerHTML = PicTrue;
                document.getElementById("_picAtHomePos").src=PicButtonRed;         
            } else {
                document.getElementById("__AtHomePos").innerHTML = "";
                document.getElementById("_picAtHomePos").src=PicButtonBlue; 
            }
            
            // Start Move To Learning Position:
            if (bAtLearningPos) {
                document.getElementById("__AtLearningPos").innerHTML = PicTrue;
                document.getElementById("_picAtLearningPos").src=PicButtonRed;
            } else {
                document.getElementById("__AtLearningPos").innerHTML = "";
                document.getElementById("_picAtLearningPos").src=PicButtonBlue;
            }
            
            // Start Move To PreGrasp from Floor Position:
            if (bAtPreGraspFloor) {
                document.getElementById("__AtPreGraspFloor").innerHTML = PicTrue;
                document.getElementById("_picAtPreGraspFloor").src=PicButtonRed;
            } else {
                document.getElementById("__AtPreGraspFloor").innerHTML = "";
                document.getElementById("_picAtPreGraspFloor").src=PicButtonBlue;
            }
            
            // Start Move To PreGrasp from Table Position:
            if (bAtPreGraspTable) {
                document.getElementById("__AtPreGraspTable").innerHTML = PicTrue;
                document.getElementById("_picAtPreGraspTable").src=PicButtonRed;
            } else {
                document.getElementById("__AtPreGraspTable").innerHTML = "";
                document.getElementById("_picAtPreGraspTable").src=PicButtonBlue;
            }
			
            // Start Move To Tray Position:
            if (bAtTrayPos) {
                document.getElementById("__AtTrayPos").innerHTML = PicTrue;
                document.getElementById("_picAtTrayPos").src=PicButtonRed;
            } else {
                document.getElementById("__AtTrayPos").innerHTML = "";
                document.getElementById("_picAtTrayPos").src=PicButtonBlue;
            }
            
            // Start Store Turntable Position:
            if (bAtTurntablePos) {
                document.getElementById("__AtTurntablePos").innerHTML = PicTrue;
                document.getElementById("_picAtTurntablePos").src=PicButtonRed;
            } else {
                document.getElementById("__AtTurntablePos").innerHTML = "";
                document.getElementById("_picAtTurntablePos").src=PicButtonBlue;
            }
            
            // Start Turning Turntable Clockwise Position:
            if (bAtCWPos) {
                document.getElementById("__AtCWPos").innerHTML = PicTrue;
                document.getElementById("_picAtCWPos").src=PicButtonRed;
            } else {
                document.getElementById("__AtCWPos").innerHTML = "";
                document.getElementById("_picAtCWPos").src=PicButtonBlue;
            }
            
            // Start Turning Turntable Counter Clockwise Position:
            if (bAtCCWPos) {
                document.getElementById("__AtCCWPos").innerHTML = PicTrue;
                document.getElementById("_picAtCCWPos").src=PicButtonRed;
            } else {
                document.getElementById("__AtCCWPos").innerHTML = "";
                document.getElementById("_picAtCCWPos").src=PicButtonBlue;
            }
            
            // Start movint to Candle Position:
            if (bAtCandlePos) {
                document.getElementById("__AtCandlePos").innerHTML = PicTrue;
                document.getElementById("_picAtCandlePos").src=PicButtonRed;
            } else {
                document.getElementById("__AtCandlePos").innerHTML = "";
                document.getElementById("_picAtCandlePos").src=PicButtonBlue;
            }
            
            if (bArmIsMoving){
                document.getElementById("__AtHomePos").innerHTML = PicMoving;
                document.getElementById("_picAtHomePos").src=PicButtonRed;
                document.getElementById("__AtLearningPos").innerHTML = PicMoving;
                document.getElementById("_picAtLearningPos").src=PicButtonRed;
                document.getElementById("__AtPreGraspFloor").innerHTML=PicMoving;
                document.getElementById("_picAtPreGraspFloor").src=PicButtonRed;
                document.getElementById("__AtPreGraspTable").innerHTML=PicMoving;
                document.getElementById("_picAtPreGraspTable").src=PicButtonRed;
                document.getElementById("__AtCandlePos").innerHTML=PicMoving;
                document.getElementById("_picAtCandlePos").src=PicButtonRed;
                document.getElementById("__AtTrayPos").innerHTML = PicMoving;
                document.getElementById("_picAtTrayPos").src=PicButtonRed;
                document.getElementById("__AtTurntablePos").innerHTML = PicMoving;
                document.getElementById("_picAtTurntablePos").src=PicButtonRed;
                document.getElementById("__AtCWPos").innerHTML = PicMoving;
                document.getElementById("_picAtCWPos").src=PicButtonRed;
                document.getElementById("__AtCCWPos").innerHTML = PicMoving;
                document.getElementById("_picAtCCWPos").src=PicButtonRed;

            } else if (bArmHasError) {
                document.getElementById("__AtHomePos").innerHTML = PicError;
                document.getElementById("_picAtHomePos").src=PicButtonRed;
                document.getElementById("__AtLearningPos").innerHTML = PicError;
                document.getElementById("_picAtLearningPos").src=PicButtonRed;
                document.getElementById("__AtPreGraspFloor").innerHTML=PicError;
                document.getElementById("_picAtPreGraspFloor").src=PicButtonRed;
                document.getElementById("__AtPreGraspTable").innerHTML=PicError;
                document.getElementById("_picAtPreGraspTable").src=PicButtonRed;
                document.getElementById("__AtCandlePos").innerHTML=PicError;
                document.getElementById("_picAtCandlePos").src=PicButtonRed;
                document.getElementById("__AtTrayPos").innerHTML = PicError;
                document.getElementById("_picAtTrayPos").src=PicButtonRed;
                document.getElementById("__AtTurntablePos").innerHTML = PicError;
                document.getElementById("_picAtTurntablePos").src=PicButtonRed;
                document.getElementById("__AtCWPos").innerHTML = PicError;
                document.getElementById("_picAtCWPos").src=PicButtonRed;
                document.getElementById("__AtCCWPos").innerHTML = PicError;
                document.getElementById("_picAtCCWPos").src=PicButtonRed;
                
            } else if (bArmStopped && bArmHomed) {
                document.getElementById("__AtHomePos").innerHTML = PicStopped;
                document.getElementById("_picAtHomePos").src=PicButtonBlue;
                document.getElementById("__AtLearningPos").innerHTML = PicStopped;
                document.getElementById("_picAtLearningPos").src=PicButtonBlue;
                document.getElementById("__AtPreGraspFloor").innerHTML=PicStopped;
                document.getElementById("_picAtPreGraspFloor").src=PicButtonBlue;
                document.getElementById("__AtPreGraspTable").innerHTML=PicStopped;
                document.getElementById("_picAtPreGraspTable").src=PicButtonBlue;
                document.getElementById("__AtCandlePos").innerHTML=PicStopped;
                document.getElementById("_picAtCandlePos").src=PicButtonBlue;
                document.getElementById("__AtTrayPos").innerHTML = PicStopped;
                document.getElementById("_picAtTrayPos").src=PicButtonBlue;
                document.getElementById("__AtTurntablePos").innerHTML = PicStopped;
                document.getElementById("_picAtTurntablePos").src=PicButtonBlue;
                document.getElementById("__AtCWPos").innerHTML = PicStopped;
                document.getElementById("_picAtCWPos").src=PicButtonBlue;
                document.getElementById("__AtCCWPos").innerHTML = PicStopped;
                document.getElementById("_picAtCCWPos").src=PicButtonBlue;
                
            } else if (bArmStopped && !bArmHomed) {
                document.getElementById("__AtHomePos").innerHTML = PicStopped;
                document.getElementById("_picAtHomePos").src=PicButtonRed;
                document.getElementById("__AtLearningPos").innerHTML = PicStopped;
                document.getElementById("_picAtLearningPos").src=PicButtonRed;
                document.getElementById("__AtPreGraspFloor").innerHTML=PicStopped;
                document.getElementById("_picAtPreGraspFloor").src=PicButtonRed;
                document.getElementById("__AtPreGraspTable").innerHTML=PicStopped;
                document.getElementById("_picAtPreGraspTable").src=PicButtonRed;
                document.getElementById("__AtCandlePos").innerHTML=PicStopped;
                document.getElementById("_picAtCandlePos").src=PicButtonRed;
                document.getElementById("__AtTrayPos").innerHTML = PicStopped;
                document.getElementById("_picAtTrayPos").src=PicButtonRed;
                document.getElementById("__AtTurntablePos").innerHTML = PicStopped;
                document.getElementById("_picAtTurntablePos").src=PicButtonRed;
                document.getElementById("__AtCWPos").innerHTML = PicStopped;
                document.getElementById("_picAtCWPos").src=PicButtonRed;
                document.getElementById("__AtCCWPos").innerHTML = PicStopped;
                document.getElementById("_picAtCCWPos").src=PicButtonRed;
                
            } else if (!bArmHomed) {
                document.getElementById("__AtHomePos").innerHTML = PicStopped;
                document.getElementById("_picAtHomePos").src=PicButtonRed;
                document.getElementById("__AtLearningPos").innerHTML = PicStopped;
                document.getElementById("_picAtLearningPos").src=PicButtonRed;
                document.getElementById("__AtPreGraspFloor").innerHTML=PicStopped;
                document.getElementById("_picAtPreGraspFloor").src=PicButtonRed;
                document.getElementById("__AtPreGraspTable").innerHTML=PicStopped;
                document.getElementById("_picAtPreGraspTable").src=PicButtonRed;
                document.getElementById("__AtCandlePos").innerHTML=PicStopped;
                document.getElementById("_picAtCandlePos").src=PicButtonRed;
                document.getElementById("__AtTrayPos").innerHTML = PicStopped;
                document.getElementById("_picAtTrayPos").src=PicButtonRed;
                document.getElementById("__AtTurntablePos").innerHTML = PicStopped;
                document.getElementById("_picAtTurntablePos").src=PicButtonRed;
                document.getElementById("__AtCWPos").innerHTML = PicStopped;
                document.getElementById("_picAtCWPos").src=PicButtonRed;
                document.getElementById("__AtCCWPos").innerHTML = PicStopped;
                document.getElementById("_picAtCCWPos").src=PicButtonRed;
            }
            
            if (bArmHomed) {
                document.getElementById("__StartArmReference").innerHTML = PicHomed;
            } else {
                document.getElementById("__StartArmReference").innerHTML = PicNotHomed;
            }
            
            if (bArmIsDisabled){
                document.getElementById("__ArmDisabled").innerHTML = PicTrue;
                document.getElementById("_picArmDisabled").src = PicButtonRed;
                document.getElementById("__ArmEnabled").innerHTML = "";
                document.getElementById("_picArmEnabled").src=PicButtonBlue;
            } else {
                document.getElementById("__ArmDisabled").innerHTML = "";
                document.getElementById("_picArmDisabled").src=PicButtonBlue;
                document.getElementById("__ArmEnabled").innerHTML = PicTrue;
                document.getElementById("_picArmEnabled").src=PicButtonRed;
                
            }
            
            if (bGripperIsClosed) {
                document.getElementById("__GripperClosed").innerHTML = PicTrue;
                document.getElementById("_picGripperClosed").src=PicButtonRed;
                document.getElementById("__GripperOpen").innerHTML = "";
                document.getElementById("_picGripperOpen").src=PicButtonBlue;
            } else {
                document.getElementById("__GripperClosed").innerHTML = "";
                document.getElementById("_picGripperClosed").src=PicButtonBlue;
                document.getElementById("__GripperOpen").innerHTML = PicTrue;
                document.getElementById("_picGripperOpen").src=PicButtonRed;
            }
            
            if (bReferenceDialogActive)
            {
                document.getElementById("_ReferencingDialog").style.display= 'block';  
                var caption = "Referencing Joint " + nJointNo + ": which side is it standing?";               
                document.getElementById("_RefDialogCaption").innerHTML = caption;
                switch(nJointNo)
                {
                    
                    case 1:
                        document.getElementById("_ReferenceJoint").innerHTML = PicJoint1;
                        break;                        
                    case 2:                        
                        document.getElementById("_ReferenceJoint").innerHTML = PicJoint2;
                        break;
                    case 3:
                        document.getElementById("_ReferenceJoint").innerHTML = PicJoint3;
                        break;
                    case 4:
                        document.getElementById("_ReferenceJoint").innerHTML = PicJoint4;
                        break;
                    case 5:
                        document.getElementById("_ReferenceJoint").innerHTML = PicJoint5;
                        break;
                    case 6:
                        document.getElementById("_ReferenceJoint").innerHTML = PicJoint6;
                        break;
                        
                }
            }
            else
            {
                document.getElementById("_ReferencingDialog").style.display= 'none';
            }
            
            
            
            
            
        } else {

            if (e.error.getTypeString() == "TcAdsWebService.ResquestError") {
                // HANDLE TcAdsWebService.ResquestError HERE;
                document.getElementById("div_log").innerHTML = "Error: StatusText = " + e.error.statusText + " Status: " + e.error.status;
            }
            else if (e.error.getTypeString() == "TcAdsWebService.Error") {
                // HANDLE TcAdsWebService.Error HERE;
                document.getElementById("div_log").innerHTML = "Error: ErrorMessage = " + e.error.errorMessage + " ErrorCode: " + e.error.errorCode;
            }
        }

    });

    // Occurs if the read-read-write command runs into timeout;
    var ReadTimeoutCallback = (function () {
        // HANDLE TIMEOUT HERE;
        document.getElementById("div_log").innerHTML = "Read timeout!";
    });

    
        
    BtnWriteClicked = (function (buttonName) {
    //function BtnWriteClicked(buttonName) {
        var switchTrueBase64Writer = new TcAdsWebService.DataWriter();
        switchTrueBase64Writer.writeBOOL(true);
        switchTrueBase64 = switchTrueBase64Writer.getBase64EncodedData();
        
        var _buttonName = buttonName;
        
        var handleNum = null;
        
        
        switch (_buttonName){
            case "SetMoveToHomePos":
                handleNum = 0;
                break;
            case "SetMoveToLearningPos":
                handleNum = 1;
                break;
            case "SetTurnTurntableCW":
                handleNum = 2;
                break;
            case "SetTurnTurntableCCW":
                handleNum = 3;
                break;
			case "SetMoveToCandlePos":
				handleNum = 29;
				break;
            case "SetStoreTurntable":
                handleNum = 4;
                break;
            case "SetMoveToTray":
                handleNum = 5;
                break;
            case "SetMoveToPreGraspFromFloor":
                handleNum = 6;
                break;
            case "SetMoveToPreGraspFromTable":
                handleNum = 7;
                break;
            case "SetStartAllAxisRef":
                handleNum = 8;
                break;
            case "SetDisableAllAxis":
                handleNum = 9;
                break;
            case "SetEnableAllAxis":
                handleNum = 10;
                break;
            case "SetOpenGripper":
                handleNum = 11;
                break;
            case "SetCloseGripper":
                handleNum = 12;
                break;
            case "SetReset":
                handleNum = 13;
                break;
            case "SetStopArm":
                handleNum = 14;
                break;
            case "SetClearPosBuffer":
                handleNum = 15;
                break;
            case "SetAllPosToZero":
                handleNum = 16;
                break;
            case "Axis1Positive":
                handleNum = 17;
                break;
            case "Axis1Negative":
                handleNum = 18;
                break;
            case "Axis2Positive":
                handleNum = 19;
                break;
            case "Axis2Negative":
                handleNum = 20;
                break;
            case "Axis3Positive":
                handleNum = 21;
                break;
            case "Axis3Negative":
                handleNum = 22;
                break;
            case "Axis4Positive":
                handleNum = 23;
                break;
            case "Axis4Negative":
                handleNum = 24;
                break;
            case "Axis5Positive":
                handleNum = 25;
                break;
            case "Axis5Negative":
                handleNum = 26;
                break;
            case "Axis6Positive":
                handleNum = 27;
                break;
            case "Axis6Negative":
                handleNum = 28;
                break;
        }
        
        if (bArmIsMoving && (_buttonName!="SetStopArm") && (_buttonName!="SetReset")) {
            document.getElementById("div_log_write").innerHTML = "Arm Is Moving..."        
        } else {
            client.write(
                NETID,
                PORT,
                0x0000F005, // 0xF005 = Call Write by Handle Commando
                handlesWrite[handleNum], // IndexOffset = Count of requested variables.
                //1+4, // Length of requested data + 4 byte errorcode per variable.
                switchTrueBase64,
                WriteCallback,
                null,
                general_timeout,
                WriteTimeoutCallback,
                true);    
        }
            
    });
    
    BtnDialogResultClicked = (function(varname)
    {        
        var _varname = varname;
        
        //set command we want to write
        var dataWriter = new TcAdsWebService.DataWriter();
        dataWriter.writeBOOL(true);
        var buffer = dataWriter.getBase64EncodedData();
        
        var handleWriter = new TcAdsWebService.DataWriter();        
        handleWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolHandleByName);
        handleWriter.writeDINT(0);
        handleWriter.writeDINT(4); // Expected size; A handle has a size of 4 byte;
        handleWriter.writeDINT(_varname.length); // The length of the symbol name string;
        handleWriter.writeString(_varname);
        
        var handleBuffer = handleWriter.getBase64EncodedData();
                             
        // Send the list-read-write command to the TcAdsWebService by use of the readwrite function of the TcAdsWebService.Client object;
        var response =  client.readwrite(
            NETID,
            PORT,
            0xF082, 	// IndexGroup = ADS list-read-write command; Used to request handles for twincat symbols;
            1, // IndexOffset = Count of requested symbol handles;
            (4) + (8), // Length of requested data + 4 byte errorcode and 4 byte length per twincat symbol;
            handleBuffer,
            null, //callback
            null,
            null, //timeout
            null, //timeout callback
            false);
            
        if (!response.hasError)    
        {
            var err = response.reader.readDWORD();
            var len = response.reader.readDWORD();
            var handle = response.reader.readDWORD();
            
            if (err == 0)
            {
                //window.alert(handle);

                if (handle != null)
                {
                    //write value
                    client.write(
                        NETID,
                        PORT,
                        0x0000F005, // 0xF005 = Call Write by Handle Commando
                        handle, // IndexOffset = Count of requested variables.
                        //1+4, // Length of requested data + 4 byte errorcode per variable.
                        buffer,
                        null, //callback
                        null,
                        null, //timeout
                        null, //timeout callback
                        false);          
                        
                    var releaseWriter = new TcAdsWebService.DataWriter();        
                    releaseWriter.writeDINT(TcAdsWebService.TcAdsReservedIndexGroups.SymbolReleaseHandle);
                    releaseWriter.writeDINT(0);
                    releaseWriter.writeDINT(4); // Expected size; A handle has a size of 4 byte;
                    releaseWriter.writeDINT(handle);                       
                        
                    //release handle
                    var releaseResp =  client.readwrite(
                        NETID,
                        PORT,
                        0xF081, 	// IndexGroup = ADS list-read-write command; Used to request handles for twincat symbols;
                        1, // IndexOffset = Count of requested symbol handles;
                        (4), // 4 byte errorcode;
                        releaseWriter.getBase64EncodedData(),
                        null, //callback
                        null,
                        null, //timeout
                        null, //timeout callback
                        false);    
                }     
            }
            else 
                window.alert("Error: " + err);
        }
        else
        {            
            if (response.error.getTypeString() == "TcAdsWebService.ResquestError") {
                // HANDLE TcAdsWebService.ResquestError HERE;
                window.alert(response.error.statusText + " ErrorCode: " + response.error.status);
                //document.getElementById("div_log").innerHTML = "Error: StatusText = " + e.error.statusText + " Status: " + e.error.status;
            }
            else if (response.error.getTypeString() == "TcAdsWebService.Error") {
                // HANDLE TcAdsWebService.Error HERE;
                window.alert(response.error.errorMessage + " ErrorCode: " + response.error.errorCode);
                //document.getElementById("div_log").innerHTML = "Error: ErrorMessage = " + e.error.errorMessage + " ErrorCode: " + e.error.errorCode;
            }
            else
                window.alert("something went wrong");
        }
        
        
    });

    var WriteCallback = (function(e,s){

        if (e && e.isBusy) {
            // HANDLE PROGRESS TASKS HERE;
            var message = "Writing data to plc...";
            document.getElementById("div_log_write").innerHTML = message;
            // Exit callback function because request is still busy;
            return;
        }

        if (e && !e.hasError) {

            var message = "Writing data successfully finished...";
            document.getElementById("div_log_write").innerHTML = message;

        } else {

            if (e.error.getTypeString() == "TcAdsWebService.ResquestError") {
                // HANDLE TcAdsWebService.ResquestError HERE;
                document.getElementById("div_log_write").innerHTML = "Error: StatusText = " + e.error.statusText + " Status: " + e.error.status;
            }
            else if (e.error.getTypeString() == "TcAdsWebService.Error") {
                // HANDLE TcAdsWebService.Error HERE;
                document.getElementById("div_log_write").innerHTML = "Error: ErrorMessage = " + e.error.errorMessage + " ErrorCode: " + e.error.errorCode;
            }

        }
    });

    // Occurs if the write-read-write command runs into timeout;
    var WriteTimeoutCallback = (function () {
        // HANDLE TIMEOUT HERE;
        document.getElementById("div_log_write").innerHTML = "Write timeout!";
    });


    /*
    BtnWriteClickDown = (function (buttonName) {
        var switchTrueBase64Writer = new TcAdsWebService.DataWriter();
        switchTrueBase64Writer.writeBOOL(true);
        switchTrueBase64 = switchTrueBase64Writer.getBase64EncodedData();

        var _buttonName = buttonName;

        var handleNum = null;


        switch (_buttonName){
            case "Axis1Positive":
                handleNum = 16;
                break;
            case "Axis1Negative":
                handleNum = 17;
                break;
            case "Axis2Positive":
                handleNum = 18;
                break;
            case "Axis2Negative":
                handleNum = 19;
                break;
            case "Axis3Positive":
                handleNum = 20;
                break;
            case "Axis3Negative":
                handleNum = 21;
                break;
            case "Axis4Positive":
                handleNum = 22;
                break;
            case "Axis4Negative":
                handleNum = 23;
                break;
            case "Axis5Positive":
                handleNum = 24;
                break;
            case "Axis5Negative":
                handleNum = 25;
                break;
            case "Axis6Positive":
                handleNum = 26;
                break;
            case "Axis6Negative":
                handleNum = 27;
                break;
        }


        client.write(
            NETID,
            PORT,
            0x0000F005, // 0xF005 = Call Write by Handle Commando
            handlesWrite[handleNum], // IndexOffset = Count of requested variables.
            //1+4, // Length of requested data + 4 byte errorcode per variable.
            switchTrueBase64,
            WriteCallbackClickDown,
            null,
            general_timeout,
            WriteTimeoutCallbackClickDown,
            true);
    });

    var WriteCallbackClickDown = (function(e,s){

        if (e && e.isBusy) {
            // HANDLE PROGRESS TASKS HERE;
            var message = "Writing data to plc...";
            document.getElementById("div_log_write").innerHTML = message;
            // Exit callback function because request is still busy;
            return;
        }

        if (e && !e.hasError) {

            var message = "Writing data successfully finished...";
            document.getElementById("div_log_write").innerHTML = message;

        } else {

            if (e.error.getTypeString() == "TcAdsWebService.ResquestError") {
                // HANDLE TcAdsWebService.ResquestError HERE;
                document.getElementById("div_log_write").innerHTML = "Error: StatusText = " + e.error.statusText + " Status: " + e.error.status;
            }
            else if (e.error.getTypeString() == "TcAdsWebService.Error") {
                // HANDLE TcAdsWebService.Error HERE;
                document.getElementById("div_log_write").innerHTML = "Error: ErrorMessage = " + e.error.errorMessage + " ErrorCode: " + e.error.errorCode;
            }

        }
    });

    // Occurs if the write-read-write command runs into timeout;
    var WriteTimeoutCallbackClickDown = (function () {
        // HANDLE TIMEOUT HERE;
        document.getElementById("div_log_write").innerHTML = "Write timeout!";
    });





    
    BtnWriteClickUp = (function (buttonName) {
    //function BtnWriteClicked(buttonName) {
        var switchFalseBase64Writer = new TcAdsWebService.DataWriter();
        switchFalseBase64Writer.writeBOOL(false);
        switchFalseBase64 = switchFalseBase64Writer.getBase64EncodedData();

        var _buttonName = buttonName;

        var handleNum = null;


        switch (_buttonName){
            case "Axis1Positive":
                handleNum = 16;
                break;
            case "Axis1Negative":
                handleNum = 17;
                break;
            case "Axis2Positive":
                handleNum = 18;
                break;
            case "Axis2Negative":
                handleNum = 19;
                break;
            case "Axis3Positive":
                handleNum = 20;
                break;
            case "Axis3Negative":
                handleNum = 21;
                break;
            case "Axis4Positive":
                handleNum = 22;
                break;
            case "Axis4Negative":
                handleNum = 23;
                break;
            case "Axis5Positive":
                handleNum = 24;
                break;
            case "Axis5Negative":
                handleNum = 25;
                break;
            case "Axis6Positive":
                handleNum = 26;
                break;
            case "Axis6Negative":
                handleNum = 27;
                break;
        }

        
        client.write(
            NETID,
            PORT,
            0x0000F005, // 0xF005 = Call Write by Handle Commando
            handlesWrite[handleNum], // IndexOffset = Count of requested variables.
            //1+4, // Length of requested data + 4 byte errorcode per variable.
            switchFalseBase64,
            WriteCallbackClickUp,
            null,
            general_timeout,
            WriteTimeoutCallbackClickUp,
            true);
    });

    var WriteCallbackClickUp = (function(e,s){

        if (e && e.isBusy) {
            // HANDLE PROGRESS TASKS HERE;
            var message = "Writing data to plc...";
            document.getElementById("div_log_write").innerHTML = message;
            // Exit callback function because request is still busy;
            return;
        }

        if (e && !e.hasError) {

            var message = "Writing data successfully finished...";
            document.getElementById("div_log_write").innerHTML = message;

        } else {

            if (e.error.getTypeString() == "TcAdsWebService.ResquestError") {
                // HANDLE TcAdsWebService.ResquestError HERE;
                document.getElementById("div_log_write").innerHTML = "Error: StatusText = " + e.error.statusText + " Status: " + e.error.status;
            }
            else if (e.error.getTypeString() == "TcAdsWebService.Error") {
                // HANDLE TcAdsWebService.Error HERE;
                document.getElementById("div_log_write").innerHTML = "Error: ErrorMessage = " + e.error.errorMessage + " ErrorCode: " + e.error.errorCode;
            }

        }
    });

    // Occurs if the write-read-write command runs into timeout;
    var WriteTimeoutCallbackClickUp = (function () {
        // HANDLE TIMEOUT HERE;
        document.getElementById("div_log_write").innerHTML = "Write timeout!";
    });
    */
        
    
    /* ToDo:   
    window.onbeforeunload = (function () {
        // Free Handles
        client.write(NETID, PORT, 0xF006, hEngine, "", FreeHandleCallback, "hEngine", general_timeout, FreeHandleTimeoutCallback, true);
        client.write(NETID, PORT, 0xF006, hDeviceUp, "", FreeHandleCallback, "hDeviceUp", general_timeout, FreeHandleTimeoutCallback, true);
        client.write(NETID, PORT, 0xF006, hDeviceDown, "", FreeHandleCallback, "hDeviceDown", general_timeout, FreeHandleTimeoutCallback, true);
        client.write(NETID, PORT, 0xF006, hSteps, "", FreeHandleCallback, "hSteps", general_timeout, FreeHandleTimeoutCallback, true);
        client.write(NETID, PORT, 0xF006, hCount, "", FreeHandleCallback, "hCount", general_timeout, FreeHandleTimeoutCallback, true);
        client.write(NETID, PORT, 0xF006, hSwitch, "", FreeHandleCallback, "hSwitch", general_timeout, FreeHandleTimeoutCallback, true);

    });
    */

})();