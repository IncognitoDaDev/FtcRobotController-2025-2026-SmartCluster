package com.smartcluster.oracleftc.hardware;

import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;

public interface OracleLynxCommandListener {
    void onCommand(LynxCommand<?> command);
    void onCommandResponse(LynxMessage response, LynxCommand<?> respondedCommand);
}

