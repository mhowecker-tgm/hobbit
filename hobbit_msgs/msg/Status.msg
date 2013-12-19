#Status message, to be provided by the modules to inform other modules or MMUI their status/progress

Header  header

string  moduleName  # name of ros-module
string  sessionID   # sessionID, which triggered this module

string  status      # current status in words ( english ), e.g. "scanning table in kitchen", "waiting for user-input"
float32 progress    # 0.0 - 1.0, a good guess by the module on its progress
Parameter[] params  # optional parameters

