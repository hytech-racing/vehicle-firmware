#include "PDB_InterfaceTasks.hpp"
#include "PDB_SystemTasks.hpp"
#include <Arduino.h>
#include "SysClock_Config.h"
#include "TempSensorInterface.h"


void setup()
{
    initializeAllInterfaces();
    initializeAllSystems();

}

void loop()
{

}