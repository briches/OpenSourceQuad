// Voltage monitor test
#include "OSQ_BatteryMonitor.h"


void setup()
{
        Serial.begin(115200);
        Serial.println("-------------------------BatteryMonitorTest-------------------------");
}

void loop()
{
        monitorBatteryVoltage();
        Serial.print("Voltage: ");
        Serial.print(batteryStats.batteryVoltage);
        Serial.print(" Soft alarm: ");
        Serial.print(softAlarm);
        Serial.print(" Critical alarm: ");
        Serial.print(criticalAlarm);
        Serial.print(" Alarm count: ");
        Serial.println(batteryStats.alarmCount);
        delay(1000);
}

