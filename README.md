# PSoC_Localization
This project implements a sonar-based localization scheme for improving UAV positioning during takeoff and landing.

## Beacon
Drives speakers with a 5V square wave at 40kHz, maximum 40mA

## Receiver
Sends a synchronization signal to initiate sonar pulse transmission by the beacon and determines the time required for the pulse to be detected by each of the 3 receivers. 
The beacon position is estimated using trilateration.

## TARA UAV Project
tara.uav@gmail.com
