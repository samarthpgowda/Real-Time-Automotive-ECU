# Real-Time-Automotive-ECU
The objective was to build a software-based automotive ECU that simulates multiple vehicle functions while demonstrating how real-time tasks communicate, synchronize, and respond to critical events.

The system consists of multiple FreeRTOS tasks:

EngineTask — Simulates RPM, engine temperature, and engine state.
SpeedTask — Simulates vehicle speed.
FuelTask — Simulates fuel consumption and generates low-fuel conditions.
DashboardTask — Collects vehicle data and displays it through UART.
DiagnosticTask — Monitors ECU events and reports faults.
EmergencyTask — Handles emergency button events and transitions the ECU into a safe state.
