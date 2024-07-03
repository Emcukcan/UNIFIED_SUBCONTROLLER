Revision Date: 7/3/2024
Firmware Version: 1.0.0



**UPDATE LOG**

- First Commitment

**************************************************************


1. Setup and Configuration:

Includes: Necessary libraries for hardware interactions, preferences storage, CAN bus communication, Bluetooth, and Modbus are included.
Pin Definitions: GPIO pins for controlling relays (charge, discharge, bypass, fan) and precharge are defined.
Preferences Loading: Settings like voltage/temperature alarms, maximum current, module size, and communication baud rates are loaded from persistent storage.
Task Creation: Several tasks are created to run concurrently:
INVERTERCANBUSTASK: Communicates with the inverter via CAN bus.
CANBUSTASK: Communicates with battery modules via CAN bus.
BREAKERCONTROLTASK: Controls relays based on battery conditions.
MODBUSTASK: Handles Modbus communication for external monitoring/control.
BT: Manages Bluetooth communication for setup and monitoring.
ANALOGTASK: Reads analog values (temperature, precharge voltage) using I2C.
SERIALMONITORTASK: Monitors serial input for commands and debugging.
2. Inverter Communication (INVERTERCANBUSTASK_CODE):

Sends battery SOC, voltage, and current data to the inverter periodically.
Implements a voltage-based current limiting strategy for the inverter to protect the batteries when the voltage is close to the maximum limit.
Sets group numbers and warning thresholds for the inverter.
The implementation differs slightly based on the inverter type (Deye or ATESS).
3. Battery Module Communication (CANBUSTASK_CODE):

Continuously polls battery modules for voltage, current, temperature, SOC, charge/discharge status, and alarms via CAN bus.
Calculates aggregate values for string 1 (voltage, current, SOC, temperature).
Determines the maximum and minimum cell voltages across all modules.
Implements a heartbeat mechanism to detect communication failures.
Relay Control (BREAKERCONTROLTASK_CODE):
Monitors battery voltage, temperature, and SOC to determine the state of charge/discharge relays, bypass relay, and fan.
Implements high/low voltage alarms, high-temperature alarm, and SOC lock functionality.
Includes a discharge takeover feature to temporarily enable discharge even under low voltage conditions.
Allows for manual (forced) control of relays via Bluetooth or serial commands.
5. Analog Reading (ANALOGTASK_CODE):

Reads analog values (temperature sensors, precharge voltage) using I2C communication.
Triggers precharge completion when the precharge voltage exceeds a threshold.
6. Bluetooth and Serial Monitoring:

The BT_CODE task handles Bluetooth communication, allowing for configuration changes, data retrieval, and relay control.
The SERIALMONITOR_CODE task monitors serial input for commands to display system status, change settings, or trigger actions.
Key Points and Observations:

Modular Design: The code is well-structured with separate tasks for different functionalities.
Error Handling: Includes basic error handling (e.g., I2C communication failure, invalid CAN bus data).
Safety Features: Implements voltage and temperature alarms, discharge takeover, and SOC lock for battery protection.
Flexibility: Allows for different inverter types and customizable settings.
Monitoring and Control: Provides multiple interfaces (Bluetooth, serial, Modbus) for monitoring and control.

