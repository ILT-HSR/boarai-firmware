boarAI Scenario Logic Analysis
==============================

This analysis should extract more information about the logic and processes of the subsystems on the boarAI.
Mainly affected are subsystems that can be configured and/or running software to define the system's behavior.

# System States

>**poff state:** The "power off state" is when the main power switch is off and/or the system has no energy supply.

>**safe state:** In safe state the system is powered but cannot execute any actions.

>**init state:** Is used to run the initialization process for the whole system.

>**misn state:** The system is in "mission state" during the execution of an mission. The system acts fully autonomous.

>**esdn state:** The "emergency shutdown state" is entered by operating the emergency switch.

>**ddev state:** In development state the system can execute all functions by overriding instructions and parameters.

# deployment

## power up

The system must be in a safe state while booting all subsystems.
The safe state is defined as:
- no motion of all system components (vibrations are accepted)
- visual indication of the safe state
- no sound

For the motor controller this means having the FETs off until the main controller gives other instructions.
The main controller must boot into a state with no motion actions at all.

## initialisation

The initialisation process of all subsystems is initiated by the main controller.
It can only be entered from safe state.
It is triggered when the system is in safe mode without beeing initialized.
This assures that the order of initialization is controlled.
The state of initialisation is indicated visually and audibly.

1. init connection to remote control (handheld)
1. init connections to sensors
 1. init gps positioning
 1. init hokuyo lasers
1. init connection to motor controller
1. init motor controller

During initialisation process the system is allowed to move slowly within a time constraint of **tbd** for each movement.
After initialisation the system state changes automatically back into safe state.
The process can be restarted if failed under certain conditions.

## idling

After successful initialization the system is idling in safe state.
While idling the system reacts on instructions from the remote.
Independent of the current state the system periodically sends it's state of operation to the remote, known as hearbeat.

# missions

A mission can only run in misn state. 
To enter misn state the system must be fully initialized and idle in safe state.
If a mission terminates successfully the system changes back to safe state.
If a mission is aborted remotely or caused by a failed mission task the system returns into safe state.
If a system error occurs the current mission aborts and the system tries to reinitialize.

## mission execution

The sequence is as follows if all steps are successful.

1. Transfer a mission descriptor to the system or activate an onboard mission.
1. Issue system into misn state.
1. Parse active mission and analyze against errors.
1. Run instructions step by step.
1. Process and Send result for flaged instructions.
1. Return into safe state.

# shutdown

There are two states that can be entered as a shutdown of the system.

- The poff state can be entered by manually operate the main power switch.
- The esdn state is entered when manually operated the emergency switch.

The key difference is that during esdn the main controller is running and the system can recover without loosing current process data.

#debug and development

//todo: describe ddev state
