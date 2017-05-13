# GridBallastDeviceController

This repository includes the source code and documentation for the GridBallast Device Simulator developed under Award Number DE-AR0000705 for the Advanced Research Projects Agency - Energy (ARPA-E) NODES program. 

The simulators can be used to reproduce the power demand of two types of electrical loads: thermostatically controlled loads (TCLs) and deferrable plug-loads. For the TCLs, the simulator implements difference equations of the thermal dynamics assuming either a single or a stratified thermal mass. Two specific end-uses have been considered to specialize the simulation of TCLs: electric water heaters (EWHs) and refrigerators. Deferrable plug-loads, on the other hand, are modeled as a fixed or parametrized power demand pattern that can be interrupted at any time as long as the pattern is completed before a user-supplied deadline. Both types of loads can be controlled with a GridBallast controller that is also available in the simulator, and which directly controls the operational status (*on/off*) of the loads based on either: local grid frequency measurements, internal temperature or a ramping reserve participation request.

A Jupyter Notebook containing example usage of the simulator as well as the theory behind the TCL and deferrable plug-load models, is included in the root directory (*getting-started.ipynb*). 
