# ENVIRONMENTAL CONTROL AND LIFE SUPPORT SYSTEMS

# 1.Air Revitalization System (ARS) Module for ISS ECLSS

## Overview
The **Air Revitalization System (ARS)** module is a critical subsystem of the **Environmental Control and Life Support System (ECLSS)** onboard the International Space Station (ISS). Its primary function is to regulate atmospheric conditions by:

- Removing excess **carbon dioxide (CO2)** from the cabin air.
- Maintaining optimal **temperature** and **humidity** levels.

This repository simulates the functionality of the ARS module, focusing on the **4-bed molecular sieve CO2 scrubber**. The simulation is implemented using ROS 2, with nodes representing various ARS subsystems and processes.

---

## Real-world Inspiration: 4-Bed CO2 Scrubber
The **4-bed molecular sieve CO2 scrubber** on the ISS operates as follows:

1. **Bed 1** removes water vapor from the incoming cabin air.
2. **Bed 2** adsorbs CO2 using zeolite crystals.
3. While Beds 1 and 2 operate, Beds 3 and 4 regenerate:
    - **Bed 3** releases captured water vapor back into the cabin air.
    - **Bed 4** desorbs CO2 into space via vacuum exposure.
4. The system periodically switches roles between the beds to ensure continuous operation.

Key advantages include:
- **Closed-loop operation**: No consumable chemicals are required.
- **Water conservation**: Captured water vapor is returned to the cabin.

---

## Simulation Design
The ARS simulation consists of two main ROS 2 nodes:

1. **ARS System Node (`ars_system`)**:
   - Monitors atmospheric conditions (CO2, temperature, humidity).
   - Detects critical CO2 levels and triggers the baking process.
   - Publishes real-time environmental data.

2. **Baking Process Node (`baking_process`)**:
   - Simulates the CO2 scrubbing process using zeolite beds.
   - Handles requests from the ARS system node to reduce CO2 levels.

---

## System Workflow

### 1. **Monitoring and Publishing Environmental Data**
The ARS system node continuously:
- Simulates the increase in CO2 levels due to crew metabolism.
- Simulates temperature and humidity fluctuations.
- Publishes the environmental data to the topic `/ars_system`.

### 2. **Critical CO2 Threshold Detection**
When the CO2 level exceeds a critical threshold:
- The ARS system node logs a warning.
- Sends a request to the baking process node to initiate CO2 scrubbing.

### 3. **CO2 Scrubbing Process**
The baking process node:
- Simulates the adsorption of CO2 using zeolite beds.
- Returns the reduced CO2 level and process status to the ARS system node.

### 4. **CO2 Level Update and Publishing**
- The ARS system node updates its CO2 level based on the response from the baking process node.
- Resumes publishing the updated environmental data.

---

## ROS 2 Topics

### Published Topics
1. **`/ars_system`**:
   - **Type**: `space_station_eclss/msg/ARS`
   - **Description**: Publishes real-time environmental data, including:
     - Current CO2 level (ppm).
     - Temperature (°C) with variance.
     - Humidity (%) with variance.

---

## ROS 2 Services

### Provided Services
1. **`/check_efficiency`** (Baking Process Node):
   - **Type**: `std_srvs/srv/Trigger`
   - **Description**: Handles CO2 reduction requests.
     - **Request**: Empty (Trigger service).
     - **Response**:
       - `success`: Boolean indicating if the baking was successful.
       - `message`: Status message describing the result.

---

## Code Implementation

### ARS System Node (`ars_system`)
The **ARS system node** simulates and publishes environmental conditions:

1. **Initialization**:
   - Parameters (e.g., `initial_co2_level`, `critical_threshold`) are loaded from a YAML file.

2. **Simulation**:
   - CO2 levels increase over time.
   - Temperature and humidity fluctuate randomly within a defined range.

3. **Critical CO2 Detection**:
   - When CO2 exceeds the threshold, a request is sent to the baking process node.

4. **Publishing**:
   - Publishes updated environmental data on `/ars_system`.

### Baking Process Node (`baking_process`)
The **baking process node** simulates the CO2 scrubbing process:

1. **CO2 Reduction**:
   - Simulates an 80% chance of successfully reducing CO2 by 30%.

2. **Response Handling**:
   - Returns the updated CO2 level and success status to the ARS system node.

---

## Parameters

### YAML Configuration
Parameters are configured via a YAML file (`config/ars_params.yaml`):

```yaml
ars_system:
  initial_co2_level: 400.0
  increase_rate: 5.0
  critical_threshold: 800.0
  bake_reduction: 300.0
  scrubber_efficiency: 0.9
  station_temperature: 22.0
  station_humidity: 50.0
```

---

## Running the Simulation

### Step 1: Build the Package
```bash
colcon build
```

### Step 2: Source the Workspace
```bash
source install/setup.bash
```

### Step 3: Launch the Nodes
```bash
ros2 launch space_station_eclss ars_system.launch.py
```

---

## Expected Logs

### ARS System Node
```plaintext
[INFO] [ars_system]: AIR REVITALIZATION SYSTEM INITIALIZED
[WARN] [ars_system]: Critical CO2 level reached: 800.00 ppm. Immediate action required!
[INFO] [ars_system]: Sending request to bake CO2.
[INFO] [ars_system]: Bake process successful. Initial CO2 level: 800.00 ppm. Reduced to: 560.00 ppm.
```

### Baking Process Node
```plaintext
[INFO] [baking_process]: Bake request received. Initial CO2 level: 800.00 ppm.
[INFO] [baking_process]: Bake request handled successfully. CO2 reduced from 800.00 ppm to 560.00 ppm.
```

---

## Future Enhancements and TODO
1. **Extended Parameterization**:
   - Include dynamic thresholds based on mission conditions.
2. **Fault Tolerance**:
   - Simulate sensor failures and degraded performance.
3. **Integration with Other ECLSS Subsystems**:
   - Combine with oxygen generation and water recovery systems.

---

## References
- **NASA-STD-3001**: Technical standards for environmental control systems.
- **ISS ECLSS Documentation**: 4-bed molecular sieve CO2 scrubber process.

---

Thank you for reviewing this contribution! Let us know if you have any feedback or questions.

