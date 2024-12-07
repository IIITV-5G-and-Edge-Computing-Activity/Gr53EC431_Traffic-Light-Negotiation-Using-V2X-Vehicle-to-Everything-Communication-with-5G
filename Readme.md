
---

# Traffic Light Negotiation Using V2X Communication with 5G

This project demonstrates a **traffic light negotiation system** leveraging Vehicle-to-Everything (V2X) communication enhanced by **5G technology**. The system models real-world intersections using **MATLAB** and **Simulink**, simulating efficient and safe navigation under dynamic traffic conditions. Key features include **traffic light detection**, **vehicle-to-vehicle (V2V)** and **vehicle-to-infrastructure (V2I)** communication, and ultra-low latency decision-making with 5G.

---
## Resources
- **Presentation Video Link**: https://drive.google.com/drive/folders/1vfPQtkY-pNp4QVyrBW-FE_Wn8fNpiEpN?usp=sharing
- **OverLeaf Editable Link**: https://www.overleaf.com/6636182164szjwvsfvzqpg#da3cbb
---

## Key Features

- **Traffic Light Negotiation**: Simulates interactions between vehicles and traffic lights for safe intersection navigation.
- **V2X Communication**: Implements V2V and V2I protocols for real-time data sharing between vehicles and infrastructure.
- **5G Technology**: Enhances communication reliability and reduces latency to sub-1 ms for critical traffic scenarios.
- **Dynamic Traffic Scenarios**: Handles complex intersections with multiple vehicles and dynamic signal changes.
- **Visualization**: Provides detailed views such as bird's eye, chase, and time-series graphs for traffic dynamics.

---

## Table of Contents

1. [Directory Structure](#directory-structure)
2. [Requirements](#requirements)
3. [Getting Started](#getting-started)
4. [Simulation Components](#simulation-components)
5. [Example Commands](#example-commands)
6. [Results Visualization](#results-visualization)
7. [Troubleshooting](#troubleshooting)
8. [Authors](#authors)
9. [References](#references)

---

## Requirements

### Software
- **MATLAB** (R2022b or later recommended)
- **Simulink**

### Required Toolboxes
- Automated Driving Toolbox
- Vehicle Network Toolbox
- Simulink 3D Animation
- 5G Toolbox

Ensure all required toolboxes are installed and accessible.

---

## Getting Started

Follow these steps to set up and run the project:

### 1. Clone the Repository
Download or clone the project:
```bash
git clone https://github.com/your-repo/TrafficLightNegotiationUsingV2XExample2.git
cd TrafficLightNegotiationUsingV2XExample2
```

### 2. Open MATLAB
Launch MATLAB and navigate to the project directory:
```matlab
cd('C:\Matlab\TrafficLightNegotiationUsingV2XExample2');
```

### 3. Open the Project
Load the Simulink project:
```matlab
openProject("TrafficLightNegotiationWithV2X");
```

### 4. Open Key Subsystems
Use the following commands to access major components:
```matlab
open_system("TrafficLightNegotiationWithV2XTestBench");
open_system("TrafficLightNegotiationWithV2XTestBench/Sensors and Environment");
open_system("TrafficLightNegotiationWithV2XTestBench/V2V Simulator");
open_system("TrafficLightNegotiationWithV2XTestBench/V2I Simulator");
open_system("DecisionLogic");
```

### 5. Run the Simulation
Execute the helper function to analyze results:
```matlab
helperTLNWithV2XResults(logsout);
```

---

## Simulation Components

### 1. **Sensors and Environment**
Models the traffic light, road network, and surrounding vehicles. It generates real-time inputs for the system’s decision logic.

### 2. **V2V Simulator**
Simulates Vehicle-to-Vehicle communication by broadcasting Basic Safety Messages (BSMs) containing vehicle states (speed, position, heading, etc.).

### 3. **V2I Simulator**
Implements Vehicle-to-Infrastructure communication by transmitting Signal Phase and Timing (SPAT) messages from traffic lights.

### 4. **Decision Logic**
Processes input data from the V2X subsystems and sensor models to make decisions for the ego vehicle (e.g., stop, go, or decelerate).

---

## Example Commands

Here are some useful MATLAB commands for working with this project:

### Open Key Models
```matlab
open_system("TrafficLightNegotiationWithV2XTestBench");
open_system("TrafficLightNegotiationWithV2XTestBench/Sensors and Environment");
open_system("TrafficLightNegotiationWithV2XTestBench/V2V Simulator");
open_system("TrafficLightNegotiationWithV2XTestBench/V2I Simulator");
open_system("DecisionLogic");
```

### Run the Helper Script
```matlab
helperTLNWithV2XResults(logsout);
```

---

## Results Visualization

The simulation provides detailed outputs across different views:

### 1. **Traffic Light State on Ego Lane**
Visualizes the state of the traffic light on the ego lane, ensuring the vehicle acts appropriately based on red, yellow, or green lights.

![Traffic Light State on Ego Lane](/images/01.jpg)

### 2. **Bird's Eye View of V2X Communication**
Shows the V2X communication range, interactions with other vehicles, and real-time updates of traffic light states.

![Bird's Eye View](/images/02.jpg)

### 3. **Chase View**
Displays the ego vehicle navigating the intersection from a rear perspective, including lane interactions and signal compliance.

![Chase View](/images/03.jpg)

### 4. **Enhanced Bird's Eye View**
Focuses on vehicle interactions with traffic lights in dense traffic scenarios, showcasing robust decision-making.

![Enhanced Bird's Eye View](/images/04.jpg)

### 5. **Time-Series Metrics**
Includes graphs for traffic light states, number of V2X detections, ego vehicle acceleration, and yaw angle over time.

![Time-Series Metrics](/images/05.jpg)

---

## Troubleshooting

### Common Issues

1. **File Not Found**:
   - Verify that the directory path is correct in MATLAB.
   - Use the `which` command to locate missing files:
     ```matlab
     which('TrafficLightNegotiationUsingV2XExample2')
     ```

2. **Undefined Function or Variable**:
   - Confirm that all required toolboxes are installed and accessible.

### Debugging Commands
```matlab
disp(logsout);
which helperTLNWithV2XResults;
```

---

## Authors

- **Banoth Vinayak**  
  Department of CSE, IIITV-ICD (Diu)  
  [202111018@diu.iiitvadodara.ac.in](mailto:202111018@diu.iiitvadodara.ac.in)

- **Chinni Jyothi Prakash**  
  Department of CSE, IIITV-ICD (Diu)  
  [202111020@diu.iiitvadodara.ac.in](mailto:202111020@diu.iiitvadodara.ac.in)

- **Maisagalla Venkatesh**  
  Department of CSE, IIITV-ICD (Diu)  
  [202111048@diu.iiitvadodara.ac.in](mailto:202111048@diu.iiitvadodara.ac.in)

- **Mogili Dinesh Reddy**  
  Department of CSE, IIITV-ICD (Diu)  
  [202111054@diu.iiitvadodara.ac.in](mailto:202111054@diu.iiitvadodara.ac.in)

- **Tettabavi Pranith Kumar**  
  Department of CSE, IIITV-ICD (Diu)  
  [202111082@diu.iiitvadodara.ac.in](mailto:202111082@diu.iiitvadodara.ac.in)

---

## References
1. On 5G-V2X Use Cases and Enabling Technologies: A Comprehensive Survey: https://ieeexplore.ieee.org/document/9497103 
2. 5G V2X communication at millimeter wave: rate maps and use cases: https://ieeexplore.ieee.org/abstract/document/9128612
3. Simulation of NR-V2X in a 5G Environment using OMNeT++: https://ieeexplore.ieee.org/document/10056746
4. Connecting vehicles to everything with C-V2X: https://www.qualcomm.com/research/5g/cellular-v2x 

---

This `README.md` is structured for clear guidance, from installation to troubleshooting, with detailed descriptions and visual aids for each feature and result. You can copy and use it directly in your project directory.