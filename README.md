## Project Overview

CORE 2024 is a project focused on designing a system of multi-robot assembly of rigidity-preserving structures, all while eliminating the use of scaffolding. This is done through the a multi-robot setup where robots act both as workers and scaffolding ensuring rigidity along the (dis)assembly process. Topological / geometry contstraints are necessary to ensure that each step of the assembly is locally and globally rigid. Our project starts by reconfiguring an existing design with those constraints in mind, then through an assembly sequence algorithm which creates the order of steps of assembly ensuring rigidity is working in cooperation with the structural reconfiguration and optimisation setup for evalutation, and the same ordered elements are used in combination with the multi-robot framework to generate the assembly programs for the robots.

**Read the full documentation of the project:**
[Team07ProjectPortfolio.pdf]((https://github.com/Antonios-M/CORE_2024/blob/main/Team07ProjectPortfolio.pdf))


---

## Repository Structure

The repository is organized as follows:

- `src/`: Contains the source code for the project  
  - `ansys/`: Contains simulation files and data on the topological optimisation and detailed analysis of node geometry
- `examples/`: Example files for assembly sequence and robot prototype
- `data/`: Datasets of cutting patterns and structural design
- `gh/`: Grasshopper files for robot prototype and structural design / assembly sequence

This repository provides access to the three main components of the project:

1) **The Assembly Sequence Algorithm**

The core structure of this computational framework is integrated into all parts of the project.

**Required Files:**
  • `assembly_sequence_and_cutting_stock.gh`

2) **The Structure Design & Optimisation Algorithm**

**Required Files:**
  • `241105_Combined model.gh`  
  • `241105_Combined model.3dm`  
  • `on_site_assembly.xlsx`

3) **The Multi-Robot Assembly Setup Algorithm**

**Required Files:**
  • `RoboticAssemblyPrototype.3dm`  
  • `RoboticAssemblyPrototype.rdk`  
  • `RoboticAssemblyPrototype.gh`

**Optional External Interface:**
  • `RobotManager.py`

---

### Prerequisites

- **Rhinoceros 8**  
  Download from [https://www.rhino3d.com/](https://www.rhino3d.com/)

- **RoboDk**  
  Download from [https://robodk.com/download](https://robodk.com/download)

- **Rhinoceros & RoboDk Files**  
  Download from [Google Drive](https://drive.google.com/drive/folders/1hj0ywdX9TM16v8JOCXHmmRV7WWfTj7M6?usp=drive_link)

- **GH_Linear_Cutting**  
  Download .gha file from [https://github.com/AlexanderMorozovDesign/GH_Linear_Cutting](https://github.com/AlexanderMorozovDesign/GH_Linear_Cutting)

- **Karamba**  
  Download from [https://karamba3d.com/](https://karamba3d.com/)

- **Hops**  
  Download from [https://www.food4rhino.com/en/app/hops](https://www.food4rhino.com/en/app/hops)

- **Telepathy**  
  Download from [https://www.food4rhino.com/en/app/telepathy](https://www.food4rhino.com/en/app/telepathy)

- **Human**  
  Download from [https://www.food4rhino.com/en/app/human](https://www.food4rhino.com/en/app/human)

- **Fox**  
  Download from [https://www.food4rhino.com/en/app/fox](https://www.food4rhino.com/en/app/fox)

- **Pufferfish**  
  Download from [https://www.food4rhino.com/en/app/pufferfish](https://www.food4rhino.com/en/app/pufferfish)

---

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Antonios-M/CORE_2024.git

2. Install Pre-requisites

### Contact

For questions or support, please contact:
**Email** : smaniatis@tudelft.nl
