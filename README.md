# Robotics Lab Final Project: Path Planning with A* Algorithm

## Overview

### Project Description
This project demonstrates the use of the A* (A-star) path planning algorithm to find the optimal path for moving a car to a target position while avoiding obstacles. The algorithm calculates the most efficient route based on predefined criteria and displays the resulting path in a simulation.

### Objectives
- Implement the A* algorithm for path planning.
- Navigate a car to a target position while avoiding obstacles.
- Visualize the path planning process and the resulting path in a animation environment.

### Key Features
- Implementation of the A* algorithm for optimal pathfinding.
- Obstacle avoidance to ensure safe navigation.
- Simulation of the car's movement along the calculated path.

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/robotics-lab-final-project.git
    cd robotics-lab-final-project
    ```

2. Create a virtual environment and activate it:
    ```sh
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```

3. Install the required packages:
    ```sh
    pip install -r requirements.txt
    ```

## Usage

1. Configure the simulation environment by setting up the initial position, target position, and obstacles in the `config.json` file.

2. Run the path planning script:
    ```sh
    python path_planning.py
    ```

3. The simulation will display the car's movement along the calculated optimal path, avoiding obstacles.

## Contributing
Contributions are welcome! If you have any improvements or additions, please follow these steps:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes and commit them (`git commit -m 'Add some feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a Pull Request.
