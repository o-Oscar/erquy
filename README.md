# Erquy - A barebone high performance open-source rigid body simulator with minimal python bindings

I implemented erquy after I could not use [raisim](https://raisim.com/) on multiple computes easily.

It was built with two ideas in mind :

- **Provide a fast simulator for reinforcement learning.**

Erquy is built around [Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source fast and efficient kinematics and dynamics library. Erquy thus uses minimal coordinates and Lagrangian dynamics to simulate an articulated system: this makes Erquy as close as numerically possible to an analytical solution, without the risk of joint violation.

- **Build a minimal viable product first. Complexify later.**

Erquy is at its core only five files, with less than 700 lines of code total. It was build with minimal features to make the code easy to understand and extend. 

## Key features

### General

- Designed with machine learning in mind, with easy wrapping of robots as [OpenAI Gym](https://github.com/openai/gym) environments.
- Available for both Linux and Windows platforms.

### Physics

- Simulation of multi-body systems using minimal coordinates for fast simulation and Lagrangian dynamics for accurate constrain resolution.
- Accurate resolution of solid friction contrains with the [bisection method](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/335381/contact-iteration-method%20%284%29.pdf?sequence=1).
- Support contact and collision between every two paires of collision geometry.
- Able to simulate multiple articulated systems simultaneously, interacting with each other, to support use cases such as multi-agent reinforcement learning or swarm robotics.
- Support of compliant joints with force-based stable spring-damper dynamics with semi-implicit Euler integration, to model joint elasticity, a common phenomenon particularly in legged robotics.



## Post-it

In order to build and run this project, you need [Pinocchio](https://github.com/stack-of-tasks/pinocchio). For me, some path variables were missing here are what as needed for me to complete the build.

- export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
- export PATH=/opt/openrobots/bin:$PATH
- export PATH=/opt/openrobots/sbin:$PATH
