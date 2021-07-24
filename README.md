# Erquy - A barebone high performance open-source rigid body simulator with minimal python bindings

I implemented erquy after I could not use [raisim](https://raisim.com/) on multiple computes easily.

It was built with two ideas in mind :

- **Provide a fast simulator for reinforcement learning.**

Erquy is built around [Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source fast and efficient kinematics and dynamics library. Erquy thus uses minimal coordinates and Lagrangian dynamics to simulate an articulated system: this makes Erquy as close as numerically possible to an analytical solution, without the risk of joint violation.

- **Build a minimal viable product first. Complexify later.**

Erquy is at its core only five files, with less than 700 lines of code total. It was build with minimal features to make the code easy to understand and extend. 

## Key features

### General

- Simulation of multi-body systems using minimal coordinates and Lagrangian dynamics.
- Designed with machine learning in mind, with easy wrapping of robots as [OpenAI Gym](https://github.com/openai/gym) environments.
- Available for both Linux and Windows platforms.

### Physics

- Provide impulse-level LCP based on maximum energy dissipation principle.
- Support contact and collision between every two paires of collision geometry.
- Able to simulate multiple articulated systems simultaneously, interacting with each other, to support use cases such as multi-agent reinforcement learning or swarm robotics.
- Support of compliant joints with force-based stable spring-damper dynamics with semi-implicit Euler integration, to model joint elasticity, a common phenomenon particularly in legged robotics.



## Post-it

export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export PATH=/opt/openrobots/bin:$PATH
export PATH=/opt/openrobots/sbin:$PATH
