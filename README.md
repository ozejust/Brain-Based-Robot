# Brain_Based_Robot

# NAO in V-REP
This goal of this project is to build a Brain-Based Robot (BBR) using hybrid techniques that incorporate both Brain-Based Device(BBD) and computational algorithms.
BBDs are biologically inspired machines which have its behavior guided by a simulated nervous system.

In this project a Nao robot moves in an environment with blocks with different colors, to learn to associate some colors to good taste, and other colors with bad taste.
After such relations is built between different sensory inputs, motor actions are associated also with such relation to approach(or avoid) coloured blocks.

A big part of this was built upon [pool-martin]'s and [PierreJac]'s work. Please read there work as a start point for this.
In addition to v-rep we will use the Choregraphe suite and the Python NAOqi SDK from Aldebaran for physical simulation.
Nengo library is used for neural simulation.

This work was build on an ubuntu 14.04 machine.

### Requirements
- [v-rep] : A free simulation environment.
- [Python NAOqi-SDK] : Contain all the function you need to manipulate your NAO (virtual or not) using python.
- [Nengo] : a python package for a neural simulation environment developed at the centre for theoretical neuroscience at the university of Waterloo.
- [OpenCV]

# Recommended
-[Anaconda package]

### Quickstart
- Replace /script/remoteApi.dll(and .so) with one suitale for your OS.
- After all prerequisites installed:
***Edit the start_linux.sh by setting the approporiate paths before you run***
$ ./EnvironmentSetup/start_linux.sh
Then run /script/bbr_main.py
