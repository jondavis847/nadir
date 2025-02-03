![alt text](<Screenshot from 2025-01-28 00-07-00.png>)
# Intro
NASA Attitude Dynamics in Rust (NADIR) is a collection of rust libraries that support modeling and simulation of dynamical systems. The primary motivation is high fidelity modeling of spacecraft dynamics, but general dynamical systems are supported.

# DISCLAIMER
NADIR is under active development with many missing features, lots of bugs, and virtually no documentation. 

# Getting Started
The best way to start is to look at the examples. They can all be run from a terminal by changing directory in to the examples root folder and entering 'cargo run --release'. The first time you run you will see a bunch of dependencies get downloaded and compiled from cargo. If you don't include the --release it will still work, it will just compile faster and run slower.

Once the simulation is completed, you will see a message in the terminal and a results folder will appear in the directory that contains .csv files with the simulation output. 

To plot or animate results, install the NADIR command line interface. cd into the 'nadir' crate and enter 'cargo install --path .' which will install the nadir cli on your machine and add it to the path.

To plot, from the example's root directory enter 'nadir plot -r (sim name)' and follow the cli prompts

To animate the result in 3d, from the example's root directory enter 'nadir animate -r (sim name)'. Hit esc for animation options while in the window.