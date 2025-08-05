# Intro
NADIR is a collection of Rust crates that support modeling and simulation of dynamical systems. The primary motivation is high fidelity modeling of spacecraft dynamics, but general dynamical systems are supported.

# DISCLAIMER
NADIR is under active development with many missing features, lots of bugs, and virtually no documentation. 

# Getting Started
NADIR is written in the Rust programming language, which can downloaded and installed from here https://www.rust-lang.org/tools/install. Rust comes with a build system/package manager called cargo that is used to manage  crates (packages) and run rust code. You can use Rust in pretty much any code editor, but whichever you choose it is basically mandatory that you also install the rust-analyzer extension for that editor.

The best way to start using NADIR is to look at the examples. In the crate 'examples' are some spacecraft specific demos. There are examples for the differential equation crate 'nadir_diffeq' that run monte carlo and do basically general ode45 like solving. There are also examples in the 'multibody' crate for doing general multibody dynamics. They can all be run from a terminal by changing directory from the command line in to the examples root folder and entering 'cargo run --release --example (name)'. For the spacecraft examples, those are standalone programs, so you will change into the folder and just run 'cargo run --release' The first time you run you will see a bunch of dependencies get downloaded and compiled from cargo. If you don't include the --release it will still work, it will just run slower.

There is an interactive Read-Evaluate-Print-Loop (REPL) program in the 'nadir' crate. It includes some basic types for interacting with NADIR's crates, as well as some utilities for animating and plotting data. To install, change directory to the 'nadir' crate directory and enter 'cargo install --path .'. You will need admin privileges. To access the REPL, open a command line and just type 'nadir' after installing. 

After running the spacecraft examples, you will have .csv files in a results folder in the directory that you ran from. To use the nadir REPL to plot the results, you can use the Figure::from_file() command to navigate and select the csv file and data to plot. To animate the results in the spacecraft or multibody examples, enter the nadir REPL, cd to the result folder, and just type 'animate()'.