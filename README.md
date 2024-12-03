# AutoHomePlan

This is my graduation project for USTC2024. 

## Getting Started

AutoHomePlan utilizes CMake for project file generation, vcpkg for managing external dependencies.

### 1. Install Gurobi
Please ensure that you have installed Gurobi and set the system environment variable GUROBI_HOME. Visit website [Gurobi](https://www.gurobi.com/) for more detailed information.


Once GUROBI is successfully installed and the environment variables are correctly set, the FindGUROBI.cmake file will automatically link and configure gurobi.

### 2. Clone

```
git clone https://github.com/iambrc/AutoHomePlan.git
git submodule update --init --recursive
```
### 3. Build

First, use vcpkg to install the dependencies, and then complete the configuration using cmake.
```
vcpkg install
```

### 4.Visualization of scene graphs

Here, we use [Graphviz](https://graphviz.org/) to visualize the scene graph.

You can save the scene graph as a .dot file and run the following code to obtain the scene graph:

```
dot -Tpng input.dot -o output.png
```


## Assets
3D assets are from [BlenderKit](https://www.blenderkit.com/).

## Other Information
TODO

## Acknowledgement
TODO
