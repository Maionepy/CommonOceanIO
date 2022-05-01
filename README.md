![image info](./documentation/2020_WS_Practical/commonocean_logo.png)

This repository includes the python package for representing benchmarks for marine motion planning. In addition,
some hand-crafted scenarios, an adapted version of the commonroad drivability checker, and student documentations are provided.

The structure of the repository is:

```
.
├── documentation                   # Student documentation of commonocean-io (presentations, reports, thesis)
├── scenarios                       # Some hand-crafted example sceanrios for testing
├── commonroad-drivability-checker  # Some hand-crafted example sceanrios for testing
├── commonocean                     # Source files
│   ├── common                      # Folders which represent the package structure
│   ├── ...                         # ...
│   ├── doc                         # Files to generate a documentation for the commonocean-io package (not yet developed)
│   └── tests                       # Test files (not yet developed)
└── main.py                         # Main file             
```

## Installation instructions

Create a new Anaconda environment for Python 3.7 (here called co37). 

Run in your Terminal window:
```
$ conda create −n co37 python=3.7
```
Activate your environment
```
$ conda activate co37
```
Install all required packages through requirement.txt and if you want to use the jupyter notebook also install jupyter
```
$ pip install -r requirements.txt
$ pip install -e .
$ conda install jupyter
```
Additionally, the commonroad-drivability-checker library has to be built. Adapt the path to your conda environment and specify the python version (here env: co37 & version: 3.7)
```
$ cd commonroad-drivability-checker
$ sudo bash build.sh -e /path/to/your/anaconda3/envs/co37 -v 3.7 --cgal --serializer -i -j 4
$ cd ..
```
Now everything is installed and you can run the main.py file or start jupyter notebook to run the example notebook
```
$ jupyter notebook
```
