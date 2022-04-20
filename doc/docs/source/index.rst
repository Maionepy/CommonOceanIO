CommonOcean Input-Output
=====

This tool includes the python package for representing benchmarks for marine motion planning. In addition,
some hand-crafted scenarios, an adapted version of the commonroad drivability checker, and student documentations are provided.
The structure of the repository is:

|   .
|   ├── documentation                   # Student documentation of commonocean-io (presentations, reports, thesis)
|   ├── scenarios                       # Some hand-crafted example sceanrios for testing
|   ├── commonroad-drivability-checker  # Some hand-crafted example sceanrios for testing
|   ├── commonocean                     # Source files
|   │       ├── common                      # Folders which represent the package structure
|   │       ├── ...                         # ...
|   │       ├── doc                         # Files to generate a documentation for the commonocean-io package (not yet developed)
|   │       └── tests                       # Test files (not yet developed)
|   └── main.py                         # Main file       



.. _documentation:

Documentation
------------

The full documentation of the API and introducing examples can be found under `commonocean.cps.in.tum.de <https://commonocean.cps.in.tum.de>`__.

For getting started, we recommend our `tutorials <https://commonocean.cps.in.tum.de/commonocean-io>`__.

.. _requirements:

Requirements
============

The required dependencies for running CommonRoad_io are:

* numpy>=1.13
* shapely>=1.6.4
* matplotlib>=2.2.2
* lxml>=4.2.2
* networkx>=2.2
* Pillow>=7.0.0

.. _installation:

Installation
------------

Create a new Anaconda environment for Python 3.7 (here called co37).
Run in your Terminal window:

.. code-block:: console

   $ conda create −n co37 python=3.7

Activate your environment

.. code-block:: console

   $ conda activate co37
   
Install all required packages through requirement.txt and if you want to use the jupyter notebook also install jupyter

.. code-block:: console

   $ pip install -r requirements.txt
   $ pip install -e .
   $ conda install jupyter

Additionally, the commonroad-drivability-checker library has to be built. Adapt the path to your conda environment and specify the python version (here env: co37 & version: 3.7)

.. code-block:: console

   $ cd commonroad-drivability-checker
   $ sudo bash build.sh -e /path/to/your/anaconda3/envs/co37 -v 3.7 --cgal --serializer -i -j 4
   $ cd ..

Now everything is installed and you can run the main.py file or start jupyter notebook to run the example notebook

.. code-block:: console

   $ jupyter notebook


.. _contactinformation:

Contact information
===================

:Website: `https://commonocean.cps.in.tum.de <https://commonocean.cps.in.tum.de>`_
:Email: `commonocean@lists.lrz.de <commonocean@lists.lrz.de>`_
