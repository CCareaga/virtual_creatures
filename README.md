#### Virtual Creatures Re-Implementation

![](https://github.com/CCareaga/cmpt766_final_project/blob/main/images/banner.png?raw=true)

#### Installing

This repo depends on a few popular packages:
    
- PyBullet (for physics simulation)
- PyTorch (for creature controller)
- NumPy (dealing w/ positions, math, etc.)
- odio_urdf (for creating and manipulating URDF XML)

#### Running

This repo contains two main scripts, one for evolving virtual creatures and one for testing evolved creatures. To train the creatures run:

```
$ python train.py --help

usage: train.py [-h] [--task TASK] [--workers WORKERS] [--pop_size POP_SIZE] [--top_k TOP_K] [--mutate MUTATE] [--num_gens NUM_GENS] [--out_dir OUT_DIR]

evolve virtual creatures

optional arguments:
  -h, --help           show this help message and exit
  --task TASK          walking or following
  --workers WORKERS    number of workers in multi-processing pool
  --pop_size POP_SIZE  number of creatures in the population
  --top_k TOP_K        number of creatures used for mutation
  --mutate MUTATE      number of mutated creatures to generate
  --num_gens NUM_GENS  number of generations to run
  --out_dir OUT_DIR    directory to save evolved creature files
```

To run the walking task with default settings simply run `python train.py`. After running this script the best creature will be logged (by default to the models/ directory). The logged creatures are python objects stored as pickle files. These files can then be tested using the test script:

```
$ python test.py -h

usage: test.py [-h] [--creature CREATURE] [--task TASK] [--record] [--out_path OUT_PATH]

test a generated creature

optional arguments:
  -h, --help           show this help message and exit
  --creature CREATURE  path of stored creature pickle file
  --task TASK          walking or following
  --record             wether or not to record the simulation
  --out_path OUT_PATH  name of .mp4 file for recording output
```

The evolved walking creatures can be ran using the following command:

```
$ python test.py --creature models/best_100.pkl
```

Some examples of evolved creatures are included in the `examples/` directory

