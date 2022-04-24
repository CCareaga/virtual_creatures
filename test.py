import os
from time import sleep, time
import tempfile
import random
import pickle
import argparse

import pybullet as p
import pybullet_data

import numpy as np
import torch

from simulation import test_walking, test_following
from creature import gen_random_creature

parser = argparse.ArgumentParser(description='test a generated creature')
parser.add_argument('--creature', type=str, help='path of stored creature pickle file', default='')
parser.add_argument('--task', type=str, help="walking or following", default='walking')
parser.add_argument('--record', action='store_true', help="wether or not to record the simulation")
parser.add_argument('--out_path', type=str, help="name of .mp4 file for recording output", default='')
args = parser.parse_args()

if args.creature != '':
    with open(args.creature, 'rb') as f:
        creature = pickle.load(f)
else:
    creature = gen_random_creature(args.task)

if args.task == 'following':
    test_following(creature, record=args.record, out_file=args.out_path)
else:
    test_walking(creature, record=args.record, out_file=args.out_path)
