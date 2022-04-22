import os
import copy
from time import sleep 
import tempfile
import random
import pickle
from multiprocessing import Pool
import argparse

import pybullet as p
import pybullet_data

import numpy as np
import torch

from controller import Controller
from creature import gen_random_creature
from simulation import train_following, train_walking

parser = argparse.ArgumentParser(description='evolve virtual creatures')
parser.add_argument('--task', type=str, help="walking or following", default='walking')
parser.add_argument('--workers', type=int, help="number of workers in multi-processing pool", default=6)
parser.add_argument('--pop_size', type=int, help="number of creatures in the population", default=60)
parser.add_argument('--top_k', type=int, help="number of creatures used for mutation", default=5)
parser.add_argument('--mutate', type=int, help="number of mutated creatures to generate", default=10)
parser.add_argument('--num_gens', type=int, help="number of generations to run", default=5000)
parser.add_argument('--out_dir', type=str, help="directory to save evolved creature files", default='models/')
args = parser.parse_args()

if args.task == 'following':
    train_func = train_following
else:
    train_func = train_walking

population = []

for i in range(args.pop_size):
    creature = gen_random_creature(args.task)
    population.append(creature)

if not os.path.exists(args.out_dir):
    os.mkdirs(args.out_dir)

for gen in range(args.num_gens):
    
    scores = []
    
    with Pool(args.workers) as pool:
        scores = pool.map(train_func, population)
    
    sorted_pop = [i[0] for i in sorted(zip(population, scores), key=lambda x:x[1], reverse=True)]
    best_k = sorted_pop[:args.top_k]
    new_pop = best_k[:]

    for i in range(args.mutate):
        parent = random.choice(best_k)

        child = copy.deepcopy(parent)
        child.mutate()
        new_pop.append(child)

    for i in range(args.pop_size - (args.top_k + args.mutate)):
        creature = gen_random_creature(args.task)
        new_pop.append(creature)
        
    population = new_pop
    print(gen, sorted(scores)[-1])
    
    best = sorted_pop[0]
    
    if gen % 5 == 0:
        with open(f'models/best_{gen}.pkl', 'wb') as f:
            pickle.dump(best, f, pickle.HIGHEST_PROTOCOL)
    

