import os
import copy
from time import sleep 
import tempfile
import random
import pickle
from multiprocessing import Pool

import pybullet as p
import pybullet_data

import numpy as np
import torch

from controller import Controller
from creature import gen_random_creature
from simulation import train_simulation

POP_SIZE = 60
TOP_K = 5
MUTATE = 15
NUM_GENERATIONS = 10000
NUM_STEPS = 10000

population = []

for i in range(POP_SIZE):
    creature = gen_random_creature()
    population.append(creature)
    
dt = 1 / 240

for gen in range(NUM_GENERATIONS):
    
    scores = []
    
    with Pool(6) as pool:
        scores = pool.map(train_simulation, population)
        
    sorted_pop = [i[0] for i in sorted(zip(population, scores), key=lambda x:x[1], reverse=True)]
    best_k = sorted_pop[:TOP_K]
    new_pop = best_k[:]

    for i in range(MUTATE):
        parent = random.choice(best_k)

        child = copy.deepcopy(parent)
        child.mutate()
        new_pop.append(child)

    for i in range(POP_SIZE - (TOP_K + MUTATE)):
        creature = gen_random_creature()
        new_pop.append(creature)
        
    population = new_pop
    print(gen, sorted(scores)[-1])
    
    best = sorted_pop[0]
    
    if gen % 5 == 0:
        with open(f'models/best_{gen}.pkl', 'wb') as f:
            pickle.dump(best, f, pickle.HIGHEST_PROTOCOL)
    

