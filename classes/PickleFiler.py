import pickle
import numpy as np

# Saving data to a pickle file
def save_to_pickle(data, filename):
    with open(filename, 'wb') as file:
        pickle.dump(data, file)

# Loading data from a pickle file
def load_from_pickle(filename):
    with open(filename, 'rb') as file:
        return pickle.load(file)