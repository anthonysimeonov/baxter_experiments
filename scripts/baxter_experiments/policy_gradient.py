# set GPU to use
import os
os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"   
os.environ["CUDA_VISIBLE_DEVICES"]="1"

# useful libraries
from tqdm import tqdm #gives progress bars
%matplotlib inline
import matplotlib.pyplot as plt
import numpy as np
from collections import namedtuple
from copy import deepcopy
import random
import time
from collections import namedtuple
import numpy as np

# pytorch
import torch
from torch.autograd import Variable
import torch.nn.utils as utils
import torch.nn as nn
import torch
from torch import nn, optim
from torch.autograd import Variable
import torch.nn as nn
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
from torch.distributions import Normal
from torch.distributions import Categorical

#####################################################
#                Helper functions                   #
#####################################################

#----------------------------------------------------
#exponential moving average
#----------------------------------------------------
def numpy_ewma_vectorized_v2(data, window):

    alpha = 2 /(window + 1.0)
    alpha_rev = 1-alpha
    n = data.shape[0]

    pows = alpha_rev**(np.arange(n+1))

    scale_arr = 1/pows[:-1]
    offset = data[0]*pows[1:]
    pw0 = alpha*alpha_rev**(n-1)

    mult = data*pw0*scale_arr
    cumsums = mult.cumsum()
    out = offset + cumsums*scale_arr[::-1]
    return out

#----------------------------------------------------
#timing function
#----------------------------------------------------
class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print('[%s]' % self.name,)
        print('Elapsed: %s' % (time.time() - self.tstart))

#----------------------------------------------------
#flip a pytorch vector
#----------------------------------------------------
def flip(x, dim):
    dim = x.dim() + dim if dim < 0 else dim
    inds = tuple(slice(None, None) if i != dim
             else x.new(torch.arange(x.size(i)-1, -1, -1).tolist()).long()
             for i in range(x.dim()))
    return x[inds]

#----------------------------------------------------
#normalize actions
#----------------------------------------------------
#https://github.com/openai/gym/blob/78c416ef7bc829ce55b404b6604641ba0cf47d10/gym/core.py

class NormalizeAction(gym.ActionWrapper):
    def action(self, action):
        #tanh outputs (-1,1) from tanh, need to be [action_space.low,environment.high]
        return (action + 1) / 2 * (self.action_space.high - self.action_space.low) + self.action_space.low
        
    def reverse_action(self, action):
        #reverse of that above
        return (action - self.action_space.low) / (self.action_space.high - self.action_space.low) * 2 - 1

class PolicyGradientLearner(object):
