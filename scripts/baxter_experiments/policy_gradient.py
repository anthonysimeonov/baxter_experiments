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


#####################################################
#                Class Definition                   #
#####################################################


class PolicyGradientLearner(object):
    class network(nn.module):
        def __init__(self,
            input_size,
            output_size,
            n_layers=1,
            size=64,
            activation='relu',
            output_activation='softmax'):

            super(network, self).__init__()
            self.output_activation = output_activation
            #populate architecture list
            self.architecture = []
            #self.architecture.append(input_size)
            for i in range(n_layers):
                self.architecture.append(size)
                #if i%2 == 0:
                self.architecture.append(activation)
            self.architecture.append(output_size)
            #self.architecture.append(output_activation)
            
            print('Network architecture is : {}'.format(self.architecture))
            
            #construct network
            input_channels = input_size #initialize input_channels
            self.layers = []
            for layer in self.architecture:
                output_channels = layer
                if layer == 'tanh':
                    self.layers.append(nn.Tanh())  #NOT INPLACE? MIGHT CAUSE ISSUE OR BE OK?
                elif layer == 'relu':
                    self.layers.append(nn.ReLU(inplace=True))
                elif layer == None:
                    pass
                else:
                    self.layers.append(nn.Linear(input_channels, output_channels))
                    input_channels = output_channels
            self.fc1 = nn.Sequential(*self.layers)
            if(self.output_activation == 'softmax'):
                self.softmax = nn.Softmax(dim = 1)
            self._initialize_weights()

        def _initialize_weights(self):
            for m in self.modules():
                if isinstance(m, nn.Linear):
                    m.weight.data.normal_(0, 0.1)
                    m.bias.data.zero_()

        def forward(self, x):
        x = self.fc1(x)
        if(self.output_activation == 'softmax'):
            x = self.softmax(x)
        return x

    def __init__(self):
        super(PolicyGradientLearner, self).__init__()

    def pathlength(self, path):
        return len(path["reward"])

    def sample_action(self, logit, discrete):
        if discrete:
            m = Categorical(logit)
            action = m.sample()
            log_odds_action = m.log_prob(action)
            return action, log_odds_action#, probability
        else:
            shape = int(logit.shape[1]/2)
            mu = logit[:,:shape]
            sigma = logit[:,shape:]
            sigma = F.softplus(sigma)
            
            m = Normal(mu, sigma)
            action = m.sample()
            log_odds_action = m.log_prob(action)
            #if math.isnan(log_odds_action):
            #    print(log_odds_action)
            #    print(action)
            #    print(logit)
                
            return action, log_odds_action

    def update_policy(self, paths, net, cumsum = 1):#ob_no, ac_na, rew, log_odd):
        num_paths = len(paths)

        rew_cums = []
        log_odds = []
        for path in paths:
            rew = path['reward']
            log_odd = path['log_odds']
            
            log_odds.append(log_odd)

            rew = torch.Tensor(rew)
            
            rew_cum = flip(torch.cumsum(flip(rew,0),0),0) #make a matrix multiplication to incorporate the decreasing value too
            if cumsum:
                rew_cums.append(rew_cum)
            else: #raw sum, not using reward to go
                max_rew = torch.ones(len(rew)) * rew_cum[0]
                #rew_cums.append(torch.sum(rew,0)*rew_cum)
                #rew = (rew-rew+1)*torch.sum(rew,0)
                rew_cums.append(max_rew)

    def update(self,state, action, reward, log_odds):
        self.path['observation'].append(state)
        self.path['action'].append(action)
        self.path['reward'].append(reward)
        self.path['log_odds'].append(log_odds)


