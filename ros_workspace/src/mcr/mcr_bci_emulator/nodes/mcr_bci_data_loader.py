#!/usr/bin/env python3

# import dependencies
import numpy as np
import os
import random

# PTF = path to file
PTF = str(os.path.dirname(__file__))

# load prepared signals
SIGNALS = np.load( PTF + '/signals/signals.npy', allow_pickle = True)
TRUE_TARGETS = np.load( PTF + '/signals/labels.npy', allow_pickle = True).argmax(1) 

# load run info
__log_txt_file_handle = open(PTF + "/presentation_layer/init_msg.txt","r")
LOGINFO = __log_txt_file_handle.read()
__log_txt_file_handle.close()

# DEBUG mode flag
DEBUG = False

class Brain:
    def __init__(self):
        self.brainwaves = SIGNALS
        self.intentions = TRUE_TARGETS
        self.brainwaves_chunked = self._split_data_by_target()
        self._stream_stack = []

    def _split_data_by_target(self):
        brainwaves_blocks = [ [] for _ in range(5) ]

        for i in range(len(self.intentions)):
            intention = self.intentions[i]
            brainwaves_blocks[intention].append(self.brainwaves[i])

        if DEBUG == True:
            for i in range(len(brainwaves_blocks)):
                print(len(brainwaves_blocks[i]))

        return brainwaves_blocks

    def get_brainwave_block(self, block_idx):
        # block_idx legend:
        # 0 - forward
        # 1 - backward
        # 2 - rotate left
        # 3 - rotate right
        # 4 - stop 

        bw_idx = random.randint(0, len(self.brainwaves_chunked[block_idx])-1)
        random_data_block = self.brainwaves_chunked[block_idx][bw_idx]
        return random_data_block

    def get_data_to_stream(self, intention_idx, purity_frac, stream_length, return_stream = False):
        stream = [ [] for _ in range(stream_length)]

        nb_dist_blocks = int((1-purity_frac)*stream_length)

        for i in range(nb_dist_blocks): 
            drawn_idx = random.randint(0,stream_length-1)
            if len(stream[drawn_idx]) == 0:
                stream[drawn_idx] = self.get_brainwave_block(4) # idle state brainwave 
            else:
                i -= 1 # if cell in stream array is occupied - give yourself another chance :)


        for i in range(stream_length):
            if len(stream[i]) == 0:
                stream[i] = self.get_brainwave_block(intention_idx)
        
        self._stream_stack = stream

        if return_stream == True:
            return stream 
    

    def feed_data(self,**args):
        if len(self._stream_stack) != 0:
            return self._stream_stack.pop() 
        
        i_indx = 0
        p_frac = 0.8
        s_lght = 10

        if "intention_idx" in args:
            i_indx = args["intention_idx"]
        if "purity_frac" in args:
            p_frac = args["purity_frac"]
        if "stream_length" in args:
            s_lght = args["stream_length"]

        self.get_data_to_stream(intention_idx=i_indx, purity_frac = p_frac, stream_length = s_lght)
        
        return self._stream_stack.pop()



