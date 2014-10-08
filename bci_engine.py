from __future__ import print_function
__author__ = 'rbtying'

import parallel  # parallel port interface
from socket import socket
import time
import random


class BCIEngine(object):
    RESET_PORT = 0

    ###########################################################################
    # Note: The way the ProducerConsumer BCI Engine works is that it          #
    # watches the parallel port for commands, and then dumps them back        #
    # over TCP/IP.                                                            #
    ###########################################################################

    ### tcpip
    REQUEST_ATTENTION = 17              # request that tcp/ip server pays attention to input
    STOP = 'stop'                       # on program end

    ### parallel port
    COMMAND_LENGTH = 0.02               # time parallel port stays set to command, seconds
    MARK_LENGTH = 0.003                 # time parallel port stays on a given mark

    ### commands
    BEGIN_BLOCK = 20                    # indicates start of stimulus block
    END_BLOCK = 25                      # indicates end of stimulus block
    BASE_REF = 40                       # reference value of the stimulus
    GET_BLOCK_RESULTS = 30              # request results for last block
    TEST_MODE_START = 10                # indicates beginning of test mode
    TRAIN_MODE_START = 8                # indicates beginning of training session (train mode)
    DO_TRAIN = 12                       # train the classifier on the collected data
    DO_END_SESSION = 15                 # end the session

    ### labels
    NON_TARGET_CD = 80                  # labeled non target
    TARGET_CD = 160                     # labeled target
    UNLABELED_STIMULUS = 200            # unlabeled stimulus
    UNLABELED_STIMULUS_ONLINE = 240     # unlabeled stimulus, classify immediately
    TEST_TARGET = 210                   # labeled target test stimulus
    TEST_NON_TARGET = 220               # labeled non target test stimulus

    EVENT_TYPES = (NON_TARGET_CD, TARGET_CD, UNLABELED_STIMULUS, UNLABELED_STIMULUS_ONLINE)

    def __init__(self, hostname='127.0.0.1', port=4444, blocksize=4096):
        self.port = parallel.Parallel()
        self.port.setData(self.RESET_PORT)
        self.sock = socket()
        self.sock.connect((hostname, port))
        self.blocksize = blocksize

        self.in_block = False
        self.num_blocks = 0

        self.mode = None

    def begin_block(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_command(self.BEGIN_BLOCK)
        self.in_block = True

    def end_block(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')

        self._send_command(self.END_BLOCK)

        if self.in_block:
            self.num_blocks += 1
        self.in_block = False

    def base_ref(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_command(self.BASE_REF)

    def _parse_block(self, data):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        data = data.replace('[', ' ').replace(']', ' ')
        data_list = data.split(';')
        output = []
        for datum in data_list:
            block_id, image_id, eeg, sort_position = datum.split()
            output.append(BlockResult(int(block_id), int(image_id), float(eeg), int(sort_position)))
        return output

    def get_block_results(self):
        """
        Get results for the last block
        :return: block data
        """
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')

        if not self.in_block and self.num_blocks:
            self._send_command(self.GET_BLOCK_RESULTS)
            resp = self.sock.recv(self.blocksize)

            while True:
                try:
                    return self._parse_block(resp)
                except ValueError:
                    resp += self.sock.recv(self.blocksize)
        else:
            raise RuntimeError('Not ready for block results')

    def train(self):
        """
        Train the classifier
        :return: classifier name
        """
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_command(self.DO_TRAIN)
        resp = self.sock.recv(self.blocksize)
        return resp

    def start_training_session(self):
        self._send_command(self.TRAIN_MODE_START)
        self.mode = 'Train'

    def start_testing_session(self, classifier_name):
        self._send_command(self.TEST_MODE_START)
        self.sock.send(classifier_name + '\n')
        resp = self.sock.recv(self.blocksize)
        if not 'OK' in resp:
            raise RuntimeError('Could not start test mode: {}'.format(resp))
        self.mode = 'Test'

    def end_session(self):
        self._send_command(self.DO_END_SESSION)
        self.mode = None

    def mark_target(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_mark(self.TARGET_CD)

    def mark_non_target(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_mark(self.NON_TARGET_CD)

    def mark_unlabeled(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_mark(self.UNLABELED_STIMULUS)

    def mark_unlabeled_online(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_mark(self.UNLABELED_STIMULUS_ONLINE)

    def mark_test_target(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_mark(self.TEST_TARGET)

    def mark_test_non_target(self):
        if not self.mode:
            raise RuntimeError('need to enter training or test mode before running function')
        self._send_mark(self.TEST_NON_TARGET)

    def _send_mark(self, cmd):
        self.reset()
        time.sleep(self.MARK_LENGTH)
        self.port.setData(cmd)
        time.sleep(self.MARK_LENGTH)
        self.reset()

    def _send_command(self, cmd):
        self.reset()
        time.sleep(self.COMMAND_LENGTH)
        self.port.setData(cmd)
        time.sleep(self.COMMAND_LENGTH)
        self.reset()

    def reset(self):
        self.port.setData(self.RESET_PORT)

    def close(self):
        self.reset()
        self.end_session()
        self.sock.send(self.STOP)
        self.sock.close()


class BlockResult(object):

    @classmethod
    def random_blocks(cls, block_id, option_counts, index_list):
        rand_opt = list(enumerate(option_counts))
        random.shuffle(rand_opt)

        index_lut = [[] for _ in range(len(option_counts))]
        for (i, idx) in enumerate(index_list):
            index_lut[idx].append(i)

        eegs = list(sorted([random.gauss(1.0e-9, 1.0e-10) for _ in range(len(index_list))]))
        for i in range(rand_opt[0][1]):
            eegs[-i-1] = 1.0e-11

        output = []
        for (idx, counts) in rand_opt:
            for j in range(counts):
                output.append(BlockResult(block_id, index_lut[idx].pop() + 1, eegs.pop(), len(index_list) - len(eegs)))
        return output

    def __init__(self, block_id, image_id, eeg, sort_position):
        self.block_id = block_id
        self.image_id = image_id
        self.eeg = eeg
        self.sort_position = sort_position

    def __str__(self):
        return 'blk: {}\timg: {}\teeg: {}\tsort_position: {}'.format(
            self.block_id, self.image_id, self.eeg, self.sort_position)

    def __unicode__(self):
        return self.__str__()