__author__ = 'rbtying'
"""
A driver program for rsvp_display - Sets some prameters and runs rsvp_display
If simulate_bci is not set, it also initializes the BCI engine.
"""
import time
import os
import sys

import rospy

import pygame
pygame.init()

import argparse

from bci_engine import BCIEngine
from rsvp_display import RSVPDisplay
import logging
import datetime

TRIALNAME = 'trial3'
SLUG = 'bci_{}_log_{}'.format(TRIALNAME, datetime.datetime.now().strftime('%b_%d_%H_%M_%S'))

logger = logging.getLogger('trial')
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('logs/{}.log'.format(SLUG))
handler.setLevel(logging.DEBUG)
sh = logging.StreamHandler()
sh.setLevel(logging.DEBUG)
logger.addHandler(handler)
logger.addHandler(sh)

logger.info('Initializing experiment')


def main_function():
    parser = argparse.ArgumentParser(description='ROS node to provide ranking action')
    parser.add_argument('--bci_host', type=str, required=False, default='localhost',
                        help='TCP/IP host for the BCI publisher_consumer')
    parser.add_argument('--bci_port', type=int, required=False, default=4444,
                        help='TCP/IP port for the BCI publisher_consumer')

    parser.add_argument('--width', type=int, required=False, default=640, help='Width of the window')
    parser.add_argument('--height', type=int, required=False, default=640, help='Height of the window')

    args = parser.parse_args(sys.argv[1:])

    for var in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_PACKAGE_PATH'):
        if not var in os.environ:
            print >>sys.stderr, '{} not set in environment!'.format(var)

    # ensure that we have a new session
    BCIEngine(args.bci_host, args.bci_port).end_session()
    time.sleep(4)

    print 'connecting to producer/consumer'
    rospy.init_node('rsvp_bci_node')
    r = RSVPDisplay(use_bci=True,
                    hostname=args.bci_host,
                    port=args.bci_port,
                    size=(args.width, args.height))

    PATH = r'C:\Program Files\Neuromatters\cbci'

    print 'entering testing mode'
    time.sleep(10)
    # path
    fnames = sorted((p for p in os.listdir(PATH) if p.startswith('_cbci_')),
                    key=lambda x: os.path.getmtime(os.path.join(PATH, x)))

    logger.info('Using training session {}'.format(fnames[-1]))
    r.bci.start_testing_session(fnames[-1])
    logger.info('session started!')

    TRIAL_PATH = os.path.join(r'C:\Users\rbtying\Desktop\graspit\act_refin_imgs', TRIALNAME)
    TARGET_PATH = os.path.join(TRIAL_PATH, 'target')
    NONTARGET_PATH = os.path.join(TRIAL_PATH, 'nontarget')

    targets = []
    nontargets = []

    for fn in os.listdir(TARGET_PATH):
        fp = os.path.join(TARGET_PATH, fn)
        img = pygame.image.load(fp)
        targets.append(img)
        logger.info('Adding {} to targets'.format(fp))

    for fn in os.listdir(NONTARGET_PATH):
        fp = os.path.join(NONTARGET_PATH, fn)
        img = pygame.image.load(fp)
        nontargets.append(img)
        logger.info('Adding {} to nontargets'.format(fp))

    try:
        r.do_experiment(targets, nontargets, SLUG)
    finally:
        print 'ending session'
        logger.info('ending session')
        r.bci and r.bci.end_session()

    print 'shutting down'
    rospy.signal_shutdown('Exiting program')
    logging.shutdown()
    quit()
    sys.exit(0)

if __name__ == '__main__':
    main_function()
