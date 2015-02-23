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





def main_function():
    parser = argparse.ArgumentParser(description='ROS node to provide ranking action')
    parser.add_argument('--simulate_bci', action='store_true', default=False,
                        help='Set this value to simulate the BCI device. Note that random indices will be selected'
                             'instead')
    parser.add_argument('--bci_host', type=str, required=False, default='localhost',
                        help='TCP/IP host for the BCI publisher_consumer')
    parser.add_argument('--bci_port', type=int, required=False, default=4444,
                        help='TCP/IP port for the BCI publisher_consumer')

    parser.add_argument('--width', type=int, required=False, default=640, help='Width of the window')
    parser.add_argument('--height', type=int, required=False, default=640, help='Height of the window')

    args = parser.parse_args(sys.argv[1:])


    SLUG = 'bci_{}_log_{}'.format('inactive' if args.simulate_bci else 'active', datetime.datetime.now().strftime('%b_%d_%H_%M_%S'))

    logger = logging.getLogger('trial')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('logs/{}.log'.format(SLUG))
    handler.setLevel(logging.DEBUG)
    sh = logging.StreamHandler()
    sh.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.addHandler(sh)

    for var in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_PACKAGE_PATH'):
        if not var in os.environ:
            print >>sys.stderr, '{} not set in environment!'.format(var)

    # ensure that we have a new session
    if not args.simulate_bci:
        BCIEngine(args.bci_host, args.bci_port).end_session()
        time.sleep(4)

    print 'connecting to producer/consumer'
    rospy.init_node('rsvp_bci_node')
    r = RSVPDisplay(use_bci=not args.simulate_bci,
                    hostname=args.bci_host,
                    port=args.bci_port,
                    size=(args.width, args.height))

    PATH = r'C:\Program Files\Neuromatters\cbci'

    if r.bci:
        print 'entering testing mode'
        time.sleep(10)
        # path
        fnames = sorted((p for p in os.listdir(PATH) if p.startswith('_cbci_')),
                        key=lambda x: os.path.getmtime(os.path.join(PATH, x)))

        logger.info(fnames)

        logger.info('Using training session {}'.format(fnames[-1]))
        r.bci.start_testing_session(fnames[-1])
        logger.info('session started!')

    try:
        r.do_loop(SLUG)
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
