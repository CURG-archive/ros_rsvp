__author__ = 'rbtying'

import time
import os
import sys

import rospy

import pygame
pygame.init()

import argparse

from bci_engine import BCIEngine
from rsvp_display import RSVPDisplay


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

    if r.bci:
        print 'entering testing mode'
        time.sleep(10)
        r.bci.start_testing_session('_cbci_735941.5806')

    try:
        r.do_loop()
    finally:
        print 'ending session'
        r.bci and r.bci.end_session()

    print 'shutting down'
    rospy.signal_shutdown('Exiting program')
    quit()
    sys.exit(0)

if __name__ == '__main__':
    main_function()
