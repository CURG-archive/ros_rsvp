__author__ = 'rbtying'
import random
import pygame
import collections
import heapq
import math


class Trial(object):

    class State(object):
        INIT = 0
        PREVIEW = 1
        RUNNING = 2
        COMPLETED = 3
        ABORTED = 4

    BG_COLORS = [
        (255, 0, 0), (255, 102, 0), (255, 166, 64), (230, 226, 172), (0, 230, 0), (61, 242, 157), (0, 82, 102),
        (182, 190, 242), (135, 115, 153), (242, 0, 129), (191, 96, 134), (204, 0, 0), (229, 161, 115), (153, 122, 0),
        (109, 115, 86), (0, 217, 0), (38, 153, 99), (61, 182, 242), (37, 0, 140), (43, 0, 64), (255, 191, 225),
        (89, 0, 12), (217, 170, 163), (64, 45, 32), (76, 65, 19), (85, 128, 0), (0, 179, 0), (121, 242, 218),
        (32, 57, 128), (137, 108, 217), (230, 115, 222), (77, 57, 68), (229, 57, 80), (115, 52, 29), (127, 68, 0),
        (229, 218, 57), (43, 51, 38), (16, 64, 29), (35, 140, 133), (38, 48, 77), (157, 61, 242), (115, 0, 92),
        (51, 0, 20)
    ]

    def __init__(self, options, size=(640, 480), preview_time=1000.0, image_time=100, min_repeat=2, max_repeat=5):
        """
        Creates a trial
        :param options: list of pygame Surfaces to display
        :param size: the size of the window
        :param preview_time: the amount of time to show the preview
        :param image_time: the amount of time to show each image
        :param min_repeat: the minimum number of times each image is repeated
        :param max_repeat: the maximum number of times each image is repeated
        :return:
        """
        self.mode = Trial.State.INIT
        self.size = size
        self.preview_time = preview_time
        self.image_time = image_time
        self.options = [pygame.transform.smoothscale(opt) for opt in options]
        self.font = pygame.font.Font(None, 36)

        self.min_repeat = max(1, min_repeat)
        self.max_repeat = max(self.min_repeat, max_repeat)

        self.index_list, self.option_counts = self._construct_trial()

        self.index_ptr = 0
        self.selected_index_ptr = None

    def show_next_image(self, screen, bci=None):
        """
        Shows the next image in the sequence
        :param screen:
        :param bci:
        :return: the time the image should stay on screen before show_next_image is called again
        """
        if self.mode == Trial.State.INIT:
            self.mode = Trial.State.PREVIEW
            screen.fill((0, 0, 0))
            self._display_preview_image(screen)
            screen.blit(self.font.render(
                'Previewing {} images'.format(len(self.options)), 1, (255, 255, 255)), (40, self.size[1] / 2))
            return self.preview_time
        elif self.mode in (Trial.State.PREVIEW, Trial.State.RUNNING):
            if self.mode == Trial.State.PREVIEW:
                self.mode = Trial.State.RUNNING
                self.index_ptr = 0
            else:
                self.index_ptr += 1

            bci and bci.mark_unlabeled()

            im_idx = self.index_list[self.index_ptr]
            screen.fill(self.BG_COLORS[im_idx])
            screen.blit(self.options[im_idx], (0, 0))

            if self.index_ptr >= len(self.index_list):
                self.mode = Trial.State.COMPLETED

            return self.image_time
        else:
            return None

    def _display_preview_image(self, screen):
        """
        Creates a preview image which shows each of the individual images
        in a grid
        :return:
        """
        elements_per_side = int(math.sqrt(len(self.options)) + 0.5)
        rowlist = list(zip(*((iter(self.options)),) * elements_per_side))

        print rowlist

        pass

    def _construct_trial(self):
        """
        Creates a sequence of indexes which minimizes the chance that adjacent
        indices are equal
        :return: a list of indexes, and their counts
        """
        option_counts = [random.randrange(self.min_repeat, self.max_repeat) for _ in self.options]
        index_list = list(self._nonadjacent(option_counts))

        return index_list, option_counts

    @staticmethod
    def _nonadjacent(option_counts):
        heap = [(-count, key) for (key, count) in enumerate(option_counts)]
        heapq.heapify(heap)
        count, key = 0, None
        while heap:
            count, key = heapq.heapreplace(heap, (count, key)) if count else heapq.heappop(heap)
            yield key
            count += 1
        for index in xrange(-count):
            yield key
