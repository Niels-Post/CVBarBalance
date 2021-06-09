from typing import Any
import numpy as np

def SMA_SimpleAverage(lst):
    return sum(lst) / len(lst)


def SMA_SimpleIntAverage(lst):
    return int(sum(lst) / len(lst))

def SMA_CoordinateAverage(lst):
    return np.average(np.asarray(lst), axis=0)

def SMA_IntCoordinateAverage(lst):
    return tuple(int(i) for i in SMA_CoordinateAverage(lst))


class SimpleMovingAverage:
    """
    A generic moving average queue.
    Can be used with any type of input data, as long as a fitting averaging_method is provided.
    """
    def __init__(self, length: int, initial_value: Any, averaging_method=SMA_SimpleAverage):
        self.length = length
        self.initial_value = initial_value
        self.data = [initial_value] * length

        self.averaging_method = averaging_method
        self._current_index = 0

    def add(self, value: Any):
        """
        Add a value to the queue. When the queue is filled, this overwrites the oldest value in the queue
        """
        self.data[self._current_index] = value
        self._current_index = (self._current_index + 1) % self.length

    def get(self):
        """
        Get the current calculated average.
        """
        return self.averaging_method(self.data)

    def clear(self):
        """
        Clear the queue. This sets all values back to the initial value
        """
        self.data = [self.initial_value] * self.length
