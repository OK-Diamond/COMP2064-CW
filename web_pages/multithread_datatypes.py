'''
Implements thread-safe data types.
'''

import threading

class ThreadsafeList:
    '''Thread-safe list'''
    def __init__(self) -> None:
        self._items = []
        self._lock = threading.Lock()

    def add(self, data) -> None:
        '''Add a item to the list'''
        with self._lock:
            self._items.append(data)

    def get_next(self):
        '''Get the next item from the front'''
        with self._lock:
            if not self._items:
                return None
            return self._items.pop(0)

    def remove(self, data) -> bool:
        '''Remove a specific item if it exists in the list'''
        with self._lock:
            if data in self._items:
                self._items.remove(data)
                return True
            return False

    def is_empty(self) -> bool:
        '''Check if the list is empty'''
        with self._lock:
            return len(self._items) == 0

    def size(self) -> bool:
        '''Get the current size of the list'''
        with self._lock:
            return len(self._items)

    def peek(self) -> int:
        '''View the next item without removing it'''
        with self._lock:
            if not self._items:
                return None
            return self._items[0]

    def contains(self, data) -> bool:
        '''Check if a item exists in the list'''
        with self._lock:
            return data in self._items
