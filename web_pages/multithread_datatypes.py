'''
Implements thread-safe data types.
'''

import threading

class ThreadsafeRoomList:
    '''Thread-safe list for staff rooms'''
    def __init__(self) -> None:
        self._rooms = []
        self._lock = threading.Lock()

    def add_room(self, room_number) -> None:
        '''Add a room to the list'''
        with self._lock:
            self._rooms.append(room_number)

    def get_next_room(self) -> int:
        '''Get the next room from the front'''
        with self._lock:
            if not self._rooms:
                return None
            return self._rooms.pop(0)

    def remove_room(self, room_number) -> bool:
        '''Remove a specific room if it exists in the list'''
        with self._lock:
            if room_number in self._rooms:
                self._rooms.remove(room_number)
                return True
            return False

    def is_empty(self) -> bool:
        '''Check if the list is empty'''
        with self._lock:
            return len(self._rooms) == 0

    def size(self) -> bool:
        '''Get the current size of the list'''
        with self._lock:
            return len(self._rooms)

    def peek(self) -> int:
        '''View the next room without removing it'''
        with self._lock:
            if not self._rooms:
                return None
            return self._rooms[0]

    def contains(self, room_number) -> bool:
        '''Check if a room exists in the list'''
        with self._lock:
            return room_number in self._rooms
