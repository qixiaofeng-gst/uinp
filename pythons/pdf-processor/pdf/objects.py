from abc import ABC, abstractmethod


class BaseObject(ABC):
    @abstractmethod
    def read_from_stream(self, stream):
        ...

    @abstractmethod
    def write_to_stream(self, stream):
        ...

    @abstractmethod
    def probe(self, stream):
        ...
