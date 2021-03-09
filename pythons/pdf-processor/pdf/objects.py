from abc import ABC, abstractmethod


class BaseObject(ABC):
    @abstractmethod
    def read_from_stream(self, stream):
        ...

    @abstractmethod
    def write_to_stream(self, stream):
        ...


class Loaded(BaseObject):
    def read_from_stream(self, stream):
        pass

    def write_to_stream(self, stream):
        pass
