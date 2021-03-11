import pdf.tools as _t
from abc import ABC, abstractmethod


class BaseComponent(ABC):
    @staticmethod
    def read_component_from(stream):
        for component_class in _t.get_descendants_recursively(BaseComponent):
            # noinspection PyUnresolvedReferences
            if component_class.probe(stream):
                return component_class().read_from_stream(stream)
        raise ValueError('Unrecognized stream.')

    @abstractmethod
    def read_from_stream(self, stream):
        ...

    @abstractmethod
    def write_to_stream(self, stream):
        ...

    @staticmethod
    @abstractmethod
    def probe(stream):
        ...


class Reference(BaseComponent):
    @staticmethod
    def probe(stream):
        pass

    def __init__(self):
        self._index = None
        self._generation = None

    def read_from_stream(self, stream):
        pass

    def write_to_stream(self, stream):
        pass

    def __eq__(self, other):
        if isinstance(other, Reference):
            return (self._index == other._index) and (self._generation == other._generation)
        else:
            return False
