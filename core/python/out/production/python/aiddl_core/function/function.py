from abc import ABC, abstractmethod


class Function(ABC):
    @abstractmethod
    def __call__(self, args):
        """Apply function to input term."""


class LazyFunction(Function):
    @abstractmethod
    def __call__(self, args):
        """Apply function to input term."""


class InitializableFunction(ABC):
    @abstractmethod
    def initialize(self, args):
        """Initialize function."""


class ConfigureFunction(ABC):
    @abstractmethod
    def configure(self, cfg, freg):
        """Configure function."""


class InterfaceImplementation(ABC):
    @abstractmethod
    def get_interface_uri(self):
        """Get function interface URI."""
