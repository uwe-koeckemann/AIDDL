from abc import abstractmethod

from .term import Term


class Numerical(Term):
    @abstractmethod
    def __add__(self, other):
        """Add numerical term to another numerical term."""
    @abstractmethod
    def __sub__(self, other):
        """Subtract numerical term from another numerical term."""
    @abstractmethod
    def __mul__(self, other):
        """Multiply numerical term with another numerical term."""
    @abstractmethod
    def __truediv__(self, other):
        """Divide numerical term by another numerical term."""
    @abstractmethod
    def __floordiv__(self, other):
        """Divide numerical term by another numerical term
        and round to nearest integer."""
    @abstractmethod
    def __lt__(self, other):
        """<"""
    @abstractmethod
    def __le__(self, other):
        """<="""
    @abstractmethod
    def __gt__(self, other):
        """>"""
    @abstractmethod
    def __ge__(self, other):
        """>="""


