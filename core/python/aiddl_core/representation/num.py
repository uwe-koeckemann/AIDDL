from abc import abstractmethod

from .term import Term


class Num(Term):
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

    @abstractmethod
    def is_zero(self):
        """is zero"""

    @abstractmethod
    def is_positive(self):
        """is positive"""

    @abstractmethod
    def is_negative(self):
        """is negative"""

    def is_nan(self):
        return False

    def is_inf(self):
        return False

    def is_inf_pos(self):
        return False

    def is_inf_neg(self):
        return False


