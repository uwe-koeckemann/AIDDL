from abc import abstractmethod

from .term import Term


class Num(Term):
    @abstractmethod
    def __add__(self, other: 'Num') -> 'Num':
        """ Add numerical term to another numerical term.

        :param other: term to add to this term
        :return: result of addition
        """

    @abstractmethod
    def __sub__(self, other: 'Num') -> 'Num':
        """ Subtract numerical term from this numerical term.

        :param other: term to subtract from this term
        :return: result of subtraction
        """

    @abstractmethod
    def __mul__(self, other: 'Num') -> 'Num':
        """ Multiply numerical term with this numerical term.

        :param other: term to multiply with this term
        :return: result of multiplication
        """

    @abstractmethod
    def __truediv__(self, other: 'Num') -> 'Num':
        """Divide this term by another numerical term.

        :param other: term to divide this term by
        :return: result of division
        """

    @abstractmethod
    def __floordiv__(self, other: 'Num') -> 'Num':
        """Divide this term by another numerical term and round to the nearest integer.

        :param other: term to divide this term by
        :return: result of division rounded down to the nearest integer
        """

    @abstractmethod
    def __lt__(self, other: 'Num') -> bool:
        """ Check if this term is less than another numerical term

        :param other: term to compare to
        :return: self < other
        """

    @abstractmethod
    def __le__(self, other: 'Num') -> bool:
        """ Check if this term is less than or equal to another numerical term

        :param other: term to compare to
        :return: self <= other
        """

    @abstractmethod
    def __gt__(self, other: 'Num') -> bool:
        """ Check if this term is greater than another numerical term

        :param other: term to compare to
        :return: self > other
        """

    @abstractmethod
    def __ge__(self, other: 'Num') -> bool:
        """ Check if this term is greater than or equal to another numerical term

        :param other: term to compare to
        :return: self >= other
        """

    @abstractmethod
    def is_zero(self) -> bool:
        """ Check if this term is zero

        :return: true if this numerical is equal to zero, false otherwise
        """

    @abstractmethod
    def is_positive(self) -> bool:
        """ Check if this term is positive

        :return: self > 0
        """

    @abstractmethod
    def is_negative(self) -> bool:
        """ Check if this term is positive

        :return: self < 0
        """

    def is_nan(self):
        """ Check if this term is not a number

        Note: this is required because NaN is not considered equal to NaN

        :return: true if this term is the non-a-number term, false otherwise
        """
        return False

    def is_inf(self):
        """ Check if this term is infinite

        Note: this is required because infinity is not considered equal to infinity

        :return: true if this term is positive or negative infinity, false otherwise
        """
        return False

    def is_inf_pos(self):
        """ Check if this term is positive infinity

        Note: this is required because infinity is not considered equal to infinity

        :return: true if this term is positive infinity, false otherwise
        """
        return False

    def is_inf_neg(self):
        """ Check if this term is positive infinity

        Note: this is required because infinity is not considered equal to infinity

        :return: true if this term is negative infinity, false otherwise
        """
        return False


