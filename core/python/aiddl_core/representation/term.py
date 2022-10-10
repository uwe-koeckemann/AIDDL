from abc import ABC, abstractmethod
from typing import Optional, Union

# from aiddl_core.container.container import Container
from aiddl_core.representation.substitution import Substitution


class Term(ABC):
    def size(self) -> int:
        """ Get size if this is a collection term or tuple

        :return: length of term if applicable
        :raises: AttributeError if length not defined for this term
        """
        return len(self)

    def __len__(self) -> int:
        """ Get size if this is a collection term or tuple

        :return: length of term if applicable
        :raises: AttributeError if length not defined for this term
        """
        raise AttributeError("Not implemented for: " +
                             str(type(self)) + " " + str(self))

    def __call__(self, x: Union[int, 'Term']) -> Optional['Term']:
        """ Apply x to this term (see get(x) below)

        :param x: integer index or term
        :return: term resulting from applying x to this term if defined, None if the
                 operation is supported but the result undefined
        :raises: attribute error if this term does not support the operation
        """
        self.get(x)

    def __getitem__(self, x: Union[int, 'Term']) -> Optional['Term']:
        """ Apply x to this term (see get(x) below)

        :param x: integer index or term
        :return: term resulting from applying x to this term if defined, None if the
                 operation is supported but the result undefined
        :raises: attribute error if this term does not support the operation
        """
        return self.get(x)

    def get(self, x: Union[int, 'Term']) -> Optional['Term']:
        """ Apply x to this term and get the result

        If this term is a collection or tuple and x is a term, this method will look for a
        key-value pair with key x. If this term is a tuple or list and x is an integer,
        this method will look for the value at index x. If this term is a function reference
        to a function f, this method will return f(x). Otherwise, an exception is raised.

        :param x: integer index or term
        :return: term resulting from applying x to this term if defined, None if the
                 operation is supported but the result undefined
        :raises: attribute error if this term does not support the operation
        """
        raise AttributeError(f"Not implemented for: {type(self)} {self}")

    def get_or_default(self, x, default) -> 'Term':
        """ Use self.get(x) and return a default value if the result is None

        :param x: integer index or term
        :param default: a default value
        :return: result term or default
        """
        result = self.get(x)
        if result is not None:
            return result
        else:
            return default

    def get_or_panic(self, x, default):
        """ Use self.get(x) and throw an exception if the result is None

        :param x:
        :param default:
        :return:
        """
        result = self.get(x)
        if result is not None:
            return result
        raise AttributeError(f"Key {type(self)} not found in {self}")

    @abstractmethod
    def unpack(self) -> object:
        """  Unpack AIDDL term to regular python object.

        :return: a python object version of this AIDDL term
        """

    def match(self, other: 'Term') -> Optional['Substitution']:
        """ Attempt to match this term to another

        :param other: another term
        :return: a substitution that makes this term equal to the other one or None if this is not possible
        """
        if self == other:
            return Substitution()
        return None

    def resolve(self, container: 'Container'):
        """ Recursively resolve any references found in this term

        :param container: Used to look up the values of any referenced entries
        :return: Term with all references resolved
        """
        return self

    def substitute(self, substitution: 'Substitution') -> 'Term':
        """ Apply a substitution to this term and return the result.

        :param substitution: collection of replacement terms
        :return: term with all replacements applied
        """
        return substitution.substitute(self)

    @abstractmethod
    def __eq__(self, other):
        """Check if two terms are equal."""

    @abstractmethod
    def __ne__(self, other):
        """Check if two terms are not equal."""

    @abstractmethod
    def __str__(self, other):
        """Create string from term."""

    def __setattr__(self, name, value):
        raise AttributeError("Immutable object.")

    def __setitem__(self, key, value):
        raise AttributeError("Immutable object.")
