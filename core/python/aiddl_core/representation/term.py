from abc import ABC, abstractmethod
from typing import Optional, Union

# from aiddl_core.container.container import Container
from aiddl_core.representation.substitution import Substitution


class Term(ABC):
    def size(self):
        return len(self)

    def __len__(self) -> int:
        """ Get size if this is a collection term or tuple

        :return: length of term if applicable
        """
        raise AttributeError("Not implemented for: " +
                             str(type(self)) + " " + str(self))

    def __call__(self, x: Union[int, 'Term']) -> 'Term':
        """ Apply x to this term (see get(x) below)

        :param x:
        """
        self.get(x)

    def get(self, x: Union[int, 'Term']) -> Optional['Term']:
        """ Apply x to this term and get the result

        If this term is a collection or tuple and x is a term, this method will look for a
        key-value pair with key x. If this term is a tuple or list and x is an integer,
        this method will look for the value at index x. If this term is a function reference
        to a function f, this method will return f(x).

        :param x: integer index or term
        :return: term resulting from applying x to this term
        """
        raise AttributeError(f"Not implemented for: {type(self)} {self}")

    @abstractmethod
    def unpack(self) -> object:
        """Unpack AIDDL to regular python object."""

    def get_or_default(self, x, default) -> 'Term':
        """ Apply x to this term and get the result if defined or a default value

        :param x: integer index or term
        :param default: default value
        :return: result or default
        """
        return default

    def get_or_panic(self, x, default):
        """

        :param x:
        :param default:
        :return:
        """
        r = self.get(x)
        if r is not None:
            return r
        raise AttributeError("Key %s not found in %s"
                             % (str(type(self)), str(self)))

    def __getitem__(self, key):
        return self.get(key)

    def __setitem__(self, key, value):
        raise AttributeError("Immutable object.")

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

        :param substitution:
        :return:
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
