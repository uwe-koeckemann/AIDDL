from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Union, TYPE_CHECKING

from aiddl_core.representation.substitution import Substitution

if TYPE_CHECKING:
    from aiddl_core.representation import List, KeyVal, Num, Var, Sym, Set, Tuple, Str, Int, Real, Rat, Inf, NaN, \
    Collection, EntRef, FunRef, Boolean
    from aiddl_core.container import Container

class Term(ABC):
    """
    Abstract base class representing a term.

    This class serves as a foundation for various types of terms that can be
    used in the Abstract Intelligent Design Description Language (AIDDL). It
    """
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

    def get_or_panic(self, x: Union[int, 'Term']):
        """ Use self.get(x) and throw an exception if the result is None

        :param x: integer index or term
        :return: result term
        """
        result = self.get(x)
        if result is not None:
            return result
        raise KeyError(f"Key {x} not found in {self}")

    @abstractmethod
    def unpack(self) -> object:
        """  Unpack AIDDL term to a regular python object.

        :return: Python object version of this AIDDL term
        """

    def match(self, other: 'Term') -> Optional['Substitution']:
        """ Attempt to match this term to another

        :param other: another term
        :return: substitution that makes this term equal to the other one or None if this is not possible
        """
        if self == other:
            return Substitution()
        return None

    def resolve(self, container: Container) -> 'Term':
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

    def as_sym(self) -> Sym:
        """
        :return: this term as a symbol (Sym) if possible
        """
        raise ValueError(f"Term {self} is not a symbol (Sym)")

    def as_bool(self) -> Boolean:
        """
        :return: this term as a symbol (Sym) if possible
        """
        raise ValueError(f"Term {self} is not a Boolean (Boolean)")

    def as_var(self) -> Var:
        """
        :return: this term as a variable (Var) if possible
        """
        raise ValueError(f"Term {self} is not a variable (Var)")

    def as_str(self) -> Str:
        """
        :return: this term as a string (Str) if possible
        """
        raise ValueError(f"Term {self} is not a string (Str)")

    def as_num(self) -> Num:
        """
        :return: this term as a numerical (Num) if possible
        """
        raise ValueError(f"Term {self} is not a numerical (Num)")

    def as_int(self) -> Int:
        """
        :return: this term as an integer (Int) if possible
        """
        raise ValueError(f"Term {self} is not a int (Int)")

    def as_rat(self) -> Rat:
        """
        :return: this term as a rational number (Rat) if possible
        """
        raise ValueError(f"Term {self} is not a rational (Rat)")

    def as_real(self) -> Real:
        """
        :return: this term as a floating point number (Real) if possible
        """
        raise ValueError(f"Term {self} is not a real (Real)")

    def as_inf(self) -> Inf:
        """
        :return: this term as infinity (Inf) if possible
        """
        raise ValueError(f"Term {self} is not infinity (Inf)")

    def as_nan(self) -> NaN:
        """
         :return: this term as not-a-number (NaN) if possible
         """
        raise ValueError(f"Term {self} is not Not-a-Number (NaN)")

    def as_kvp(self) -> KeyVal:
        """
        :return: this term as a key-value pair (KeyVal) if possible
        """
        raise ValueError(f"Term {self} is not a key-value pair (KeyVal)")

    def as_col(self) -> Collection:
        """
        :return: this term as a collection (Collection) if possible
        """
        raise ValueError(f"Term {self} is not a list (List)")

    def as_list(self) -> List:
        """
        :return: this term as a list (List) if possible
        """
        raise ValueError(f"Term {self} is not a list (List)")

    def as_set(self) -> Set:
        """
        :return: this term as a set (Set) if possible
        """
        raise ValueError(f"Term {self} is not a set (Set)")

    def as_tuple(self) -> Tuple:
        """
        :return: this term as a tuple (Tuple) if possible
        """
        raise ValueError(f"Term {self} is not a tuple (Tuple)")

    def as_ent_ref(self) -> EntRef:
        """
        :return: this term as an entry reference (EntRef) if possible
        """
        raise ValueError(f"Term {self} is not an entry reference (EntRef)")

    def as_fun_ref(self) -> FunRef:
        """
        :return: this term as a function reference (FunRef) if possible
        """
        raise ValueError(f"Term {self} is not a function reference (FunRef)")

    @abstractmethod
    def __eq__(self, other) -> bool:
        """Check if two terms are equal."""

    @abstractmethod
    def __ne__(self, other) -> bool:
        """Check if two terms are not equal."""

    @abstractmethod
    def __str__(self) -> str:
        """Create string from term."""

    @abstractmethod
    def __hash__(self):
        """Get hash value of this term"""

    def __setattr__(self, name, value):
        raise AttributeError("Immutable object.")

    def __setitem__(self, key, value):
        raise AttributeError("Immutable object.")
