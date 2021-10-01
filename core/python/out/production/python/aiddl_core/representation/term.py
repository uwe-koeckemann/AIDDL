from abc import ABC, abstractmethod
from aiddl_core.representation.substitution import Substitution


class Term(ABC):
    def size(self):
        return len(self)

    def __len__(self):
        """Get size if this is a collection term or tuple."""
        raise AttributeError("Not implemented for: " +
                             str(type(self)) + " " + str(self))

    def get(self, x):
        """Get term with key or at index index x."""
        raise AttributeError("Not implemented for: " +
                             str(type(self)) + " " + str(self))

    @abstractmethod
    def unpack(self):
        """Unpack AIDDL to regular python."""

    def getOrDefault(self, x, default):
        """Get term with key or at index index x.
        Return default value if requested data does not exit."""
        raise AttributeError("Not implemented for: " +
                             str(type(self)) + " " + str(self))

    def getOrPanic(self, x, default):
        r = self.get(x)
        if r is not None:
            return r
        raise AttributeError("Key %s not found in %s"
                             % (str(type(self)), str(self)))

    def __getitem__(self, key):
        return self.get(key)

    def __setitem__(self, key, value):
        raise AttributeError("Immutable object.")

    def match(self, other):
        if self == other:
            return Substitution()
        return None

    def resolve(self, container):
        return self

    def substitute(self, substitution):
        return substitution.substitute(self)

    @abstractmethod
    def __eq__(self):
        """Check if two terms are equal."""

    @abstractmethod
    def __ne__(self):
        """Check if two terms are not equal."""

    @abstractmethod
    def __str__(self):
        """Create string from term."""

    def __setattr__(self, name, value):
        raise AttributeError("Immutable object.")
