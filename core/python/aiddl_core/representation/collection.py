import abc
import aiddl_core.representation.term as term


class Collection(term.Term):
    @abc.abstractmethod
    def __add__(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def add_all(self, other):
        """Add collection to collection."""

    @abc.abstractmethod
    def put(self, key, value):
        """Add key value to collection."""

    @abc.abstractmethod
    def put_all(self, other):
        """Add collection of key values to collection."""

    @abc.abstractmethod
    def is_empty(self):
        """Check if collection is empty."""

    @abc.abstractmethod
    def remove(self, other):
        """Remove term from collection."""

    @abc.abstractmethod
    def remove_all(self, other):
        """Remove terms to collection."""

    @abc.abstractmethod
    def __in__(self, other):
        """Check if term is in collection."""

    @abc.abstractmethod
    def contains(self, other):
        """Check if collection contains term."""

    @abc.abstractmethod
    def contains_all(self, other):
        """Check if collection contains another collection."""

    @abc.abstractmethod
    def contains_any(self, other):
        """Check if collection contains any term of another."""

    @abc.abstractmethod
    def contains_key(self, other):
        """Check if this collection term contains a key value with a specific key."""
