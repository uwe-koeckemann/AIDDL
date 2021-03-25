import abc
import aiddl_core.representation.term as term


class Collection(term.Term):
    @abc.abstractmethod
    def __add__(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def add_all(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def put(self, key, value):
        """Add term to collection."""

    @abc.abstractmethod
    def put_all(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def is_empty(self):
        """Add term to collection."""

    @abc.abstractmethod
    def remove(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def remove_all(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def __in__(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def contains(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def contains_all(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def contains_any(self, other):
        """Add term to collection."""

    @abc.abstractmethod
    def contains_key(self, other):
        """Add term to collection."""
