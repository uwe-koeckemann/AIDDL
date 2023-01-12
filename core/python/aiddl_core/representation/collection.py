import abc
import aiddl_core.representation.term as term


class Collection(term.Term):
    @abc.abstractmethod
    def __add__(self, other: term.Term) -> 'Collection':
        """Add term to collection.

        :param other: Term to add to this collection
        :return: Collection including the added term
        """

    @abc.abstractmethod
    def add_all(self, other: 'Collection') -> 'Collection':
        """Add another collection to this collection.

        :param other: Collection to add to this collection
        :return: Collection including the added terms
        """

    @abc.abstractmethod
    def put(self, key: term.Term, value: term.Term) -> 'Collection':
        """ Add key-value pair to collection.

        :param key: key term
        :param value: value term
        :return: this collection with a key value pair added
        """

    @abc.abstractmethod
    def put_all(self, other: 'Collection') -> 'Collection':
        """ Add collection of key values to collection.

        :param other: collection of key-value pairs
        :return: this collection with a key value pair added
        """

    @abc.abstractmethod
    def is_unique_map(self) -> bool:
        """ Check if this collection is a unique map (i.e., each key-value term has a different key)

        :return: true if each key-value pair in this collection has a different key, false otherwise
        """

    @abc.abstractmethod
    def is_empty(self) -> bool:
        """ Check if collection is empty.

        :return: true if this collection is empty, false otherwise
        """

    @abc.abstractmethod
    def remove(self, other: 'term.Term') -> 'Collection':
        """ Remove term from collection.

        :param other: term to remove
        :return: collection without the term
        """

    @abc.abstractmethod
    def remove_all(self, other: 'Collection') -> 'Collection':
        """ Remove collection of terms from collection.

        :param other: collection of terms to remove
        :return: collection without the terms
        """

    @abc.abstractmethod
    def __contains__(self, other: 'term.Term') -> 'bool':
        """ Check if a term is contained in this collection

        :param other: term to check
        :return: true if this collection contains the other term, false otherwise
        """

    def contains(self, other: 'term.Term') -> 'bool':
        """ Check if a term is contained in this collection

        :param other: term to check
        :return: true if this collection contains the other term, false otherwise
        """
        return other in self

    def contains_all(self, other: 'Collection') -> 'bool':
        """Check if collection contains another collection.

        :param other: collection
        :return: true if all terms in the other collection are also in this collection, false otherwise
        """
        for t in other:
            if t not in self:
                return False
        return True

    def contains_any(self, other: 'Collection') -> 'bool':
        """Check if collection contains at least one element of another collection.

        :param other: collection
        :return: true if any term in the other collection js also in this collection, false otherwise
        """
        for t in other:
            if t in self:
                return True
        return False

    @abc.abstractmethod
    def contains_key(self, other: 'term.Term') -> 'bool':
        """ Check if this collection term contains a key value with a specific key.

        :param other: a key term to search for
        :return: true if this collection contains a key-value pair with the specified key, false otherwise
        """
