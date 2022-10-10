from aiddl_core.representation.term import Term
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.tuple import Tuple


class Entry:
    __slots__ = ["_type", "_name", "_value"]

    def __init__(self, entry_type, entry_name, entry_value):
        """ Create a new entry

        :param entry_type: type of the entry
        :param entry_name: name of the entry
        :param entry_value: value term of the entry
        """
        if entry_type is None or entry_name is None or entry_value is None:
            raise ValueError("Entry constructor needs non-null arguments.")
        self.__setattr__("_type", entry_type)
        self.__setattr__("_name", entry_name)
        self.__setattr__("_value", entry_value)

    @property
    def type(self) -> Term:
        """ The type of this entry

        :return: term representing the entry type
        """
        return self._type

    @property
    def name(self) -> Term:
        """ The name of this entry

        :return: term representing the entry name
        """
        return self._name

    @property
    def value(self) -> Term:
        """ The value of this entry

        :return: value term of this entry
        """
        return self._value

    def as_tuple(self) -> Tuple:
        """ Convert this entry to a tuple

        :return: tuple term with the same contents as the entry
        """
        return Tuple(self._type, self._name, self._value)

    def substitute(self, s: Substitution):
        """ Substitute the type, name, and value of this entry

        :param s: a substitution
        :return: new entry with the substitution applied
        """
        return Entry(self._type.substitute(s),
                     self._name.substitute(s),
                     self._value.substitute(s))

    def __hash__(self):
        return self._type.__hash__() \
            + 3*self._name.__hash__() \
            + 7*self._value.__hash__()

    def __eq__(self, other: object):
        if isinstance(other, Entry):
            return other.type == self._type \
                and other.name == self._name \
                and other.value == self._value
        return False

    def __ne__(self, other: object):
        return not self == other

    def __str__(self) -> str:
        return f"( {self.type} {self.name} {self.value})"

    def __repr__(self) -> str:
        return str(self._value)

    def unpack(self) -> object:
        """ Unpack entry and its contents in to a python tuple

        :return: python tuple with unpacked entry contents
        """
        return (self._type.unpack(),
                self._name.unpack(),
                self._value.unpack())
