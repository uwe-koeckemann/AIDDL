from aiddl_core.container.entry import Entry
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.substitution import Substitution

MOD = Sym("#mod")


class Module:
    def __init__(self, module_uri: Sym):
        """ Create a new module

        :param module_uri: name of the module
        """
        self._module_uri = module_uri
        self._data = {}

    @property
    def name(self) -> Sym:
        """ Name of the module

        :return: name of the module
        """
        return self._module_uri

    def put_entry(self, e):
        """ Add an entry to this module.

        Existing entries with the same name will be overwritten.

        :param e: added entry
        """
        self._data[e.name] = e

    def remove_entry(self, e: Entry):
        """ Remove an entry with the same name as e.

        :param e: entry to remove
        """
        del self._data[e.name]

    def get_entries(self):
        """ Get all entries from this module.

        :return: list of entries
        """
        return self._data.values()

    def substitute(self, s: Substitution):
        """ Apply a substitution to all entries in this module

        :param s: substitution to apply
        """
        subbed_entries = []
        for e in self._data.values():
            subbed_entries.append(e.substitute(s))
        for e in subbed_entries:
            self.put_entry(e)

