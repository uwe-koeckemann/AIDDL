import aiddl_core.representation.term as term
from aiddl_core.representation import Sym


class EntRef(term.Term):
    """An entry reference used to refer to terms in other entries."""
    __slots__ = ["_ref_target", "_alias", "_mod_name"]

    def __init__(self, ref_target, mod_name, alias=None):
        super(term.Term, self).__setattr__("_ref_target", ref_target)
        super(term.Term, self).__setattr__("_mod_name", mod_name)
        super(term.Term, self).__setattr__("_alias", alias)

    @property
    def target(self) -> term.Term:
        """ Target of the reference (i.e. the name of an entry)

        :return: name of target entry
        """
        return self._ref_target

    @property
    def module(self) -> Sym:
        """ Module of the reference entry

        :return: name of the module
        """
        return self._mod_name

    @property
    def alias(self) -> Sym:
        """ Alias of the reference entry

        :return: alias of the module
        """
        return self._alias

    def resolve(self, container):
        return container.resolve_reference(self)

    def convert2uri(self):
        """ Attempt to convert this entry reference to a symbolic URI

        Concatenates the module name with the target. Only works if the target is symbolic.

        :return: New symbol made of the module name and the target
        """
        return self._mod_name + self._ref_target

    def substitute(self, s):
        if self._alias is None:
            return EntRef(self._ref_target.substitute(s),
                          self._mod_name.substitute(s))
        else:
            return EntRef(self._ref_target.substitute(s),
                          self._mod_name.substitute(s),
                          alias=self._alias.substitute(s))

    def __str__(self):
        if self._alias is not None:
            return "%s@%s" % (str(self._ref_target), str(self._alias))
        else:
            return "%s@%s" % (str(self._ref_target), str(self._mod_name))

    def __repr__(self):
        return self.__str__()
        
    def __eq__(self, other):
        if isinstance(other, EntRef):
            return self._ref_target == other._ref_target \
                and self._mod_name == other._mod_name \
                and self._alias == other._alias
        else:
            return False

    def __ne__(self, other):
        return not (self == other)

    def __hash__(self):
        return 3*hash(self._ref_target) \
            + 5*hash(self._alias) \
            + 7*hash(self._mod_name)

    def unpack(self):
        return str(self)
        
