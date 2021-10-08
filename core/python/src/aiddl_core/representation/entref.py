import aiddl_core.representation.term as term


class EntRef(term.Term):
    __slots__ = ["_ref_target", "_alias", "_mod_name"]

    def __init__(self, ref_target, mod_name, alias=None):
        super(term.Term, self).__setattr__("_ref_target", ref_target)
        super(term.Term, self).__setattr__("_mod_name", mod_name)
        super(term.Term, self).__setattr__("_alias", alias)

    def get_ref_target(self):
        return self._ref_target

    def get_ref_module(self):
        return self._mod_name

    def resolve(self, container):
        return container.resolve_reference(self)

    def convert2uri(self):
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
        
