import aiddl_core.representation.term as term
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.key_value import KeyValue


class Set(Collection):
    __slots__ = ["_internal_set", "_internal_map"]

    def __init__(self, l):
        internal_set = set()
        internal_map = {}
        for t in l:
            internal_set.add(t)
            if isinstance(t, KeyValue):
                internal_map[t.get_key()] = t.get_value()
        super(term.Term, self).__setattr__("_internal_set", internal_set)
        super(term.Term, self).__setattr__("_internal_map", internal_map)
        super(term.Term, self).__setattr__("_hash", hash(tuple(internal_set)))

    def resolve(self, container):
        l_new = []
        change = False
        for e in self._internal_set:
            e_new = e.resolve(container)
            l_new.append(e_new)
            change = change or (e != e_new)
        if not change:
            return self
        else:
            return Set(l_new)

    def substitute(self, substitution):
        l_new = []
        change = False
        for e in self._internal_set:
            e_new = e.substitute(substitution)
            l_new.append(e_new)
            change = change or (e != e_new)
        if not change:
            return self
        else:
            return Set(l_new)

    def match(self, other):
        raise ValueError("Set term cannot be matched.")

    def __iter__(self):
        return self._internal_set.__iter__()

    def __next__(self):
        pass

    def add(self, value):
        l_new = set()
        for t in self._internal_set:
            l_new.add(t)
        l_new.add(value)
        return Set(l_new)

    def __add__(self, S):
        l_new = set()
        for t in self._internal_set:
            l_new.add(t)
        l_new += S
        return Set(l_new)

    def add_all(self, S):
        l_new = set()
        for t in self._internal_set:
            l_new.add(t)
        for e in S:
            l_new.add(e)
        return Set(l_new)

    def put(self, key, value):
        l_new = set()
        for t in self._internal_set:
            if not isinstance(t, KeyValue) or t.get_key() != key:
                l_new.add(t)
        l_new.add(KeyValue(key, value))
        return Set(l_new)

    def put_all(self, values):
        l_new = set()
        for k in self._internal_map.keys():
            if not values.contains_key(k):
                l_new.add(KeyValue(k, self.get(k)))
        for t in values:
            if isinstance(t, KeyValue):
                l_new.add(t)
        return Set(l_new)

    def __len__(self):
        return len(self._internal_set)

    def size(self):
        return len(self._internal_set)

    def is_empty(self):
        return len(self._internal_set) == 0

    def remove(self, e):
        l_new = []
        for t in self._internal_set:
            if not e == t:
                l_new.append(t)
        return Set(l_new)

    def remove_all(self, c):
        l_new = []
        for t in c:
            if t not in c:
                l_new.append(t)
        return Set(l_new)

    def get(self, n):
        if n in self._internal_map.keys():
            return self._internal_map[n]
        return None

    def get_or_default(self, n, default):
        if isinstance(n, int):
            return self._internal_list[n]
        if n in self._internal_map.keys():
            return self._internal_map[n]
        return default

    def get_or_panic(self, n):
        if isinstance(n, int):
            return self._internal_list[n]
        if n in self._internal_map.keys():
            return self._internal_map[n]
        raise AttributeError("Key not found:", n, "in", self)

    def is_unique_map(self):
        seen = set()
        for e in self._internal_set:
            if not isinstance(e, KeyValue) or e.get_key() in seen:
                return False
            seen.add(e.get_key())
        return True

    def __in__(self, other):
        return self in other

    def __contains__(self, other):
        return other in self._internal_set

    def contains(self, other):
        return other in self

    def contains_all(self, other):
        for t in other:
            if t not in self:
                return False
        return True

    def contains_any(self, other):
        for t in other:
            if t in self:
                return True
        return False

    def contains_key(self, other):
        return other in self._internal_map.keys()

    def __eq__(self, other):
        return isinstance(other, Set) \
            and self._internal_set == other._internal_set

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        if len(self._internal_set) == 0:
            return "{}"
        s = "{"
        for e in self._internal_set:
            s += str(e) + " "
        return s[0:-1] + "}"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self._hash

    def unpack(self):
        s = set()
        for e in self:
            s.add(e.unpack())
        return s
