import aiddl_core.representation.term as term
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.key_value import KeyValue
from aiddl_core.representation.substitution import Substitution


class List(Collection):
    __slots__ = ["_internal_list", "_internal_map", "_hash"]

    def __init__(self, l):
        internal_list = []
        internal_map = {}
        for t in l:
            internal_list.append(t)
            if isinstance(t, KeyValue):
                internal_map[t.get_key()] = t.get_value()
        internal_list = tuple(internal_list)
        super(term.Term, self).__setattr__("_internal_list", internal_list)
        super(term.Term, self).__setattr__("_internal_map", internal_map)
        super(term.Term, self).__setattr__("_next_id", -1)
        super(term.Term, self).__setattr__("_hash", 3*hash(internal_list))

    def resolve(self, container):
        l_new = []
        change = False
        for e in self._internal_list:
            e_new = e.resolve(container)
            l_new.append(e_new)
            change = change or (e != e_new)
        if not change:
            return self
        else:
            return List(l_new)

    def substitute(self, substitution):
        l_new = []
        change = False
        for e in self._internal_list:
            e_new = e.substitute(substitution)
            l_new.append(e_new)
            change = change or (e != e_new)
        if not change:
            return self
        else:
            return List(l_new)

    def match(self, other):
        if isinstance(other, List):
            sCombined = Substitution()
            if len(self._internal_list) != len(other._internal_list):
                return None
            for i in range(len(self._internal_list)):
                subArg = self._internal_list[i].match(other._internal_list[i])
                if subArg is None:
                    return None
                if not sCombined.add_substitution(subArg):
                    return None
            return sCombined
        return None

    def __iter__(self):
        super(term.Term, self).__setattr__("_next_id", -1)
        return self

    def __next__(self):
        super(term.Term, self).__setattr__("_next_id", self._next_id + 1)
        if self._next_id == len(self._internal_list):
            super(term.Term, self).__setattr__("_next_id", -1)
            raise StopIteration
        else:
            return self._internal_list[self._next_id]

    def append_right(self, v):
        l_new = []
        for t in self._internal_list:
            l_new.append(t)
        l_new.append(v)
        return List(l_new)

    def append_left(self, v):
        l_new = []
        l_new.append(v)
        for t in self._internal_list:
            l_new.append(t)
        return List(l_new)

    def __add__(self, l):
        l_new = []
        for t in self.internal_list:
            l_new.append(t)
        l_new += l
        return List(l_new)

    def add_all(self, l):
        l_new = []
        for t in self.internal_list:
            l_new.append(t)
        l_new += l
        return List(l_new)

    def put(self, key, value):
        l_new = []
        for t in self._internal_list:
            if not isinstance(t, KeyValue) or t.get_key() != key:
                l_new.add(t)
        l_new.add(KeyValue(key, value))
        return List(l_new)

    def put_all(self, values):
        l_new = []
        for k in self._internal_map.keys():
            if not values.contains_key(k):
                l_new.append(KeyValue(k, self.get(k)))
        for t in values:
            if isinstance(t, KeyValue):
                l_new.append(t)
        return List(l_new)

    def __len__(self):
        return len(self._internal_list)

    def size(self):
        return len(self._internal_list)

    def is_empty(self):
        return len(self._internal_list) == 0

    def remove(self, e):
        l_new = []
        for t in self._internal_list:
            if not e == t:
                l_new.append(t)
        return List(l_new)

    def remove_all(self, c):
        l_new = []
        for t in self._internal_list:
            if t not in c:
                l_new.append(t)
        return List(l_new)

    def get(self, n):
        if isinstance(n, int):
            return self._internal_list[n]
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

    def __in__(self, other):
        return self in other

    def __contains__(self, other):
        return other in self._internal_list

    def contains(self, other):
        return self.__contains__(other)

    def contains_all(self, other):
        for v in other:
            if v not in self:
                return False
        return True

    def contains_any(self, other):
        for v in other:
            if v in self:
                return True
        return False

    def contains_key(self, other):
        return other in self._internal_map.keys()

    def __eq__(self, other):
        return isinstance(other, List) \
            and self._internal_list == other._internal_list

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        if len(self._internal_list) == 0:
            return "[]"
        s = "["
        for e in self._internal_list:
            s += str(e) + " "
        return s[0:-1] + "]"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self._hash

    def unpack(self):
        l = []
        for e in self:
            l.append(e.unpack())
        return l
