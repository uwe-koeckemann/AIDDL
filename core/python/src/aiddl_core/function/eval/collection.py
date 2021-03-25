from aiddl_core.representation.term import Term
from aiddl_core.representation.symbolic import Boolean
from aiddl_core.representation.integer import Integer
from aiddl_core.representation.numerical import Numerical
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.list import List
from aiddl_core.representation.set import Set


class Add:
    def apply(self, args: Tuple) -> Collection:
        C = args[0]
        e = args[1]
        return C.add(e)


class AddAll:
    def apply(self, args: Tuple) -> Collection:
        C1 = args[0]
        C2 = args[1]
        return C1.add_all(C2)


class Remove:
    def apply(self, args: Tuple) -> Collection:
        C = args[0]
        e = args[1]
        return C.remove(e)


class RemoveAll:
    def apply(self, args: Tuple) -> Collection:
        C1 = args[0]
        C2 = args[1]
        return C1.remove_all(C2)


class InCollection:
    def apply(self, args: Tuple) -> Boolean:
        e = args[0]
        C = args[1]
        return Boolean(C.contains(e))


class Contains:
    def apply(self, args: Tuple) -> Boolean:
        C = args[0]
        e = args[1]
        return Boolean(C.contains(e))


class ContainsMatch:
    def apply(self, args: Tuple) -> Boolean:
        C = args[0]
        e = args[1]
        for e_c in C:
            if e.match(e_c) is not None:
                return Boolean(True)
        return Boolean(False)


class ContainsAll:
    def apply(self, args: Tuple) -> Boolean:
        C1 = args[0]
        C2 = args[1]
        return Boolean(C1.contains_all(C2))


class ContainsAny:
    def apply(self, args: Tuple) -> Boolean:
        C1 = args[0]
        C2 = args[1]
        for e in C2:
            if e in C1:
                return Boolean(True)
        return Boolean(False)


class Size:
    def apply(self, args: Term) -> Integer:
        return args.size()


class Sum:
    def apply(self, C: Collection) -> Numerical:
        s = Integer(0)
        for e in C:
            s += e
        return s


class Union:
    def apply(self, x: Collection) -> Set:
        U = set()
        for s in x:
            for e in s:
                U.add(e)
        return Set(U)


class Concat:
    def apply(self, x):
        L = []
        for s in x:
            for e in s:
                L.append(e)
        return List(L)
