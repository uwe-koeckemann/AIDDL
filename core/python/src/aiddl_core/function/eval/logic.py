from aiddl_core.representation.symbolic import Boolean
from aiddl_core.representation.collection import Collection
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.list import List


class Not:
    def apply(self, x):
        return Boolean.create(not x.bool_value())


class And:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        if x.size() == 0:
            return Boolean.create(True)
        for i in range(1, x.size()):
            if not self.evaluator.apply(x.get(i)).bool_value():
                return Boolean.create(False)
        return Boolean.create(True)


class If:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        if self.evaluator.apply(x.get(0)).bool_value():
            return self.evaluator.apply(x.get(1))
        else:
            return self.evaluator.apply(x.get(2))


class Or:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        if x.size() == 0:
            return Boolean.create(False)
        for i in range(1, x.size()):
            if self.evaluator.apply(x.get(i)).bool_value():
                return Boolean.create(True)
        return Boolean.create(False)


class Cond:
    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        for i in range(1, x.size()):
            if self.evaluator.apply(x.get(i).get_key()).bool_value():
                return self.evaluator.apply(x.get(i))
        raise ValueError("Conditional does not cover all cases.\n"
                         + str(x) + "\n"
                         + "Make sure the existing conditions cover all cases,"
                         + " or add a catch all case (True) : <default-result>"
                         + " at the end to avoid this.")


class Exists:
    ExistsHelp = "Uses format: (exists (x S C)) where x is a term" \
                 + " matched to all elements of collection term s.\n" \
                 + "The resulting terms must satisfy all constraints in set C."

    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        if len(x) != 3:
            raise ValueError(x, ": ", Exists.ExistsHelp)

        matchTerm = x.get(0)
        collTerm = self.evaluator.apply(x.get(1))

        if not isinstance(collTerm, Collection) and \
           not isinstance(collTerm, Tuple):
            raise ValueError(
                x.get(2) + " not a CollectionTerm or TupleTerm "
                + Exists.ExistsHelp)

        if isinstance(collTerm, Collection):
            collection = collTerm
        else:
            list = []
            for i in range(collTerm.size()):
                list.add(collTerm.get(i))
            collection = List(list)

        constraints = x.get(2)

        for t in collection:
            s = matchTerm.match(t)

            if s is None:
                continue

            conSub = constraints.substitute(s)

            if self.evaluator.apply(conSub).bool_value():
                return Boolean.create(True)
        return Boolean.create(False)


class Forall:
    ForallHelp = "Uses format: (forall (x S C)) where x is a term" \
                 + " matched to all elements of collection term S.\n" \
                 + "The resulting terms is then evaluated based on C."

    def __init__(self, evaluator):
        self.evaluator = evaluator

    def apply(self, x):
        if len(x) != 3:
            raise ValueError(x, ":", Forall.ForallHelp)

        matchTerm = x.get(0)
        collTerm = self.evaluator.apply(x.get(1))

        if not isinstance(collTerm, Collection) and \
           not isinstance(collTerm, Tuple):
            raise ValueError(
                x.get(2),
                " not a CollectionTerm or TupleTerm ",
                Forall.ForallHelp)

        if isinstance(collTerm, Collection):
            collection = collTerm
        else:
            list = []
            for i in range(collTerm.size()):
                list.add(collTerm.get(i))
            collection = List(list)

        constraints = x.get(2)

        for t in collection:
            s = matchTerm.match(t)
            if s is None:
                return Boolean.create(False)

            conSub = constraints.substitute(s)

            if not self.evaluator.apply(conSub).bool_value():
                return Boolean.create(False)
        return Boolean.create(True)
