import random as r

from aiddl_core.representation.integer import Integer
from aiddl_core.representation.real import Real
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.list import List
from aiddl_core.representation.set import Set


class UniformRealSampler:
    def apply(self, x):
        return Real(r.random())


class UniformIntegerSampler:
    def apply(self, x):
        min = x[0].int_value()
        max = x[1].int_value()
        return Integer(r.randint(min, max-1))


class UniformElementSampler:
    def apply(self, x):
        if isinstance(x, Tuple) or isinstance(x, List):
            idx = r.randint(0, len(x)-1)
            return x[idx]
        elif isinstance(x, Set):
            idx = r.randint(0, len(x)-1)
            for e in x:
                if idx == 0:
                    return e
                idx -= 1
        else:
            raise AttributeError


class NormalDistributionSampler:
    def apply(self, x):
        mean = x.get(0).get_real_value()
        std = x.get(1).get_real_value()
        return Real(r.normalvariate(mean, std))
