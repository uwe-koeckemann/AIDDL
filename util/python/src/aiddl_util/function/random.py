import random as r

from aiddl_core.representation.int import Int
from aiddl_core.representation.real import Real
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.list import List
from aiddl_core.representation.set import Set


class UniformRealSampler:
    def __call__(self, x):
        return Real(r.random())


class UniformIntegerSampler:
    def __call__(self, x):
        rmin = x[0].int_value()
        rmax = x[1].int_value()
        return Int(r.randint(rmin, rmax-1))


class UniformElementSampler:
    def __call__(self, x):
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
    def __call__(self, x):
        mean = x.get(0).get_real_value()
        std = x.get(1).get_real_value()
        return Real(r.normalvariate(mean, std))
