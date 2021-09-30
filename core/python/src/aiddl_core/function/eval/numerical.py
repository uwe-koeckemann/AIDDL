from aiddl_core.representation.integer import Integer
from aiddl_core.representation.symbolic import Boolean


class AbsoluteValue:
    def apply(self, x):
        if x < Integer(0):
            return x * Integer(-1)
        return x


class Addition:
    def apply(self, args):
        sum = Integer(0)
        for i in range(0, args.size()):
            sum += args[i]
        return sum


class Subtraction:
    def apply(self, args):
        if args.size() == 0:
            return Integer(0)
        if args.size() == 1:
            return Integer(0) - args[0]
        sum = args[0]
        for i in range(1, args.size()):
            sum -= args[i]
        return sum


class Multiplication:
    def apply(self, args):
        prod = Integer(1)
        for i in range(0, args.size()):
            prod *= args[i]
        return prod


class Division:
    def apply(self, args):
        if args.size() == 0:
            return Integer(1)
        if args.size() == 1:
            return Integer(1) / args[0]
        prod = args[0]
        for i in range(1, args.size()):
            prod /= args[i]
        return prod


class Modulo:
    def apply(self, args):
        return Integer(args.get(0).int_value() % args.get(1).int_value())


class GreaterThan:
    def apply(self, args):
        return Boolean.create(args.get(0) > args.get(1))


class GreaterOrEquals:
    def apply(self, args):
        return Boolean.create(args.get(0) >= args.get(1))


class LessThan:
    def apply(self, args):
        return Boolean.create(args.get(0) < args.get(1))


class LessOrEquals:
    def apply(self, args):
        return Boolean.create(args.get(0) <= args.get(1))


class IsPositive:
    def apply(self, args):
        return Boolean.create(args.is_positive())


class IsNegative:
    def apply(self, args):
        return Boolean.create(args.is_negative())


class IsZero:
    def apply(self, args):
        return Boolean.create(args.is_zero())


class IsInfinite:
    def apply(self, args):
        return Boolean.create(args.is_inf())


class IsInfinitePositive:
    def apply(self, args):
        return Boolean.create(args.is_inf_pos())


class IsInfiniteNegative:
    def apply(self, args):
        return Boolean.create(args.is_inf_neg())


class IsNaN:
    def apply(self, args):
        return Boolean.create(args.is_nan())

