from aiddl_core.representation.int import Int


class AbsoluteValue:
    def __call__(self, x):
        if x.is_negative():
            return x * Int(-1)
        return x


class Addition:
    def __call__(self, args):
        s = Int(0)
        for i in range(0, args.size()):
            s += args[i]
        return s


class Subtraction:
    def __call__(self, args):
        if args.size() == 0:
            return Int(0)
        if args.size() == 1:
            return Int(0) - args[0]
        s = args[0]
        for i in range(1, args.size()):
            s -= args[i]
        return s


class Multiplication:
    def __call__(self, args):
        prod = Int(1)
        for i in range(0, args.size()):
            prod *= args[i]
        return prod


class Division:
    def __call__(self, args):
        if args.size() == 0:
            return Int(1)
        if args.size() == 1:
            return Int(1) / args[0]
        prod = args[0]
        for i in range(1, args.size()):
            prod /= args[i]
        return prod


class Modulo:
    def __call__(self, args):
        return Int(args.get(0).int_value % args.get(1).int_value)
