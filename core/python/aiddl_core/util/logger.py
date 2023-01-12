from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.keyval import KeyVal


class Logger:
    depth = 0

    def inc_depth():
        Logger.depth += 1

    def dec_depth():
        Logger.depth -= 1

    def tabbing():
        return "  "*Logger.depth

    def msg(source, msg):
        print("%s[%s] %s" % (Logger.tabbing(), source, str(msg)))

    def pretty_print(t, depth):
        base = str(t)
        if len(base) > 80:
            s = ""
            if isinstance(t, Set):
                s += Logger.simple_tabbing(depth)
                s += "{\n"
                for e in t:
                    s += Logger.pretty_print(e, depth + 1)
                    s += "\n"
                s += Logger.simple_tabbing(depth)
                s += "}\n"
            elif isinstance(t, List):
                s += Logger.simple_tabbing(depth)
                s += "[\n"
                for e in t:
                    s += Logger.pretty_print(e, depth + 1)
                    s += "\n"
                s += Logger.simple_tabbing(depth)
                s += "]\n"
            elif isinstance(t, Tuple):
                s += Logger.simple_tabbing(depth)
                s += "(\n"
                for e in t:
                    s += Logger.pretty_print(e, depth + 1)
                    s += "\n"
                s += Logger.simple_tabbing(depth)
                s += ")\n"
            elif isinstance(t, KeyVal):
                s += Logger.pretty_print(t.key, depth)
                s += ":\n"
                s += Logger.pretty_print(t.value, depth+1)
            else:
                s += Logger.simple_tabbing(depth)
                s += base
            return s
        return Logger.simple_tabbing(depth) + base

    def simple_tabbing(n):
        return "  " * n
