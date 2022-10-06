import aiddl_core.representation.term as term

class Substitution:

    def is_self_referential(t_from, t_to):
        s = Substitution()
        s.add(t_from, Symbolic("--NIL--"))
        return t_to.substitute(s) != t_to

    def __init__(self, m=None):
        if m is None:
            self.m = {}
        else:
            self.m = m

    def from_term(t):
        s = Substitution()
        for e in t:
            s.add(e.get_key(), e.get_value())
        return s

    def add(self, t_from, t_to):
        if not self.causes_loop(t_from, t_to):
            self.m[t_from] = t_to
            return True
        return False

    def substitute(self, t):
        if t in self.m.keys():
            return self.m[t]
        else:
            return t

    def add_substitution(self, sub):
        for t_from in sub.m:
            t_to = sub.m[t_from]
            if self.causes_loop(t_from, t_to):
                return False
        for t_from in sub.m:
            t_to = sub.m[t_from]
            self.add(t_from, t_to)
        return True

    def causes_loop(self, t_from, t_to):
        reachable = set()
        reachable.add(t_to)
        size = -1
        while size != len(reachable):
            size = len(reachable)
            for t in reachable:
                if t in self.m.keys():
                    reachable.add(self.m[t])
                    if t_from in reachable:
                        return True
        return False

    def copy(self):
        c = Substitution()
        for k in self.m.keys():
            c.add(k, self.m[k])
        return c

    def __len__(self):
        return len(self.m)

    def __str__(self):
        s = "{"
        for k in self.m.keys():
            s += str(k)+":"+str(self.m[k])+" "
        s = s[0:-1] + "}"
        return s

    def __repr__(self):
        return str(self)
