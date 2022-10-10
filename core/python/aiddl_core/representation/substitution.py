import aiddl_core.representation.term as term

class Substitution:

    def __init__(self, m=None):
        if m is None:
            self.m = {}
        else:
            self.m = m

    @staticmethod
    def from_term(t: 'Collection') -> 'Substitution':
        """ Create a substitution from a collection of key-value pairs

        :param t: collection term consisting of key-value pairs
        :return:
        """
        s = Substitution()
        for e in t:
            s.add(e.key, e.value)
        return s

    def add(self, t_from: 'Term', t_to: 'Term') -> bool:
        """ Attempt to add a new element to this substitution. If the replaced term t_from already exists no change
        will occur.

        :param t_from: term to be replaced by substitution
        :param t_to: replacement term
        :return: true if the added element was compatible with this substitution, false otherwise
        """
        assert isinstance(t_from, term.Term)
        assert isinstance(t_to, term.Term)
        if t_from not in self.m.keys():
            self.m[t_from] = t_to
            return True
        else:
            self.m[t_from] == t_to

    def add_substitution(self, sub: 'Substitution') -> bool:
        """ Add another substitution to this one, if it is compatible.

        If sub is incompatible self will not be changed.

        :param sub: a substitution
        :return: true if sub was compatible with self, false otherwise
        """
        for t_from in sub.m.keys():
            if t_from in self.m.keys():
                if self.m[t_from] != sub.m[t_from]:
                    return False
        for t_from in sub.m.keys():
            self.add(t_from, sub.m[t_from])
        return True

    def substitute(self, t: 'Term') -> 'Term':
        """ Substitute a term

        :param t: term to substitute
        :return: the substitution if one exists, t otherwise
        """
        if t in self.m.keys():
            return self.m[t]
        else:
            return t

    def copy(self) -> 'Substitution':
        """ Create a new substitution identical to this one

        :return: a cops of this substitution
        """
        c = Substitution()
        for k in self.m.keys():
            c.add(k, self.m[k])
        return c

    def __len__(self) -> int:
        """ Number of replacements in this substitution

        :return: number of replacements
        """
        return len(self.m)

    def __str__(self):
        s = "{"
        for k in self.m.keys():
            s += str(k)+":"+str(self.m[k])+" "
        s = s[0:-1] + "}"
        return s

    def __repr__(self):
        return str(self)
