from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.substitution import Substitution

MOD = Symbolic("#mod")

class Module:
    def __init__(self, modID):
        self.modID = modID
        self.data = {}
        self.observerMap = {}

    def get_name(self):
        return self.modID

    def put_entry(self, e):
        self.data[e.get_name()] = e
        if e.get_name() in self.observerMap.keys():
            for obs in self.observerMap[e.get_name()]:
                obs.update(e.getValue())

    def remove_entry(self, e):
        del self.data[e.get_name()]

    def get_entries(self):
        return self.data.values()

    def add_observer(self, entryName, obs):
        if entryName not in self.observerMap.keys():
            self.observerMap.putIfAbsent(entryName, [])
        self.observerMap[entryName] = obs

    def get_namespace_substitution(self):
        name_sub = Substitution()
        for e in self.data.values():
            if not e.get_type() == MOD:
                name_sub.add(e.get_name(), e.get_value())
        return name_sub

    def substitute(self, s):
        subbed_entries = []
        for e in self.data.values():
            subbed_entries.append(e.substitute(s))
        for e in subbed_entries:
            self.put_entry(e)

