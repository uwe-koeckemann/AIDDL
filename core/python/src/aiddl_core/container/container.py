from aiddl_core.representation.term import Term
from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.reference import Reference
from aiddl_core.representation.tuple import Tuple

from aiddl_core.container.entry import Entry
from aiddl_core.container.module import Module

MOD = Symbolic("#mod")


class Container:
    def __init__(self):
        self.modules = {}
        self.moduleList = []
        self.aliasLookup = {}
        self.selfAliasLookup = {}
        self.working_module = None

    def get_entry(self, name, module=None):
        if module is None:
            m = self.modules[self.working_module]
        else:
            if module in self.modules.keys():
                m = self.modules[module]
            else:
                m = None
        if m is None:
            print("Registered Modules:")
            for m_e in self.moduleList:
                print(m_e.get_name())
                print(type(m_e.get_name()))
                print(self.modules[m_e.get_name()] is not None)
                print(m_e.get_name() == module)
            raise AttributeError(
                "Requesting entry from unknown module: "
                + str(module) + " type " + str(type(module)))
        for e in m.get_entries():
            s = e.get_name().match(name)
            if s is not None:
                return e.substitute(s)
        return None

    def get_matching_entries(self, modulePattern, typePattern, namePattern):
        r = []
        for m in self.moduleList:
            s_base = None
            if modulePattern is not None:
                s_base = modulePattern.match(m.get_name())
            else:
                s_base = Substitution()
            if s_base is not None:
                for e in m.get_entries():
                    s = s_base.copy()

                    if typePattern is not None:
                        s_tmp = typePattern.match(e.get_type())
                        if s_tmp is None or not s.add_substitution(s_tmp):
                            continue
                    if namePattern is not None:
                        s_tmp = namePattern.match(e.get_name())
                        if s_tmp is None or not s.add_substitution(s_tmp):
                            continue
                        else:
                            r.append(e)
                    elif s is not None:
                        r.append(e)
        return r

    def set_entry(self, entry, module=None):
        if module is not None:
            mSet = self.modules[module]
        else:
            mSet = self.modules[self.working_module]

        if mSet is None:
            raise ValueError("Module " + str(module) + " does not exist.")
        if entry.get_name().resolve(self) != entry.get_name():
            raise ValueError("Entry name " + entry.get_name()
                             + " has references. Make sure to resolve"
                             + " references before creating the entry.")
        prev = self.get_entry(entry.get_name(), module=module)
        if prev is not None:
            mSet.remove_entry(prev)
        mSet.put_entry(entry)

    def copy_entry(self, a, module_a, b, module_b):
        aEntry = self.get_entry(a, module=module_a)
        self.set_entry(Entry(aEntry.get_type(),
                             b,
                             aEntry.get_value()),
                       module=module_b)

    def delete_entry(self, entry, module=None):
        if module is not None:
            mSet = self.modules[module]
        else:
            mSet = self.modules[self.working_module]
        if mSet is None:
            raise ValueError("Module", module, "does not exist.")
        mSet.remove_entry(entry)

    def export(self, moduleName, filename):
        m = self.modules.get(moduleName)
        if m is None:
            raise ValueError("Requesting non-existing module:", moduleName)
        f_out = open(filename, "w")
#        f_out.write("(#mod self " + m.get_name())
#        f_out.write("\n")
        for e in m.get_entries():
            f_out.write("(")
            f_out.write(str(e.get_type()))
            f_out.write("\n\t")
            f_out.write(str(e.get_name()))
            f_out.write("\n\t")
            f_out.write(str(e.get_value()))
            f_out.write("\n)\n\n")
        f_out.close()

    def resolve_reference(self, t):
        next_term = t
        while True:
            if isinstance(next_term, Reference):
                next_term = self.resolve_reference_once(next_term)
            else:
                return next_term
            if next_term is None:
                break
        raise ValueError("Reference cannot be resolved:", t)

    def resolve_reference_once(self, t):
        target = t.get_ref_target()
        if isinstance(target, Reference):
            return target
        module = t.get_ref_module()
        e = self.get_entry(target, module=module)
        if e is None:
            return None
        return e.get_value()

    def __str__(self):
        s = ""
        s += "Entries:\n"
        for m in self.moduleList:
            s += "Module: " + str(m.modID)
            if m == self.working_module:
                s += " <--- Working Module"
            s += "\n"
            for e in m.get_entries():
                s += "\t"
                s += str(e)
                s += "\n"
        return s

    def get_working_module(self):
        return self.working_module

    def set_working_module(self, module):
        if module not in self.modules.keys():
            raise ValueError("Trying use non-existing module "
                             + module + " as working module.")
        self.working_module = self.modules[module].get_name()

    def add_module(self, name):
        m = Module(name)
        if name not in self.modules.keys():
            self.modules[name] = m
            self.moduleList.append(m)
        if self.working_module is None:
            self.working_module = m.get_name()

    # def add_proxy_module(self, name, nodeUri, receiver, sender):
    #     m = ProxyModule(name, nodeUri, sender, receiver)
    #     if name not in self.modules.keys():
    #         self.modules[name] = m
    #         self.moduleList.append(m)

    #     if self.working_module is None:
    #         self.working_module = m

    #         for e in m.get_entries():
    #             if e.get_type() == Symbolic("#mod"):
    #                 self.add_module_alias(name, e.get_name(), name)
    #             elif e.get_type() == Symbolic("#req"):
    #                 ref_mod = e.get_value()
    #                 if isinstance(ref, StringTerm):
    #                     refMod = Parser.getModuleFromFile(
    #                         refMod.getStringValue())
    #                     self.add_modul
    #                          e_alias(name, e.get_name(), refMod)

    def remove_module(self, name):
        m = self.modules[name]
        del self.modules[name]
        self.moduleList.remove(m)
        if self.working_module == m:
            self.working_module = None

    def get_module_names(self):
        r = []
        for m in self.modules.keys():
            r.append(m)
        return r

    def add_module_alias(self, sourceModule, alias, targetModule):
        if sourceModule not in self.aliasLookup.keys():
            self.aliasLookup[sourceModule] = {}
        self.aliasLookup[sourceModule][alias] = targetModule
        if sourceModule == targetModule:
            self.selfAliasLookup[sourceModule] = alias

    def resolve_module_alias(self, sourceModule, alias):
        if sourceModule in self.aliasLookup.keys():
            return self.aliasLookup[sourceModule][alias]
        return None

    def find_self_alias(self, moduleName):
        return self.selfAliasLookup.get(moduleName)

    # def register_observer(self, module, entryName, obs):
    #     m = self.modules.get(module)
    #     if m is None:
    #         raise ValueError(
    #             "Trying to add observer to non-existing module: " + module)
    #     m.addObserver(entryName, obs)

    def toggle_namespaces(self, flag):
        sub_map = {}
        for m in self.moduleList:
            ns_sub = m.get_namespace_substitution()
            if not flag:
                ns_sub = ns_sub.inverse()
            sub_map[m.get_name()] = ns_sub

        for m in self.moduleList:
            namespaces = self.get_matching_entries(m.get_name(),
                                                   Symbolic("#namespace"),
                                                   None)
            namespaces += self.get_matching_entries(m.get_name(),
                                                    Symbolic("#nms"),
                                                    None)
            s = Substitution()
            for ns in namespaces:
                ns_term = ns.get_value()

                if isinstance(ns_term, Symbolic):
                    ns_mod_name = self.resolve_module_alias(m.get_name(),
                                                            ns.get_name())

                    if ns_mod_name is None:
                        raise ValueError("Could not find alias: %s %s" %
                                         (m.get_name(), ns.get_name()))
                    if not s.add_substitution(sub_map.get(ns_mod_name)):
                        raise ValueError("Namespace conflict in module "
                                         + str(m.get_name())
                                         + ". Change namespace or use "
                                         + "#req and references.")

                elif isinstance(ns_term, Tuple):
                    ns_name = ns_term[0]
                    ns_table = self.resolve_reference(ns_term[1])
                    ns_sub = Substitution()
                    for entry in ns_table:
                        ns_sub.add(entry[ns_name], entry[0])
                    if not s.add_substitution(ns_sub):
                        print("Namespace term:", ns_term)
                        print("Substitution before:", s)
                        print("Failed to add:", ns_sub)
                        raise ValueError("Namespace conflict in module "
                                         + str(m.get_name())
                                         + ". Change namespace or use "
                                         + "#req and references.")
                        
            if len(s) > 0:
                m.substitute(s)


