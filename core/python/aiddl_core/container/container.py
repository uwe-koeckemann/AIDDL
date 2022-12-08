from typing import Optional

from aiddl_core.representation.sym import Sym
from aiddl_core.representation.substitution import Substitution
from aiddl_core.representation.entref import EntRef
from aiddl_core.representation.tuple import Tuple
from aiddl_core.container.entry import Entry
from aiddl_core.container.module import Module
from aiddl_core.function import EVAL
import aiddl_core.function.default as dfun
from aiddl_core.util.logger import Logger

MOD = Sym("#mod")


class Container:
    def __init__(self):
        """ Create a new container

        """
        self.modules = {}
        self.module_list = []
        self.alias_lookup = {}
        self.self_alias_lookup = {}
        self.working_module = None
        self._fun_reg = dfun.get_default_function_registry(self)

    @property
    def fun_reg(self):
        """ Function registry used by this container

        :return: function registry
        """
        return self._fun_reg

    def eval(self, term):
        """ Evaluate a term

        :param term: term to evaluate
        :return: the term after all it has been fully evaluated
        """
        return self._fun_reg.get_function(EVAL)(term)

    @property
    def evaluator(self):
        """ Get the evaluator registered in the container function registry

        :return: the evaluator
        """
        return self._fun_reg.get_function(EVAL)

    def get_entry(self, name, module=None) -> Optional[Entry]:
        """ Get an entry stored in the container.

        If no module is specified, the working module will be used.

        :param name: name of the entry
        :param module: optional module
        :return: the entry if it exists, None otherwise
        """
        if module is None:
            m = self.modules[self.working_module]
        else:
            if module in self.modules.keys():
                m = self.modules[module]
            else:
                m = None
        if m is None:
            print("Registered Modules:")
            for m_e in self.module_list:
                print(m_e.name)
                print(type(m_e.name))
                print(self.modules[m_e.name] is not None)
                print(m_e.name == module)
            raise AttributeError(
                f"Requesting entry from unknown module{module} with type {type(module)}")
        for e in m.get_entries():
            s = e.name.match(name)
            if s is not None:
                return e.substitute(s)
        return None

    def get_processed_value(self, name, module=None) -> Optional['Term']:
        """ Get entry found under name and return its fully processed value

        If no module is specified, the working module will be used.

        :param name: name of the entry
        :param module: optional module
        :return: processed value or None if entry does not exist
        """
        e = self.get_entry(name, module=module)
        if e is None:
            return e
        else:
            self.eval.set_follow_references(True)
            r = self.eval(e.get_value().resolve(self))
            self.eval.set_follow_references(False)
            return r

    def get_processed_value_or_panic(self, name, module=None):
        """ Get entry found under name and return its fully processed value. Raises an exception if entry does not
        exist.

        If no module is specified, the working module will be used.

        :param name: name of the entry
        :param module: optional module
        :return: processed value
        """
        self.evaluator.set_follow_references(True)
        r = self.eval(self.get_entry(name, module=module).value.resolve(self))
        self.evaluator.set_follow_references(False)
        return r

    def get_matching_entries(self, module_pattern, type_pattern, name_pattern):
        """ Collect all entries with matching modules, types, and names.

        :param module_pattern: term to match to modules
        :param type_pattern: term to match to types
        :param name_pattern: term to match to names
        :return: list of all entries with matching module, type, and name
        """
        r = []
        for m in self.module_list:
            if module_pattern is not None:
                s_base = module_pattern.match(m.name)
            else:
                s_base = Substitution()
            if s_base is not None:
                for e in m.get_entries():
                    s = s_base.copy()

                    if type_pattern is not None:
                        s_tmp = type_pattern.match(e.type)
                        if s_tmp is None or not s.add_substitution(s_tmp):
                            continue
                    if name_pattern is not None:
                        s_tmp = name_pattern.match(e.name)
                        if s_tmp is None or not s.add_substitution(s_tmp):
                            continue
                        else:
                            r.append(e)
                    elif s is not None:
                        r.append(e)
        return r

    def set_entry(self, entry, module=None):
        """ Set an entry in a module.

         If no module is specified, the working module will be used.

        :param entry: the entry
        :param module: optional module
        """
        if module is not None:
            m_set = self.modules[module]
        else:
            m_set = self.modules[self.working_module]

        if m_set is None:
            raise ValueError(f"Module {module} does not exist.")
        if entry.name.resolve(self) != entry.name:
            raise ValueError(f"Entry name {entry.name}"
                             + " has references. Make sure to resolve"
                             + " references before creating the entry.")
        prev = self.get_entry(entry.name, module=module)
        if prev is not None:
            m_set.remove_entry(prev)
        m_set.put_entry(entry)

    def copy_entry(self, a, module_a, b, module_b):
        """ Copy an entry (to another module)

        :param a: name of entry to copy
        :param module_a:  module of entry to copy
        :param b: name of entry copy
        :param module_b: module of entry copy
        """
        a_entry = self.get_entry(a, module=module_a)
        self.set_entry(Entry(a_entry.type,
                             b,
                             a_entry.value),
                       module=module_b)

    def delete_entry(self, entry, module=None):
        """ Delete an entry

        If no module is specified, the working module will be used.

        :param entry: entry to be removed
        :param module: optional module
        """
        if module is not None:
            m_set = self.modules[module]
        else:
            m_set = self.modules[self.working_module]
        if m_set is None:
            raise ValueError(f"Module {module} does not exist.")
        m_set.remove_entry(entry)

    def export(self, module_name, filename):
        """ Export a module to a file.

        :param module_name: name of the module
        :param filename: filename to write to
        """
        m = self.modules.get(module_name)
        if m is None:
            raise ValueError(f"Requesting non-existing module: {module_name}")
        f_out = open(filename, "w")
#        f_out.write("(#mod self " + m.get_name())
#        f_out.write("\n")
        for e in m.get_entries():
            f_out.write("(")
            f_out.write(str(e.type))
            f_out.write("\n\t")
            f_out.write(Logger.pretty_print(e.name, 1))
            f_out.write("\n\t")
            f_out.write(Logger.pretty_print(e.value, 1))
            f_out.write("\n)\n\n")
        f_out.close()

    def resolve_reference(self, t):
        """ Resolve an entry reference by replacing it with its referent until it becomes a non entry reference term.

        :param t:
        :return:
        :raises: ValueError if reference cannot be resolved or resolving references leads to loop
        """
        next_term = t
        seen = []
        while True:
            seen.append(next_term)
            if isinstance(next_term, EntRef):
                next_term = self._resolve_reference_once(next_term)
                if next_term in seen:
                    raise ValueError(f"Infinite reference loop {seen} cannot be fully resolved.")
            else:
                return next_term
            if next_term is None:
                break
        raise ValueError("Reference cannot be resolved:", t)

    def _resolve_reference_once(self, t):
        if isinstance(t.target, EntRef):
            return t.target
        e = self.get_entry(t.target, module=t.module)
        if e is None:
            return None
        return e.value

    def get_working_module(self) -> Sym:
        """ Get the name of the working module

        :return: name of working module
        """
        return self.working_module

    def set_working_module(self, module: Sym):
        """ Set the working module

        :param module: name of working module
        """
        if module not in self.modules.keys():
            raise ValueError(f"Trying use non-existing module {module} as working module.")
        self.working_module = self.modules[module].name

    def add_module(self, name: Sym):
        """ Add a new module.

        The first added module becomes the working module

        :param name: name of module
        """
        if not isinstance(name, Sym):
            raise ValueError(f"Illegal module name: {name}. Name must be of type Sym.")
        m = Module(name)
        if name not in self.modules.keys():
            self.modules[name] = m
            self.module_list.append(m)
        if self.working_module is None:
            self.working_module = m.name

    def remove_module(self, name: Sym):
        """ Remove a module

        :param name: name of module
        """
        m = self.modules[name]
        del self.modules[name]
        self.module_list.remove(m)
        if self.working_module == m:
            self.working_module = None

    def get_module_names(self):
        """ Get list of all module names

        :return: list of module names
        """
        r = []
        for m in self.modules.keys():
            r.append(m)
        return r

    def add_module_alias(self, source_module, alias, target_module):
        """ Add an alias for a module used by another module.

        :param source_module: module that uses the alias
        :param alias: the alias
        :param target_module: module that the alias refers to
        """
        if source_module not in self.alias_lookup.keys():
            self.alias_lookup[source_module] = {}
        self.alias_lookup[source_module][alias] = target_module
        if source_module == target_module:
            self.self_alias_lookup[source_module] = alias

    def resolve_module_alias(self, source_module, alias) -> Optional[Sym]:
        """ Resolve a module alias

        :param source_module: module that uses the alias
        :param alias: the alias
        :return: URI of target module or None if alias does not exist
        """
        if source_module in self.alias_lookup.keys():
            return self.alias_lookup[source_module][alias]
        return None

    def find_self_alias(self, module_name):
        """ Get the name a module uses to refer to itself.

        Specified with the name of the #mod entry in every module.
        For example (#mod self module-uri) has the self alias 'self'.


        :param module_name: name of the module
        :return: self alias
        """
        return self.self_alias_lookup.get(module_name)

    # def register_observer(self, module, entryName, obs):
    #     m = self.modules.get(module)
    #     if m is None:
    #         raise ValueError(
    #             "Trying to add observer to non-existing module: " + module)
    #     m.addObserver(entryName, obs)

    def toggle_namespaces(self, flag):
        """ Toggle namespaces on or off.

        This will lead to substitutions in all entries of modules that use namespaces

        :param flag: true to turn namespace on, false to rever changes made by namespaces
        """
        sub_map = {}
        for m in self.module_list:
            ns_sub = Substitution()
            for e in self.modules[m.name].get_entries():
                if not e.type == MOD:
                    ns_sub.add(e.name, e.value)
                    if not flag:
                        ns_sub = ns_sub.inverse()
                    sub_map[m.name] = ns_sub

        for m in self.module_list:
            namespaces = self.get_matching_entries(m.name,
                                                   Sym("#namespace"),
                                                   None)
            namespaces += self.get_matching_entries(m.name,
                                                    Sym("#nms"),
                                                    None)
            s = Substitution()
            for ns in namespaces:
                ns_term = ns.value

                if isinstance(ns_term, Sym):
                    print(f"[Warning] Deprecated #nms/#namespace usage: {ns}."
                          f" Use (reference to) set of key-values instead.")
                    ns_mod_name = self.resolve_module_alias(m.name,
                                                            ns.name)

                    if ns_mod_name is None:
                        raise ValueError("Could not find alias: %s %s" %
                                         (m.name, ns.name))
                    if not s.add_substitution(sub_map.get(ns_mod_name)):
                        raise ValueError("Namespace conflict in module "
                                         + str(m.name)
                                         + ". Change namespace or use "
                                         + "#req and references.")

                elif isinstance(ns_term, Tuple):
                    print(f"[Warning] Deprecated #nms/#namespace usage: {ns}."
                          f" Use (reference to) set of key-values instead.")
                    ns_name = ns_term[0]
                    ns_table = self.resolve_reference(ns_term[1])
                    ns_sub = Substitution()
                    for entry in ns_table:
                        ns_sub.add(entry[ns_name], entry[0])
                    if not s.add_substitution(ns_sub):
                        print("Namespace term:", ns_term)
                        print("Substitution before:", s)
                        print("Failed to add:", ns_sub)
                        raise ValueError(f"Namespace conflict in module {m.name}. "
                                         f"Change namespace or use #req and references.")
                else:
                    ns_res = ns_term.resolve(self)
                    ns_sub = Substitution.from_term(ns_res)
                    if not s.add_substitution(ns_sub):
                        print("Namespace term:", ns_term)
                        print("Substitution before:", s)
                        print("Failed to add:", ns_sub)
                        raise ValueError(f"Namespace {ns.name} creates conflict in module {m.name}."
                                         f"Change namespace or use #req and references.")

            if len(s) > 0:
                m.substitute(s)

    def __str__(self):
        s = ""
        s += "Entries:\n"
        for m in self.module_list:
            s += "Module: " + str(m._module_uri)
            if m == self.working_module:
                s += " <--- Working Module"
            s += "\n"
            for e in m.get_entries():
                s += "\t"
                s += str(e)
                s += "\n"
        return s
