import os
import re
import sys
from pathlib import Path

from aiddl_core import aiddl
from aiddl_core.parser.util import basic_token_to_term, replace_strings_by_placeholders, clean_up_string
from aiddl_core.representation.term import Term
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.str import Str
from aiddl_core.representation.real import Real
from aiddl_core.representation.int import Int
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.entref import EntRef
from aiddl_core.representation.keyval import KeyVal
from aiddl_core.representation.funref import FunRef
from aiddl_core.container.container import Entry

from aiddl_core.parser.simple_stack import pop, peek, push
from aiddl_core.parser.tokens import (OPEN_TUPLE, CLOSE_TUPLE,
                                      OPEN_LIST, CLOSE_LIST,
                                      OPEN_SET, CLOSE_SET,
                                      ASSOC,
                                      REF, SELF_REF, FUNCTION_REF,
                                      SPECIAL)

MOD = Sym("#mod")
REQ = Sym("#req")
NAMES = Sym("#namespace")
NAMES_ALT = Sym("#nms")


class Parser(object):
    def __init__(self, container, aiddl_modules=None, aiddl_folders=None):
        if aiddl_modules is None:
            self.aiddl_modules = []
        else:
            self.aiddl_modules = aiddl_modules

        if aiddl_folders is None:
            self.aiddl_folders = []
        else:
            self.aiddl_folders = aiddl_folders

        self.container = container
        self.aiddl_modules.append(aiddl)
        try:
            from aiddl_common import aiddl as aiddl_common_aiddl
            self.aiddl_modules.append(aiddl_common_aiddl)
        except ModuleNotFoundError:
            pass

        self.aiddl_folders = collect_paths_from_environment(self.aiddl_folders)
        self.collect_paths_from_modules()

    def collect_paths_from_modules(self):
        for module in self.aiddl_modules:
            directory = os.path.dirname(module.__file__)
            self.aiddl_folders.append(directory)

    def parse(self, filename):
        return parse(filename, self.container, root_folders=self.aiddl_folders)

    def parse_term(self, term_string):
        return parse_string(term_string, None, self.container.fun_reg)[0][0]


def parse_string(s, aiddl_paths, function_registry, my_folder='./'):
    s_new, str_lookup = replace_strings_by_placeholders(s)
    s = clean_up_string(s_new)

    stack = []
    module_name = None
    self_ref = None
    local_refs = {}
    tokens = s.split(" ")
    tuple_depth = 0
    for i in range(len(tokens)):
        token = tokens[i].strip()
        if token == CLOSE_LIST:
            assembled_list = []
            current = pop(stack)
            while current != OPEN_LIST:
                assembled_list.append(current)
                current = pop(stack)
            assembled_list.reverse()
            term = List(assembled_list)
            # push(stack, term)
        elif token == CLOSE_TUPLE:
            tuple_depth -= 1
            assembled_list = []
            current = pop(stack)
            while current != OPEN_TUPLE:
                assembled_list.append(current)
                current = pop(stack)
            assembled_list.reverse()
            term = Tuple(assembled_list)
            # push(stack, term)
            if tuple_depth == 0:
                if term.get(0) == MOD:
                    self_ref = term.get(1)
                    module_name = term.get(2)
                elif term.get(0) == REQ or \
                        term.get(0) == NAMES or \
                        term.get(0) == NAMES_ALT:
                    local_refs[term.get(1)] = term.get(2)
                    if isinstance(term.get(2), Str):
                        fname = term.get(2).string
                        req_mod_n = get_mod_name_from_file(my_folder + fname, aiddl_paths)
                        if req_mod_n is not None:
                            local_refs[term.get(1)] = req_mod_n
        elif token == CLOSE_SET:
            assembled_set = []
            current = pop(stack)
            while current != OPEN_SET:
                assembled_set.append(current)
                current = pop(stack)
            term = Set(assembled_set)
        elif token in SPECIAL:
            push(stack, token)
            if token == OPEN_TUPLE:
                tuple_depth += 1
            continue
        else:
            term = basic_token_to_term(token, str_lookup)
        if len(stack) > 0 and i + 1 < len(tokens) and tokens[i + 1] != "@":
            back_resolve = True
            while back_resolve and len(stack) > 0:
                back_resolve = False
                if peek(stack) == SELF_REF:
                    pop(stack)
                    if not isinstance(term, KeyVal):
                        term = EntRef(term, module_name)
                    else:
                        val = term.value
                        key = EntRef(term.key, module_name)
                        term = KeyVal(key, val)
                    back_resolve = True
                elif peek(stack) == REF:
                    pop(stack)
                    name = pop(stack)
                    if not isinstance(name, FunRef):
                        if term == self_ref:
                            # print("Pointing to %s in module %s (aka %s)"
                            #       % (str(name), str(module_name), str(term)))
                            term = EntRef(name, module_name, alias=term)
                        else:
                            # print("Pointing to %s in module %s (aka %s)" % (str(name), str(module_name), str(term)))
                            term = EntRef(name,
                                          local_refs[term],
                                          alias=term)
                    else:
                        bad_fref = name.function_uri
                        if term == self_ref:
                            term = FunRef(bad_fref, function_registry,
                                          module=module_name)
                        else:
                            term = FunRef(bad_fref, function_registry,
                                          module=module_name)
                    back_resolve = True
                elif peek(stack) == FUNCTION_REF:
                    pop(stack)
                    term = FunRef(term, function_registry)
                    back_resolve = True
                elif peek(stack) == ASSOC:
                    pop(stack)
                    key = pop(stack)
                    if isinstance(key, KeyVal):
                        term = KeyVal(key.key,
                                      KeyVal(key.value, term))
                    else:
                        term = KeyVal(key, term)
                    back_resolve = True
        push(stack, term)
    return stack, module_name, self_ref, local_refs


def parse_term(s):
    return parse_string(s, None, None)[0][0]


def to_aiddl(o):
    if isinstance(o, Term):
        return o
    if isinstance(o, str):
        return Str(o)
    elif isinstance(o, int):
        return Int(o)
    elif isinstance(o, float):
        return Real(o)
    elif isinstance(o, list):
        List([to_aiddl(x) for x in o])
    elif isinstance(o, set):
        Set([to_aiddl(x) for x in o])
    else:
        raise ValueError("Not convertable to term:", type(o), o)


def find_and_open_file(module_name, aiddl_paths):
    """
    Filenames may originate from different sources. This function tries all known sources when trying to open a file.
    :param module_name: filename of module as a string or symbolic name of module
    :param aiddl_paths:
    :return:
    """
    if isinstance(module_name, Sym):
        filename = "" + module_name.string.replace(".", "/") + ".aiddl"
    else:
        filename = module_name

    if os.path.isfile(filename):
        f = open(filename, "r")
        return f
    else:
        for path in aiddl_paths:
            fname = str(path) + "/" + str(filename)
            if os.path.isfile(fname):
                f = open(fname, "r")
                return f
        raise ValueError(
            "File: " + str(filename) +
            " not found in current directory or AIDDL_PATH: "
            + str(aiddl_paths))


def get_mod_name_from_file(fname, aiddl_paths):
    f = find_and_open_file(fname, aiddl_paths)
    s = ""
    in_mod_str = False
    found_mod_entry = False
    for line in f.readlines():
        line = re.sub(r";;.*\n", "", line)
        if "#mod" in line:
            in_mod_str = True
        if in_mod_str:
            s += line
            if ")" in line:
                found_mod_entry = True
                f.close()
                break

    if found_mod_entry:
        req_mod_n = s.split("(")[1].split(")")[0].split()[-1]
        return Sym(req_mod_n)
    else:
        return None


# def is_known_module(mod_name):
#     lookup = get_mod_file_lookup(collect_paths_from_environment([]))
#     return mod_name in lookup.keys()


# def get_mod_file_lookup(paths):
#     m = {}
#     for path in paths:
#         for f_path in Path(path).rglob('*.aiddl'):
#             if "#" not in str(f_path):
#                 f_name = f_path.resolve()
#                 mod_name = get_mod_name_from_file(f_name, paths)
#                 if mod_name is not None:
#                     m[mod_name] = f_name
#     return m


def collect_paths_from_environment(paths):
    """
    Add paths from AIDDL_PATH environment variable to the list of paths
    :param paths:
    :return:
    """
    paths = list(paths)
    if 'AIDDL_PATH' in os.environ:
        if "win32" == sys.platform:
            env_path = os.environ['AIDDL_PATH'].split(";")
        else:
            env_path = os.environ['AIDDL_PATH'].split(":")
        new_paths = []
        for path in env_path:
            if path not in new_paths and path != "":
                new_paths.append(path)
        paths += new_paths
    else:
        print("Warning: AIDDL_PATH not set. This means default modules will not be found by their URI. ")
    return paths


def parse(filename, container, root_folders=None):
    if root_folders is None:
        root_folders = []
    #abs_file_path = os.path.abspath(filename)
    mod = parse_internal(filename, container, root_folders=root_folders)
    container.toggle_namespaces(True)
    container.fun_reg.load_def(container)
    container.fun_reg.load_type_functions(container)
    container.fun_reg.load_container_interfaces(container)
    # freg.load_req_python_functions(container)
    return mod


def parse_internal(filename, container, root_folders=None):
    if root_folders is None:
        root_folders = []

    # TODO: Remove once switch to class is complete
    aiddl_paths = collect_paths_from_environment(root_folders)
    directory = os.path.dirname(aiddl.__file__)
    aiddl_paths.append(directory)

    # mod_name_lookup = get_mod_file_lookup(aiddl_paths)
    f = find_and_open_file(filename, aiddl_paths)
    current_folder = os.path.dirname(os.path.abspath(f.name)) + "/"
    # f_current_folder = os.path.dirname(filename)

    s = f.read()
    f.close()
    terms, mod_name, self_ref, local_refs = parse_string(s,
                                                         aiddl_paths,
                                                         container.fun_reg,
                                                         my_folder=current_folder)

    mod_entry = terms[0]
    container.add_module(mod_name)
    container.add_module_alias(mod_name, mod_entry.get(1), mod_name)
    for term in terms[0:]:
        assert isinstance(term, Tuple) and len(term) == 3
        if term.get(0) == REQ or term.get(0) == NAMES or term.get(0) == NAMES_ALT:
            # print("Loading requirement or namespace: ", term)
            # print(os.environ['AIDDL_PATH'])
            fname = None
            if isinstance(term.get(2), Str):
                fname = current_folder + term.get(2).string
            elif isinstance(term.get(2), Sym):
                # print(mod_name_lookup)
                # for s in mod_name_lookup.keys():
                #     print(s, "->", mod_name_lookup[s])
                fname = term.get(2)

            if fname is not None:
                req_mod_name = parse_internal(fname,
                                              container,
                                              root_folders=root_folders)
                container.add_module_alias(mod_name, term.get(1), req_mod_name)
                entry = Entry(term.get(0), term.get(1), req_mod_name)
                container.set_entry(entry, module=mod_name)
            else:
                entry = Entry(term.get(0), term.get(1), term.get(2))
                container.set_entry(entry, module=mod_name)
        else:
            entry = Entry(term.get(0), term.get(1), term.get(2))
            container.set_entry(entry, module=mod_name)
    return mod_name
