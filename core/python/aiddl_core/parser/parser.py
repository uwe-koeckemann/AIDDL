import os
import re
import sys
from pathlib import Path

from aiddl_core.representation.term import Term
from aiddl_core.representation.sym import Sym
from aiddl_core.representation.sym import Boolean
from aiddl_core.representation.str import Str
from aiddl_core.representation.var import Var
from aiddl_core.representation.rat import Rat
from aiddl_core.representation.real import Real
from aiddl_core.representation.int import Int
from aiddl_core.representation.inf import Inf
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.entref import EntRef
from aiddl_core.representation.keyval import KeyVal
from aiddl_core.representation.nan import NaN
from aiddl_core.representation.funref import FunRef
from aiddl_core.container.container import Entry


OTUPLE = "("
CTUPLE = ")"

OSET = "{"
CSET = "}"

OLIST = "["
CLIST = "]"

REF = "@"
SREF = "$"
FREF = "^"
ASSOC = ":"

MOD = Sym("#mod")
REQ = Sym("#req")
NAMES = Sym("#namespace")
NAMES_ALT = Sym("#nms")

QMARKS = '"'
ANY = "_"

SPECIAL = [OTUPLE, CTUPLE,
           OSET, CSET,
           OLIST, CLIST,
           ASSOC, REF, SREF, FREF]

INFINITY = ["INF", "+INF", "-INF"]
NAN = "NaN"

REQ = Sym("#req")
MOD = Sym("#mod")


def is_float(s):
    try:
        float(s)
        return "." in s
    except ValueError:
        return False


def is_bin(s):
    if s[0:2] == "#b":
        try:
            tmp = s[2:len(s)]
            int(tmp, 2)
            return True
        except ValueError:
            return False
    return False


def is_hex(s):
    if s[0:2] == "#x":
        try:
            tmp = s[2:len(s)]
            int(tmp, 16)
            return True
        except ValueError:
            return False
    return False


def is_oct(s):
    if s[0:2] == "#o":
        try:
            tmp = s[2:len(s)]
            int(tmp, 8)
            return True
        except ValueError:
            return False
    return False


def is_int(s):
    try:
        int(s)
        return "." not in s and "e" not in s
    except ValueError:
        return False


def pop(s):
    r = s[-1]
    del s[-1]
    return r


def peek(s):
    return s[-1]


def push(s, a):
    s.append(a)


def parse_string(s, aiddl_paths, freg, my_folder='./'):
    str_id = 0
    str_lookup = {}

    bs = False
    current_str_start = None
    slices = []
    s_new = ""
    for i in range(len(s)):
        c = s[i]
        if c == '"' and not bs and current_str_start is not None:
            str_lookup[str_id] = s[current_str_start:i]

            slices.append((current_str_start, i, str(str_id)))
            s_new += ' "%d"' % (str_id)
            current_str_start = None
            str_id += 1
        elif c == '"' and not bs:
            current_str_start = i+1
        elif current_str_start is None:
            s_new += c

        if c == '\\':
            bs = True
        else:
            bs = False

    # for sl in slices:
    #     s = s[:sl[0]] + sl[2] + s[sl[1]:]

    s = s_new

    s = re.sub(r";;.*\n", "", s)
    s = s.replace(",", " ")
    s = s.replace("\n", " ")
    s = s.replace("\t", " ")

    for token in SPECIAL:
        s = s.replace(token, " " + token + " ")
    while "  " in s:
        s = s.replace("  ", " ")
    s = s.strip()
    stack = []
    module_name = None
    self_ref = None
    local_refs = {}
    tokens = s.split(" ")
    tuple_depth = 0
    for i in range(len(tokens)):
        token = tokens[i].strip()
        if token == CLIST:
            assembled_list = []
            current = pop(stack)
            while current != OLIST:
                assembled_list.append(current)
                current = pop(stack)
            assembled_list.reverse()
            term = List(assembled_list)
            # push(stack, term)
        elif token == CTUPLE:
            tuple_depth -= 1
            assembled_list = []
            current = pop(stack)
            while current != OTUPLE:
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
        elif token == CSET:
            assembled_set = []
            current = pop(stack)
            while current != OSET:
                assembled_set.append(current)
                current = pop(stack)
            term = Set(assembled_set)
        elif token in SPECIAL:
            push(stack, token)
            if token == OTUPLE:
                tuple_depth += 1
            continue
        else:
            term = None
            if token[0] == "?":
                term = Var(name=token)
            elif token == "_":
                term = Var()
            elif token[0] == '"':
                string = str_lookup[int(token[1:-1])]
                term = Str(string)
            elif "/" in token:
                n = token.split("/")[0]
                d = token.split("/")[1]
                term = Rat(int(n), int(d))
            elif is_float(token):
                term = Real(float(token))
            elif is_int(token):
                term = Int(int(token))
            elif is_bin(token):
                term = Int(int(token[2:], 2))
            elif is_oct(token):
                term = Int(int(token[2:], 8))
            elif is_hex(token):
                term = Int(int(token[2:], 16))
            elif token in INFINITY:
                if "-" in token:
                    term = Inf.neg()
                else:
                    term = Inf.pos()
            elif token == NAN:
                term = NaN()
            elif token == "true":
                term = Boolean(True)
            elif token == "false":
                term = Boolean(False)
            else:
                term = Sym(token)
        if len(stack) > 0 and i+1 < len(tokens) and tokens[i+1] != "@":
            back_resolve = True
            while back_resolve and len(stack) > 0:
                back_resolve = False
                if peek(stack) == SREF:
                    pop(stack)
                    if not isinstance(term, KeyVal):
                        term = EntRef(term, module_name)
                    else:
                        val = term.get_value()
                        key = EntRef(term.get_key(), module_name)
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
                        bad_fref = name.get_fref()
                        if term == self_ref:
                            term = FunRef(bad_fref, freg,
                                          module=module_name)
                        else:
                            term = FunRef(bad_fref, freg,
                                          module=module_name)
                    back_resolve = True
                elif peek(stack) == FREF:
                    pop(stack)
                    term = FunRef(term, freg)
                    back_resolve = True
                elif peek(stack) == ASSOC:
                    pop(stack)
                    key = pop(stack)
                    if isinstance(key, KeyVal):
                        term = KeyVal(key._key,
                                      KeyVal(key._value, term))
                    else:
                        term = KeyVal(key, term)
                    back_resolve = True
        push(stack, term)
    return (stack, module_name, self_ref, local_refs)


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


def find_and_open_file(filename, aiddl_paths):
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


def is_known_module(mod_name):
    lookup = get_mod_file_lookup(collect_aiddl_paths([]))
    return mod_name in lookup.keys()


def get_mod_file_lookup(paths):
    m = {}
    # if "win32" == sys.platform:
    #     paths += os.environ['AIDDL_PATH'].split(";")
    # else:
    #     paths += os.environ['AIDDL_PATH'].split(":")
    for path in paths:
        for f_path in Path(path).rglob('*.aiddl'):
            if "#" not in str(f_path):
                f_name = f_path.resolve()
                mod_name = get_mod_name_from_file(f_name, paths)
                if mod_name is not None:
                    m[mod_name] = f_name
    return m


def collect_aiddl_paths(paths):
    paths = list(paths)
    if "win32" == sys.platform:
        paths += os.environ['AIDDL_PATH'].split(";")
    else:
        paths += os.environ['AIDDL_PATH'].split(":")
    return paths


# def parse(filename, container, freg, current_folder, root_folders=[]):
#     mod = parse_internal(filename, container, freg, current_folder, root_folders=root_folders)
#     container.toggle_namespaces(True)
#     freg.load_def(container)
#     freg.load_type_functions(container)
#     freg.load_container_interfaces(container)
#     # freg.load_req_python_functions(container)
#     return mod


def parse(filename, container, root_folders=[]):
    abs_file_path = os.path.abspath(filename)
    mod = parse_internal(abs_file_path, container, root_folders=root_folders)
    container.toggle_namespaces(True)
    container._fun_reg.load_def(container)
    container._fun_reg.load_type_functions(container)
    container._fun_reg.load_container_interfaces(container)
    # freg.load_req_python_functions(container)
    return mod


def parse_internal(filename, container, root_folders=[]):
    current_folder = os.path.dirname(os.path.abspath(filename)) + "/"
    # print("Parsing:", filename)
    aiddl_paths = collect_aiddl_paths(root_folders)
    mod_name_lookup = get_mod_file_lookup(aiddl_paths)
    f = find_and_open_file(filename, aiddl_paths)
    # f_current_folder = os.path.dirname(filename)

    s = f.read()
    f.close()
    terms, mod_name, self_ref, local_refs = parse_string(s,
                                                         aiddl_paths,
                                                         container._fun_reg,
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
                fname = mod_name_lookup[term.get(2)]

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
