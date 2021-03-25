import os
import re
import sys
from pathlib import Path

from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.string import String
from aiddl_core.representation.variable import Variable
from aiddl_core.representation.rational import Rational
from aiddl_core.representation.real import Real
from aiddl_core.representation.integer import Integer
from aiddl_core.representation.infinity import Infinity
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.set import Set
from aiddl_core.representation.list import List
from aiddl_core.representation.reference import Reference
from aiddl_core.representation.key_value import KeyValue
from aiddl_core.representation.function_reference import FunctionReference

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

MOD = Symbolic("#mod")
REQ = Symbolic("#req")
NAMES = Symbolic("#namespace")
NAMES_ALT = Symbolic("#nms")

QMARKS = '"'
ANY = "_"

SPECIAL = [OTUPLE, CTUPLE,
           OSET, CSET,
           OLIST, CLIST,
           ASSOC, REF, SREF, FREF]

INFINITY = ["INF", "+INF", "-INF"]


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


def parse_string(s, aiddl_paths, freg):
    str_id = 0
    str_lookup = {}

    current_str_start = None
    slices = []
    s_new = ""
    for i in range(len(s)):
        c = s[i]
        if c == '"' and current_str_start is not None:
            str_lookup[str_id] = s[current_str_start:i]
            slices.append((current_str_start, i, str(str_id)))
            s_new += ' "%d"' % (str_id)
            current_str_start = None
            str_id += 1
        elif c == '"':
            current_str_start = i+1
        elif current_str_start is None:
            s_new += c

    # for sl in slices:
    #     s = s[:sl[0]] + sl[2] + s[sl[1]:]

    s = s_new

    s = re.sub(r";;.*\n", "", s)
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
    for token in tokens:
        token = token.strip()
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
                    if isinstance(term.get(2), String):
                        fname = term.get(2).get_string_value()
                        req_mod_n = get_mod_name_from_file(fname, aiddl_paths)
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
                term = Variable(name=token)
            elif token == "_":
                term = Variable()
            elif token[0] == '"':
                string = str_lookup[int(token[1:-1])]
                term = String(string)
            elif "/" in token:
                n = token.split("/")[0]
                d = token.split("/")[1]
                term = Rational(int(n), int(d))
            elif is_float(token):
                term = Real(float(token))
            elif is_int(token):
                term = Integer(int(token))
            elif is_bin(token):
                term = Integer(int(token[2:], 2))
            elif is_oct(token):
                term = Integer(int(token[2:], 8))
            elif is_hex(token):
                term = Integer(int(token[2:], 16))
            elif token in INFINITY:
                if "-" in token:
                    term = Infinity.neg()
                else:
                    term = Infinity.pos()
            else:
                term = Symbolic(token)
        if len(stack) > 0:
            back_resolve = True
            while back_resolve and len(stack) > 0:
                back_resolve = False
                if peek(stack) == SREF:
                    pop(stack)
                    term = Reference(term, module_name)
                    back_resolve = True
                elif peek(stack) == REF:
                    pop(stack)
                    name = pop(stack)
                    if not isinstance(name, FunctionReference):
                        if term == self_ref:
                            # print("Pointing to %s in module %s (aka %s)"
                            #       % (str(name), str(module_name), str(term)))
                            term = Reference(name, module_name, alias=term)
                        else:
                            # print("Pointing to %s in module %s (aka %s)"
                            #       % (str(name), str(module_name), str(term)))
                            term = Reference(name,
                                             local_refs[term],
                                             alias=term)
                    else:
                        bad_fref = name.get_fref()
                        if term == self_ref:
                            term = FunctionReference(bad_fref, freg,
                                                     module=module_name)
                        else:
                            term = FunctionReference(bad_fref, freg,
                                                     module=module_name)
                    back_resolve = True
                elif peek(stack) == FREF:
                    pop(stack)
                    term = FunctionReference(term, freg)
                    back_resolve = True
                elif peek(stack) == ASSOC:
                    pop(stack)
                    key = pop(stack)
                    if isinstance(key, KeyValue):
                        term = KeyValue(key._key,
                                        KeyValue(key._value, term))
                    else:
                        term = KeyValue(key, term)
                    back_resolve = True
        push(stack, term)
    return (stack, module_name, self_ref, local_refs)


def parse_term(s):
    return parse_string(s, None, None)[0][0]


REQ = Symbolic("#req")
MOD = Symbolic("#mod")


def find_and_open_file(filename, aiddl_paths):
    if os.path.isfile(filename):
        f = open(filename, "r")
        return f
    else:
        for path in aiddl_paths:
            fname = path + "/" + filename
            if os.path.isfile(fname):
                f = open(fname, "r")
                return f
        raise ValueError(
            "File: " + filename +
            " not found in current directory or AIDDL_PATH: "
            + str(aiddl_paths))


def get_mod_name_from_file(fname, aiddl_paths):
    f = find_and_open_file(fname, aiddl_paths)
    s = ""
    for l in f.readlines():
        s += l
        if ")" in l:
            f.close()
            break
    req_mod_n = s.split("(")[1].split(")")[0].split()[-1]
    return Symbolic(req_mod_n)


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
            f_name = f_path.resolve()
            mod_name = get_mod_name_from_file(f_name, paths)
            m[mod_name] = f_name
    return m


def collect_aiddl_paths(paths):
    if "win32" == sys.platform:
        paths += os.environ['AIDDL_PATH'].split(";")
    else:
        paths += os.environ['AIDDL_PATH'].split(":")
    return paths


def parse(filename, container, freg, current_folder):
    mod = parse_internal(filename, container, freg, current_folder)
    container.toggle_namespaces(True)
    freg.load_def(container)
    freg.load_type_functions(container)
    freg.load_container_interfaces(container)
    freg.load_req_python_functions(container)
    return mod


def parse_internal(filename, container, freg, current_folder):
    aiddl_paths = collect_aiddl_paths([current_folder])
    mod_name_lookup = get_mod_file_lookup(aiddl_paths)
    f = find_and_open_file(filename, aiddl_paths)
    f_current_folder = os.path.dirname(filename)
    s = f.read()
    f.close()
    terms, mod_name, self_ref, local_refs = parse_string(s,
                                                         aiddl_paths,
                                                         freg)

    modEntry = terms[0]
    container.add_module(mod_name)
    container.add_module_alias(mod_name, modEntry.get(1), mod_name)
    for term in terms[0:]:
        assert isinstance(term, Tuple) and len(term) == 3
        if term.get(0) == REQ or term.get(0) == NAMES or term.get(0) == NAMES_ALT:
            # print("Loading requirement or namespace: ", term)
            # print(os.environ['AIDDL_PATH'])
            fname = None
            if isinstance(term.get(2), String):
                fname = term.get(2).get_string_value()
            elif isinstance(term.get(2), Symbolic):
                # print(mod_name_lookup)
                # for s in mod_name_lookup.keys():
                #     print(s, "->", mod_name_lookup[s])
                fname = mod_name_lookup[term.get(2)]
            if fname is not None:
                req_mod_name = parse_internal(fname, container, freg, f_current_folder)
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
