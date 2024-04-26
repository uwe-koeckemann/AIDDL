import re

from aiddl_core.representation.sym import Sym
from aiddl_core.representation.sym import Boolean
from aiddl_core.representation.str import Str
from aiddl_core.representation.var import Var
from aiddl_core.representation.rat import Rat
from aiddl_core.representation.real import Real
from aiddl_core.representation.int import Int
from aiddl_core.representation.inf import Inf
from aiddl_core.representation.nan import NaN

from aiddl_core.parser.checks import is_int, is_bin, is_hex, is_oct, is_float
from aiddl_core.parser.tokens import INFINITY, NAN, SPECIAL


def replace_strings_by_placeholders(s):
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
            s_new += ' "%d"' % str_id
            current_str_start = None
            str_id += 1
        elif c == '"' and not bs:
            current_str_start = i + 1
        elif current_str_start is None:
            s_new += c

        if c == '\\':
            bs = True
        else:
            bs = False
    return s_new, str_lookup


def clean_up_string(s):
    s = re.sub(r";;.*\n", "", s)
    s = s.replace(",", " ")
    s = s.replace("\n", " ")
    s = s.replace("\t", " ")

    for token in SPECIAL:
        s = s.replace(token, " " + token + " ")
    while "  " in s:
        s = s.replace("  ", " ")
    return s.strip()


def basic_token_to_term(token, str_lookup):
    if token[0] == "?":
        return Var(name=token)
    elif token == "_":
        return Var()
    elif token[0] == '"':
        string = str_lookup[int(token[1:-1])]
        return Str(string)
    elif "/" in token:
        n = token.split("/")[0]
        d = token.split("/")[1]
        return Rat(int(n), int(d))
    elif is_float(token):
        return Real(float(token))
    elif is_int(token):
        return Int(int(token))
    elif is_bin(token):
        return Int(int(token[2:], 2))
    elif is_oct(token):
        return Int(int(token[2:], 8))
    elif is_hex(token):
        return Int(int(token[2:], 16))
    elif token in INFINITY:
        if "-" in token:
            return Inf.neg()
        else:
            return Inf.pos()
    elif token == NAN:
        return NaN()
    elif token == "true":
        return Boolean(True)
    elif token == "false":
        return Boolean(False)
    else:
        return Sym(token)
