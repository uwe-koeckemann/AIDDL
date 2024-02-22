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
