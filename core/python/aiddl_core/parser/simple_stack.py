def pop(s):
    r = s[-1]
    del s[-1]
    return r


def peek(s):
    return s[-1]


def push(s, a):
    s.append(a)
