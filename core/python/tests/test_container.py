import os
from aiddl_core.container.container import Container
from aiddl_core.container.container import Entry
from aiddl_core.parser import parser
from aiddl_core.representation.funref import FunRef
from aiddl_core.representation.key_value import KeyValue

from aiddl_core.representation.sym import Sym
from aiddl_core.representation.int import Int
from aiddl_core.representation.tuple import Tuple
from aiddl_core.representation.list import List
from aiddl_core.representation.set import Set
from aiddl_core.representation.var import Var

import aiddl_core.function.default as dfun

def test_container():
    my_abs_folder = os.path.dirname(os.path.abspath(__file__))

    test_file = my_abs_folder + "/../../test/example-module.aiddl"
    # print("MY FOLDER:", os.path.dirname(__file__))
    print("TEST FILE:", test_file)
    # print("ABS FILE:", )
    # print("CUR FOLDER:", os.path.dirname(os.path.abspath(".")) + "/") 
    # current_folder = os.path.dirname(os.path.abspath(test_file)) + "/"
 
    C = Container()
    freg = dfun.get_default_function_registry(C)

    test_mod = parser.parse(test_file, C, freg)

    e = C.get_entry(Sym("a"))

    assert(e.get_type() == Sym("org.aiddl.type.term.numerical.integer"))
    assert(e.get_name() == Sym("a"))
    assert(e.get_value() == Int(1))

    E = C.get_matching_entries(Var("?M"),
                               Var("?T"),
                               Tuple([Sym("X"), Var("_")]))

    assert(len(E) == 5)

    e = Entry(Sym("t"), Sym("a"), List([Int(1), Sym("x")]))

    C.set_entry(e, module=test_mod)

    e_c = C.get_entry(Sym("a"))

    assert(e == e_c)

    e = C.get_entry(Sym("SR"))
    t_c = Set([Sym("c"), Sym("d"), Sym("e")])
    ref = e.get_value()
    res = C.resolve_reference(ref)

    assert(t_c == res)

    req_mod = Sym("org.aiddl.test.example-module-req")

    C.copy_entry(Sym("r"),
                 test_mod,
                 Sym("r_c"),
                 req_mod)

    e_1 = C.get_entry(Sym("r"), module=test_mod)
    e_2 = C.get_entry(Sym("r_c"),
                      module=req_mod)

    assert(e_1.get_value() == e_2.get_value())

    assert(C.get_working_module() == test_mod)
    C.set_working_module(req_mod)
    assert(C.get_working_module() == req_mod)

    e_3 = C.get_entry(Sym("r_c"))
    assert(e_2 == e_3)

    E = C.get_matching_entries(Var("_"), Var("_"), Sym("r_c"))
    assert(len(E) == 1)
    C.delete_entry(e_3)
    E = C.get_matching_entries(Var("_"), Var("_"), Sym("r_c"))
    assert(len(E) == 0)

    new_mod = Sym("my-new-module")
    C.add_module(new_mod)
    E = C.get_matching_entries(new_mod, Var("_"), Var("_"))
    assert(len(E) == 0)
    C.set_entry(e_1, module=new_mod)
    E = C.get_matching_entries(new_mod, Var("_"), Var("_"))
    assert(len(E) == 1)
    assert(e_1 in E)

    kvpref = C.get_entry(Sym("KvpRef"), module=test_mod).get_value()
    assert kvpref is not None
    assert isinstance(kvpref, KeyValue)
    assert isinstance(kvpref.get_key(), Sym)
    assert isinstance(kvpref.get_value(), FunRef)

test_container()