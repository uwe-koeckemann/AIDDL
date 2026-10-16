import unittest

from aiddl_core.function import FunctionRegistry
from aiddl_core.representation import KeyVal, Sym, Var, Int, Rat, Real, Inf, NaN, Str, Set, List, Tuple, EntRef, FunRef, \
    Term, Boolean, Num, Collection


class TestTerm(unittest.TestCase):
    SYM: Term = Sym("A")
    BOOL: Term = Boolean(False)
    VAR: Term = Var("x")
    NUM: Term = Int(10)
    INT: Term = Int(20)
    RAT: Term = Rat(1, 4)
    REAL: Term = Real(0.25)
    INF: Term = Inf.pos()
    NAN: Term = NaN()
    STR: Term = Str("Hello")
    COL: Term = Set()
    SET: Term = Set()
    LIST: Term = List()
    TUPLE: Term = Tuple()
    KVP: Term = KeyVal(SYM, VAR)
    ENT_REF: Term = EntRef(SYM, SYM, SYM)
    FUN_REF: Term = FunRef(SYM, FunctionRegistry())

    def test_as_sym(self):
        a = TestTerm.SYM.as_sym()
        assert isinstance(a, Sym)
        self.assertRaises(ValueError, TestTerm.VAR.as_sym)

    def test_as_bool(self):
        a = TestTerm.BOOL.as_bool()
        assert isinstance(a, Boolean)
        self.assertRaises(ValueError, TestTerm.NAN.as_bool)

    def test_as_str(self):
        a = TestTerm.STR.as_str()
        assert isinstance(a, Str)
        self.assertRaises(ValueError, TestTerm.NAN.as_str)

    def test_as_var(self):
        a = TestTerm.VAR.as_var()
        assert isinstance(a, Var)
        self.assertRaises(ValueError, TestTerm.NUM.as_var)

    def test_as_num(self):
        assert isinstance(TestTerm.NUM.as_num(), Num)
        assert isinstance(TestTerm.INT.as_num(), Num)
        assert isinstance(TestTerm.REAL.as_num(), Num)
        assert isinstance(TestTerm.RAT.as_num(), Num)
        assert isinstance(TestTerm.NAN.as_num(), Num)
        assert isinstance(TestTerm.INF.as_num(), Num)
        self.assertRaises(ValueError, TestTerm.SET.as_num)

    def test_as_int(self):
        assert isinstance(TestTerm.INT.as_int(), Int)
        self.assertRaises(ValueError, TestTerm.VAR.as_int)

    def test_as_rat(self):
        assert isinstance(TestTerm.RAT.as_rat(), Rat)
        self.assertRaises(ValueError, TestTerm.VAR.as_rat)

    def test_as_real(self):
        assert isinstance(TestTerm.REAL.as_real(), Real)
        self.assertRaises(ValueError, TestTerm.VAR.as_real)

    def test_as_inf(self):
        assert isinstance(TestTerm.INF.as_inf(), Inf)
        self.assertRaises(ValueError, TestTerm.VAR.as_inf)

    def test_as_nan(self):
        assert isinstance(TestTerm.NAN.as_nan(), NaN)
        self.assertRaises(ValueError, TestTerm.VAR.as_nan)

    def test_as_col(self):
        assert isinstance(TestTerm.COL.as_col(), Collection)
        assert isinstance(TestTerm.LIST.as_col(), Collection)
        assert isinstance(TestTerm.SET.as_col(), Collection)
        self.assertRaises(ValueError, TestTerm.VAR.as_col)

    def test_as_set(self):
        assert isinstance(TestTerm.SET.as_set(), Set)
        self.assertRaises(ValueError, TestTerm.VAR.as_set)

    def test_as_list(self):
        assert isinstance(TestTerm.LIST.as_list(), List)
        self.assertRaises(ValueError, TestTerm.VAR.as_list)

    def test_as_tuple(self):
        assert isinstance(TestTerm.TUPLE.as_tuple(), Tuple)
        self.assertRaises(ValueError, TestTerm.VAR.as_tuple)

    def test_as_kvp(self):
        assert isinstance(TestTerm.KVP.as_kvp(), KeyVal)
        self.assertRaises(ValueError, TestTerm.VAR.as_kvp)

    def test_as_ent_ref(self):
        assert isinstance(TestTerm.ENT_REF.as_ent_ref(), EntRef)
        self.assertRaises(ValueError, TestTerm.VAR.as_ent_ref)

    def test_as_fun_ref(self):
        assert isinstance(TestTerm.FUN_REF.as_fun_ref(), FunRef)
        self.assertRaises(ValueError, TestTerm.VAR.as_fun_ref)

    def test_get_term_or_panic(self):
        self.assertRaises(KeyError, lambda : TestTerm.COL.get_or_panic(TestTerm.SYM))