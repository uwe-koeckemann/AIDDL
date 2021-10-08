import aiddl_core_path_fix  # noqa
from aiddl_core.representation.int import Int
from aiddl_core.representation.rat import Rat
from aiddl_core.representation.real import Real
from aiddl_core.representation.infinity import Infinity

def test_numerical():
    _l_integer = range(-5, 6)
    for a in _l_integer:
        for b in _l_integer:
            assert Int(a + b) == Int(a) + Int(b)
            assert Int(a - b) == Int(a) - Int(b)
            assert Int(a * b) == Int(a) * Int(b)
            if b != 0:
                assert Int(a / b) == Int(a) / Int(b)

    for a_d in _l_integer:
        for a_n in _l_integer:
            if a_d != 0:
                for b_d in _l_integer:
                    for b_n in _l_integer:
                        if b_d != 0:
                            assert Rat(
                                a_n*b_d + b_n*a_d, a_d*b_d) \
                                   == Rat(a_n, a_d) + Rat(b_n, b_d)
                            assert Rat(
                                a_n*b_d - b_n*a_d, a_d*b_d) \
                                   == Rat(a_n, a_d) - Rat(b_n, b_d)
                            assert Rat(
                                a_n*b_n, a_d*b_d) \
                                   == Rat(a_n, a_d) * Rat(b_n, b_d)
                            if b_n != 0 and a_d != 0:
                                assert Rat(
                                    a_n*b_d, a_d*b_n) \
                                       == Rat(a_n, a_d) / Rat(b_n, b_d)
                                assert Rat(a_n * b_d - ((a_n * b_d) % (a_d * b_n)),
                                           a_d * b_n) == Rat(a_n, a_d) \
                                       // Rat(b_n, b_d)

    assert(Int(2) + Int(3) == Int(5))
    assert(Int(2) - Int(3) == Int(-1))
    assert(Int(2) * Int(3) == Int(6))
    assert(Int(2) // Int(3) == Int(0))

    assert(Int(2) + Rat(2, 5) == Rat(12, 5))
    assert(Int(2) - Rat(12, 5) == Rat(-2, 5))
    assert(Int(2) * Rat(2, 5) == Rat(4, 5))
    assert(Int(2) / Rat(1, 5) == Rat(10, 1))
    assert(Int(2) // Rat(3, 5) == Int(3))

    assert(Int(2) + Real(2.5) == Real(4.5))
    assert(Int(2) - Real(2.5) == Real(-0.5))
    assert(Int(2) * Real(2.5) == Real(5.0))
    assert(Int(2) / Real(0.5) == Real(4.0))
    assert(Int(2) // Real(1.5) == Int(1))

    assert (Int(2) + Infinity.pos()).is_inf_pos
    assert (Int(2) - Infinity.pos()).is_inf_neg
    assert (Int(2) * Infinity.pos()).is_inf_pos
    assert (Int(0) * Infinity.pos()).is_nan
    assert(Int(2) / Infinity.pos() == Int(0))
    assert(Int(2) // Infinity.pos() == Int(0))

    assert(Int(0) >= Int(0))
    assert(Int(0) < Int(1))
    assert(Int(2) < Int(3))
    assert(Int(2) <= Int(2))
    assert(Int(10) > Int(9))
    assert(Int(10) >= Int(10))

    assert(Int(2) < Rat(7, 2))
    assert(Int(2) <= Rat(4, 2))
    assert(Int(10) > Rat(19, 2))
    assert(Int(10) >= Rat(20, 2))

    assert(Int(2) < Real(7.2))
    assert(Int(2) <= Real(2.0))
    assert(Int(10) > Real(9.9))
    assert(Int(10) >= Real(10.0))

    assert(Int(2) < Infinity.pos())
    assert(Int(2) <= Infinity.pos())
    assert(not Int(2) > Infinity.pos())
    assert(not Int(2) >= Infinity.pos())

    assert(Int(2) > Infinity.neg())
    assert(Int(2) >= Infinity.neg())
    assert(not Int(2) < Infinity.neg())
    assert(not Int(2) <= Infinity.neg())

    assert(Rat(2, 3) + Int(3) == Rat(11, 3))
    assert(Rat(3, 4) - Int(3) == Rat(-9, 4))
    assert(Rat(5, 2) * Int(2) == Rat(5, 1))
    assert(Rat(2, 3) / Int(4) == Rat(2, 12))
    assert(Rat(9, 2) // Int(2) == Int(2))

    assert(Rat(2, 3) + Real(3.6) == Real(2 / 3 + 3.6))
    assert(Rat(3, 4) - Real(2.5) == Real(3 / 4 - 2.5))
    assert(Rat(5, 2) * Real(2.0) == Real(5 / 2 * 2.0))
    assert(Rat(2, 3) / Real(4.5) == Real(2 / 3 / 4.5))
    assert(Rat(9, 2) // Real(1.9) == Int(2))

    assert (Rat(2, 1) + Infinity.pos()).is_inf_pos()
    assert (Rat(2, 1) - Infinity.pos()).is_inf_neg()
    assert (Rat(2, 1) * Infinity.pos()).is_inf_pos()
    assert (Rat(0, 1) * Infinity.pos()).is_nan()
    assert (Rat(2, 1) / Infinity.pos()).is_zero()
    assert (Rat(2, 1) // Infinity.pos()).is_zero()

    assert(Rat(7, 2) > Int(2))
    assert(Rat(4, 2) >= Int(2))
    assert(Rat(19, 2) < Int(10))
    assert(Rat(20, 2) <= Int(10))

    assert(Rat(7, 2) > Real(2.4))
    assert(Rat(4, 2) >= Real(2.0))
    assert(Rat(19, 2) < Real(10.3))
    assert(Rat(20, 2) <= Real(10.0))

    assert(Rat(2, 1) < Infinity.pos())
    assert(Rat(2, 1) <= Infinity.pos())
    assert(not Rat(2, 1) > Infinity.pos())
    assert(not Rat(2, 1) >= Infinity.pos())

    assert(Rat(2, 1) > Infinity.neg())
    assert(Rat(2, 1) >= Infinity.neg())
    assert(not Rat(2, 1) < Infinity.neg())
    assert(not Rat(2, 1) <= Infinity.neg())

    assert(Real(2.3) + Int(3) == Real(5.3))
    assert(Real(3.5) - Int(3) == Real(0.5))
    assert(Real(5.2) * Int(2) == Real(10.4))
    assert(Real(2.8) / Int(4) == Real(0.7))
    assert(Real(9.2) // Int(2) == Int(4))

    assert(Real(3.6) + Rat(2, 3) == Real(2 / 3 + 3.6))
    assert(Real(2.5) - Rat(3, 4) == Real(2.5 - 3 / 4))
    assert(Real(2.0) * Rat(5, 2) == Real(5 / 2 * 2.0))
    assert(Real(4.5) / Rat(2, 3) == Real(4.5 / (2 / 3)))
    assert(Real(1.9) // Rat(9, 2) == Int(0))

    assert (Real(2.1) + Infinity.pos()).is_inf_pos()
    assert (Real(2.1) - Infinity.pos()).is_inf_neg()
    assert (Real(2.1) * Infinity.pos()).is_inf_pos()
    assert (Real(0.1) * Infinity.pos()).is_inf_pos()
    assert(Real(2.1) / Infinity.pos() == Real(0.0))
    assert(Real(2.1) // Infinity.pos() == Real(0.0))

    assert(Real(7.2) > Int(2))
    assert(Real(2.0) >= Int(2))
    assert(Real(9.9) < Int(10))
    assert(Real(10.0) <= Int(10))

    assert(Real(2.4) < Rat(7, 2))
    assert(Real(2.0) <= Rat(4, 2))
    assert(Real(10.3) > Rat(19, 2))
    assert(Real(10.0) >= Rat(20, 2))

    assert(Real(2.1) > Infinity.neg())
    assert(Real(2.1) >= Infinity.neg())
    assert(not Real(2.1) < Infinity.neg())
    assert(not Real(2.1) <= Infinity.neg())

    assert(Real(2.1) > Infinity.neg())
    assert(Real(2.1) >= Infinity.neg())
    assert(not Real(2.1) < Infinity.neg())
    assert(not Real(2.1) <= Infinity.neg())

    assert((Infinity.pos() + Infinity.pos()).is_inf_pos())
    assert((Infinity.neg() - Infinity.pos()).is_inf_neg())
    assert((Infinity.pos() - Infinity.neg()).is_inf_pos())
    assert((Infinity.neg() + Infinity.neg()).is_inf_neg())

    assert((Infinity.pos() + Int(2)).is_inf_pos())
    assert((Infinity.pos() - Int(2)).is_inf_pos())
    assert((Infinity.pos() * Int(2)).is_inf_pos())
    assert((Infinity.pos() * Int(0)).is_nan())
    assert((Infinity.pos() / Int(2)).is_inf_pos())
    assert((Infinity.pos() // Int(2)).is_inf_pos())
    assert((Infinity.neg() + Int(2)).is_inf_neg())
    assert((Infinity.neg() - Int(2)).is_inf_neg())
    assert((Infinity.neg() * Int(2)).is_inf_neg())
    assert((Infinity.neg() * Int(0)).is_nan())
    assert((Infinity.neg() / Int(2)).is_inf_neg())
    assert((Infinity.neg() // Int(-2)).is_inf_pos())

    assert((Infinity.pos() + Rat(2, 1)).is_inf_pos())
    assert((Infinity.pos() - Rat(2, 1)).is_inf_pos())
    assert((Infinity.pos() * Rat(2, 1)).is_inf_pos())
    assert (Infinity.pos() * Rat(0, 1)).is_nan()
    assert((Infinity.pos() / Rat(2, 1)).is_inf_pos())
    assert((Infinity.pos() // Rat(2, 1)).is_inf_pos())
    assert((Infinity.neg() + Rat(2, 1)).is_inf_neg())
    assert((Infinity.neg() - Rat(2, 1)).is_inf_neg())
    assert((Infinity.neg() * Rat(2, 1)).is_inf_neg())
    assert (Infinity.neg() * Rat(0, 1)).is_nan()
    assert((Infinity.neg() / Rat(2, 1)).is_inf_neg())
    assert((Infinity.neg() // Rat(-2, 1)).is_inf_pos())

    assert((Infinity.pos() + Real(2.0)).is_inf_pos())
    assert((Infinity.pos() - Real(2.0)).is_inf_pos())
    assert((Infinity.pos() * Real(2.0)).is_inf_pos())
    assert (Infinity.pos() * Real(0.0)).is_nan()
    assert((Infinity.pos() / Real(2.0)).is_inf_pos())
    assert((Infinity.pos() // Real(2.0)).is_inf_pos())
    assert((Infinity.neg() + Real(2.0)).is_inf_neg())
    assert((Infinity.neg() - Real(2.0)).is_inf_neg())
    assert((Infinity.neg() * Real(2.0)).is_inf_neg())
    assert (Infinity.neg() * Real(0.0)).is_nan()
    assert((Infinity.neg() / Real(2.0)).is_inf_neg())
    assert((Infinity.neg() // Real(-2.0)).is_inf_pos())
