import aiddl_core_path_fix  # noqa
from aiddl_core.representation.integer import Integer
from aiddl_core.representation.rational import Rational
from aiddl_core.representation.real import Real
from aiddl_core.representation.infinity import Infinity

def test_numerical():
    _l_integer = range(-5, 6)
    for a in _l_integer:
        for b in _l_integer:
            assert Integer(a+b) == Integer(a) + Integer(b)
            assert Integer(a-b) == Integer(a) - Integer(b)
            assert Integer(a*b) == Integer(a) * Integer(b)
            if b != 0:
                assert Integer(a/b) == Integer(a) / Integer(b)

    for a_d in _l_integer:
        for a_n in _l_integer:
            if a_d != 0:
                for b_d in _l_integer:
                    for b_n in _l_integer:
                        if b_d != 0:
                            assert Rational(
                                a_n*b_d + b_n*a_d, a_d*b_d) \
                                == Rational(a_n, a_d) + Rational(b_n, b_d)
                            assert Rational(
                                a_n*b_d - b_n*a_d, a_d*b_d) \
                                == Rational(a_n, a_d) - Rational(b_n, b_d)
                            assert Rational(
                                a_n*b_n, a_d*b_d) \
                                == Rational(a_n, a_d) * Rational(b_n, b_d)
                            if b_n != 0 and a_d != 0:
                                assert Rational(
                                    a_n*b_d, a_d*b_n) \
                                    == Rational(a_n, a_d) / Rational(b_n, b_d)
                                assert Rational(a_n*b_d - ((a_n*b_d) % (a_d*b_n)),
                                                a_d*b_n) == Rational(a_n, a_d) \
                                    // Rational(b_n, b_d)

    assert(Integer(2)+Integer(3) == Integer(5))
    assert(Integer(2)-Integer(3) == Integer(-1))
    assert(Integer(2)*Integer(3) == Integer(6))
    assert(Integer(2)//Integer(3) == Integer(0))

    assert(Integer(2) + Rational(2, 5) == Rational(12, 5))
    assert(Integer(2) - Rational(12, 5) == Rational(-2, 5))
    assert(Integer(2) * Rational(2, 5) == Rational(4, 5))
    assert(Integer(2) / Rational(1, 5) == Rational(10, 1))
    assert(Integer(2) // Rational(3, 5) == Integer(3))

    assert(Integer(2) + Real(2.5) == Real(4.5))
    assert(Integer(2) - Real(2.5) == Real(-0.5))
    assert(Integer(2) * Real(2.5) == Real(5.0))
    assert(Integer(2) / Real(0.5) == Real(4.0))
    assert(Integer(2) // Real(1.5) == Integer(1))

    assert(Integer(2) + Infinity.pos() == Infinity.pos())
    assert(Integer(2) - Infinity.pos() == Infinity.neg())
    assert(Integer(2) * Infinity.pos() == Infinity.pos())
    assert(Integer(0) * Infinity.pos() == Integer(0))
    assert(Integer(2) / Infinity.pos() == Integer(0))
    assert(Integer(2) // Infinity.pos() == Integer(0))

    assert(Integer(0) >= Integer(0))
    assert(Integer(0) < Integer(1))
    assert(Integer(2) < Integer(3))
    assert(Integer(2) <= Integer(2))
    assert(Integer(10) > Integer(9))
    assert(Integer(10) >= Integer(10))

    assert(Integer(2) < Rational(7, 2))
    assert(Integer(2) <= Rational(4, 2))
    assert(Integer(10) > Rational(19, 2))
    assert(Integer(10) >= Rational(20, 2))

    assert(Integer(2) < Real(7.2))
    assert(Integer(2) <= Real(2.0))
    assert(Integer(10) > Real(9.9))
    assert(Integer(10) >= Real(10.0))

    assert(Integer(2) < Infinity.pos())
    assert(Integer(2) <= Infinity.pos())
    assert(not Integer(2) > Infinity.pos())
    assert(not Integer(2) >= Infinity.pos())

    assert(Integer(2) > Infinity.neg())
    assert(Integer(2) >= Infinity.neg())
    assert(not Integer(2) < Infinity.neg())
    assert(not Integer(2) <= Infinity.neg())

    assert(Rational(2, 3) + Integer(3) == Rational(11, 3))
    assert(Rational(3, 4) - Integer(3) == Rational(-9, 4))
    assert(Rational(5, 2) * Integer(2) == Rational(5, 1))
    assert(Rational(2, 3) / Integer(4) == Rational(2, 12))
    assert(Rational(9, 2) // Integer(2) == Integer(2))

    assert(Rational(2, 3) + Real(3.6) == Real(2/3+3.6))
    assert(Rational(3, 4) - Real(2.5) == Real(3/4-2.5))
    assert(Rational(5, 2) * Real(2.0) == Real(5/2*2.0))
    assert(Rational(2, 3) / Real(4.5) == Real(2/3/4.5))
    assert(Rational(9, 2) // Real(1.9) == Integer(2))

    assert(Rational(2, 1) + Infinity.pos() == Infinity.pos())
    assert(Rational(2, 1) - Infinity.pos() == Infinity.neg())
    assert(Rational(2, 1) * Infinity.pos() == Infinity.pos())
    assert(Rational(0, 1) * Infinity.pos() == Rational(0, 1))
    assert(Rational(2, 1) / Infinity.pos() == Rational(0, 1))
    assert(Rational(2, 1) // Infinity.pos() == Rational(0, 1))

    assert(Rational(7, 2) > Integer(2))
    assert(Rational(4, 2) >= Integer(2))
    assert(Rational(19, 2) < Integer(10))
    assert(Rational(20, 2) <= Integer(10))

    assert(Rational(7, 2) > Real(2.4))
    assert(Rational(4, 2) >= Real(2.0))
    assert(Rational(19, 2) < Real(10.3))
    assert(Rational(20, 2) <= Real(10.0))

    assert(Rational(2, 1) < Infinity.pos())
    assert(Rational(2, 1) <= Infinity.pos())
    assert(not Rational(2, 1) > Infinity.pos())
    assert(not Rational(2, 1) >= Infinity.pos())

    assert(Rational(2, 1) > Infinity.neg())
    assert(Rational(2, 1) >= Infinity.neg())
    assert(not Rational(2, 1) < Infinity.neg())
    assert(not Rational(2, 1) <= Infinity.neg())

    assert(Real(2.3) + Integer(3) == Real(5.3))
    assert(Real(3.5) - Integer(3) == Real(0.5))
    assert(Real(5.2) * Integer(2) == Real(10.4))
    assert(Real(2.8) / Integer(4) == Real(0.7))
    assert(Real(9.2) // Integer(2) == Integer(4))

    assert(Real(3.6) + Rational(2, 3) == Real(2/3+3.6))
    assert(Real(2.5) - Rational(3, 4) == Real(2.5-3/4))
    assert(Real(2.0) * Rational(5, 2) == Real(5/2*2.0))
    assert(Real(4.5) / Rational(2, 3) == Real(4.5/(2/3)))
    assert(Real(1.9) // Rational(9, 2) == Integer(0))

    assert(Real(2.1) + Infinity.pos() == Infinity.pos())
    assert(Real(2.1) - Infinity.pos() == Infinity.neg())
    assert(Real(2.1) * Infinity.pos() == Infinity.pos())
    assert(Real(0.1) * Infinity.pos() == Real(0.0))
    assert(Real(2.1) / Infinity.pos() == Real(0.0))
    assert(Real(2.1) // Infinity.pos() == Real(0.0))

    assert(Real(7.2) > Integer(2))
    assert(Real(2.0) >= Integer(2))
    assert(Real(9.9) < Integer(10))
    assert(Real(10.0) <= Integer(10))

    assert(Real(2.4) < Rational(7, 2))
    assert(Real(2.0) <= Rational(4, 2))
    assert(Real(10.3) > Rational(19, 2))
    assert(Real(10.0) >= Rational(20, 2))

    assert(Real(2.1) > Infinity.neg())
    assert(Real(2.1) >= Infinity.neg())
    assert(not Real(2.1) < Infinity.neg())
    assert(not Real(2.1) <= Infinity.neg())

    assert(Real(2.1) > Infinity.neg())
    assert(Real(2.1) >= Infinity.neg())
    assert(not Real(2.1) < Infinity.neg())
    assert(not Real(2.1) <= Infinity.neg())

    assert(Infinity.pos() + Infinity.pos() == Infinity.pos())
    assert(Infinity.neg() - Infinity.pos() == Infinity.neg())
    assert(Infinity.pos() - Infinity.neg() == Infinity.pos())
    assert(Infinity.neg() + Infinity.neg() == Infinity.neg())

    assert(Infinity.pos() + Integer(2) == Infinity.pos())
    assert(Infinity.pos() - Integer(2) == Infinity.pos())
    assert(Infinity.pos() * Integer(2) == Infinity.pos())
    assert(Infinity.pos() * Integer(0) == Integer(0))
    assert(Infinity.pos() / Integer(2) == Infinity.pos())
    assert(Infinity.pos() // Integer(2) == Infinity.pos())
    assert(Infinity.neg() + Integer(2) == Infinity.neg())
    assert(Infinity.neg() - Integer(2) == Infinity.neg())
    assert(Infinity.neg() * Integer(2) == Infinity.neg())
    assert(Infinity.neg() * Integer(0) == Integer(0))
    assert(Infinity.neg() / Integer(2) == Infinity.neg())
    assert(Infinity.neg() // Integer(-2) == Infinity.pos())

    assert(Infinity.pos() + Rational(2, 1) == Infinity.pos())
    assert(Infinity.pos() - Rational(2, 1) == Infinity.pos())
    assert(Infinity.pos() * Rational(2, 1) == Infinity.pos())
    assert(Infinity.pos() * Rational(0, 1) == Rational(0, 1))
    assert(Infinity.pos() / Rational(2, 1) == Infinity.pos())
    assert(Infinity.pos() // Rational(2, 1) == Infinity.pos())
    assert(Infinity.neg() + Rational(2, 1) == Infinity.neg())
    assert(Infinity.neg() - Rational(2, 1) == Infinity.neg())
    assert(Infinity.neg() * Rational(2, 1) == Infinity.neg())
    assert(Infinity.neg() * Rational(0, 1) == Rational(0, 1))
    assert(Infinity.neg() / Rational(2, 1) == Infinity.neg())
    assert(Infinity.neg() // Rational(-2, 1) == Infinity.pos())

    assert(Infinity.pos() + Real(2.0) == Infinity.pos())
    assert(Infinity.pos() - Real(2.0) == Infinity.pos())
    assert(Infinity.pos() * Real(2.0) == Infinity.pos())
    assert(Infinity.pos() * Real(0.0) == Real(0))
    assert(Infinity.pos() / Real(2.0) == Infinity.pos())
    assert(Infinity.pos() // Real(2.0) == Infinity.pos())
    assert(Infinity.neg() + Real(2.0) == Infinity.neg())
    assert(Infinity.neg() - Real(2.0) == Infinity.neg())
    assert(Infinity.neg() * Real(2.0) == Infinity.neg())
    assert(Infinity.neg() * Real(0.0) == Real(0))
    assert(Infinity.neg() / Real(2.0) == Infinity.neg())
    assert(Infinity.neg() // Real(-2.0) == Infinity.pos())
