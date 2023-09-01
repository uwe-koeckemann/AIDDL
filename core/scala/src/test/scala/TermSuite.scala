import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.ComboIterator

class TermSuite extends AnyFunSuite {
    test("Symbol Equality and Hash Codes") {
        val s1 = Sym("my-module")
        val s2 = Sym("my-module")
        val s3 = Sym("my-module-2")
        
        assert(s1 == s2)
        assert(s1.hashCode() == s2.hashCode())

        assert(s2 != s3)
        assert(s2.hashCode() != s3.hashCode())
    }

    test("Symbol, string, variable, function reference hash codes and equality") {
        val sym = Sym("name")
        val str = Str("name")
        val variable = Var("name")
        val funRef = FunRef(Sym("name"), x => x)

        assert(sym.hashCode() != str.hashCode)
        assert(sym.hashCode() != variable.hashCode)
        assert(sym.hashCode() != funRef.hashCode)
    } 

    test("Anonymous variables not equal") {
        val v1 = Var()
        val v2 = Var()
        
        assert(v1 != v2)
    }

    test("Booleans can be viewed as numerical terms") {
        assert(Bool(true).asNum == Num(1))
        assert(Bool(false).asNum == Num(0))
    }

    test("Boolean to string method") {
        assert(Bool(true).toString == "true")
        assert(Bool(false).toString == "false")
    }

    test("Basic Boolean operators") {
        assert((Bool(true) || Bool(true)) == Bool(true))
        assert((Bool(false) || Bool(true)) == Bool(true))

        assert((Bool(true) && Bool(true)) == Bool(true))
        assert((Bool(false) && Bool(true)) == Bool(false))
    }

    test("Only equal symbols match") {
        val a = Sym("a")
        val b = Sym("b")

        assert( (a unify b) == None )
        assert( (a unify a) == Some(new Substitution()))
    }

    test("Variables unify to left") {
        val x = Var("x")
        val a = Sym("a")

        assert( (a unify x) == None )
        assert( (x unify a) == Some(new Substitution(x, a)))
        assert( (x unify x) == Some(new Substitution()))  
    }

    test("Extension method allows to add Int to AIDDL Integer") {
        val a = 10
        val b = Num(20)

        assert(a + b == Num(30))
    }

    test("Tuples unify") {
        val t1 = Tuple(Sym("p"), Var("x"))
        val t2 = Tuple(Sym("p"), Sym("c"))
        val t3 = Tuple(Sym("p"), Sym("c"), Var("y"))

        assert( (t2 unify t1) == None )
        assert( (t1 unify t3) == None )
        assert( (t1 unify t2) == Some(new Substitution(Var("x"), Sym("c"))))
        assert( (t1 unify t1) == Some(new Substitution()))
    } 

    test("List construction") {
        val a = Sym("a")
        val list = ListTerm(Sym("a"), Sym("b"), Sym("c"))
        assert(list.contains(a))
    }

    test("Set construction") {
        val a = Sym("a")
        val set = SetTerm(Sym("a"), Sym("b"), Sym("c"))
        assert(set.contains(a))
    }

    test("Tuple equal to itself") {
        val tuple = Tuple(Sym("a"), Sym("b"), Sym("c"))
        assert(tuple == tuple)
    }

    test("Tuple and list are not equal") {
        val list = ListTerm(Sym("a"), Sym("b"), Sym("c"))
        val tuple = Tuple(Sym("a"), Sym("b"), Sym("c"))
        assert(list != tuple)
        assert(list.hashCode != tuple.hashCode())
    }

    test("Chaining optional substitutions") {
        val s1:Option[Substitution] = Some(new Substitution(Var("x"), Sym("a")))
        val s2:Option[Substitution] = Some(new Substitution(Var("y"), Sym("b")))
        val s3:Option[Substitution] = Some(new Substitution(Var("z"), Sym("c")))
        val s4:Option[Substitution] = Some(new Substitution(Var("z"), Sym("d")))

        val list = List(s1, s2, s3)
        val list_bad = List(s4, s2, s3, s1)   

        val init:Option[Substitution] = Some(new Substitution())

        val r = list.foldLeft(init)( (c, x) => (c flatMap (_ + x)))    //c.fold(None)(x => c + x) )
        assert(r != None)

        val r_bad = list_bad.foldLeft(init)( (c, x) => (c flatMap (_ + x)))    //c.fold(None)(x => c + x) )
        assert(r_bad == None)
    }

    test("Min and max method on numerical terms") {
        assert(Num(10) == Num(30.5).min(Num(10)))
        assert(Num(30.5) == Num(30.5).max(Num(10)))
    }

    test("Absolute value test") {
        assert(Num(10).abs == Num(-10).abs)
        assert(Num(10).abs == Num(10))
        assert(Num(-110).abs.isPos)
    }

    test("Creating a real term") {
        assert(Num(0.0).intoDouble == 0.0)
        assert(Num(0.0f).intoDouble == 0.0)
    }

    test("Infinity terms") {
        assert(InfPos().isInf)
        assert(InfNeg().isInf)
        assert(!Num(10).isInf)
        assert(!InfPos().isInfNeg)
        assert(!InfNeg().isInfPos)
        assert(-InfPos() == InfNeg())
        assert(-InfNeg() == InfPos())

        assert((InfNeg() * NaN()).isNan)
        assert(InfNeg() * Num(0) == Num(0))
        assert((InfNeg() * Num(-10)).isInfPos)
        assert((InfNeg() * Num(10)).isInfNeg)

        assert((InfPos() * NaN()).isNan)
        assert(InfPos() * Num(0) == Num(0))
        assert((InfPos() * Num(-10)).isInfNeg)
        assert((InfPos() * Num(10)).isInfPos)

        assert((InfNeg() / Num(-2)).isInfPos)
        assert((InfNeg() / Num(2)).isInfNeg)
        assert((InfNeg() / Num(-2, 4)).isInfPos)
        assert((InfNeg() / Num(2, 4)).isInfNeg)
        assert((InfNeg() / Num(-2.5)).isInfPos)
        assert((InfNeg() / Num(2.5)).isInfNeg)
        assert((InfNeg() / Num(0)).isNan)
        assert((InfNeg() / InfNeg()).isNan)
        assert(InfNeg().floorDiv(Num(0)).isNan)
        
        assert((InfPos() / Num(-2)).isInfNeg)
        assert((InfPos() / Num(2)).isInfPos)
        assert((InfPos() / Num(-2, 4)).isInfNeg)
        assert((InfPos() / Num(2, 4)).isInfPos)
        assert((InfPos() / Num(-2.5)).isInfNeg)
        assert((InfPos() / Num(2.5)).isInfPos)
        assert((InfPos() / Num(0)).isNan)
        assert((InfPos() / InfNeg()).isNan)
        assert(InfPos().floorDiv(Num(0)).isNan)

        assert(InfPos().toString == "+INF")
        assert(InfNeg().toString == "-INF")

        assert(InfPos() <= InfPos())
        assert(InfPos() > InfNeg())

        assert(Num(3,5) < InfPos())
        assert((Num(3, 4) + InfPos()).isInfPos)
        assert((Num(3, 4) + InfNeg()).isInfNeg)
        assert((Num(3, 4) - InfPos()).isInfNeg)
        assert((Num(3, 4) - InfNeg()).isInfPos)

        assert((Num(-3, 4) * InfPos()).isInfNeg)
        assert((Num(3, 4) * InfNeg()).isInfNeg)

        assert((Num(-3, 4) * InfNeg()).isInfPos)
        assert((Num(3, 4) * InfPos()).isInfPos)


    }

    test("Negating numerical term") {
        assert(-Num(2) == Num(-2))
        assert(-Num(2, 4) == Num(-2, 4))
        assert(-Num(2.5) == Num(-2.5))
    }

    test("Numerical term conversions") {
        assert(Num(5).asRat == Rational(5, 1))
        assert(Num(5).asReal == Real(5/1))
        assert(Num(0.5).asReal == Real(0.5))
        assert(Num(1, 2).asReal == Num(0.5))
    }

    test("Dividing numerical terms") {
        assert((Num(5) / InfPos()) == Num(0))
        assert((Num(5) / InfNeg()) == Num(0))
        assert((Num(5) floorDiv InfPos()) == Num(0))
        assert((Num(5) floorDiv InfNeg()) == Num(0))
        assert((Num(5) floorDiv Num(0)).isNan)
        assert((Num(5) floorDiv Num(2)) == Num(2))
        assert((Num(5) floorDiv Num(2,4)) == Rational(10, 1))
        assert((Num(5) floorDiv Num(0.5)) == Real(10.0))

        assert((Num(5, 7) / InfPos()) == Num(0))
        assert((Num(5, 7) / InfNeg()) == Num(0))
    }

    test("Adding with NaN") {
        assert((Num(2) + NaN()).isNan)
    }

    test("FunRef unification and unapply") {
        val aRef = FunRef(Sym("f"), x => x)
        val bRef = FunRef(Sym("g"), x => x)
        assert(aRef.unify(aRef).isDefined)
        assert(aRef.unify(bRef).isEmpty)

        val FunRef(uri, f) = aRef
        assert(uri == Sym("f"))
    }

    test("tryIntoBool and variants") {
        assert(Bool(true).tryIntoBool == Some(Bool(true)))
        assert(Bool(false).tryIntoBool == Some(Bool(false)))
        assert(Sym("a").tryIntoBool == None)
        assert(Tuple().tryIntoBool == Some(Bool(false)))
        assert(SetTerm.empty.tryIntoBool == Some(Bool(false)))
        assert(ListTerm.empty.tryIntoBool == Some(Bool(false)))

        assert(Tuple(Sym("a")).tryIntoBool == Some(Bool(true)))
        assert(SetTerm(Sym("a")).tryIntoBool == Some(Bool(true)))
        assert(ListTerm(Sym("a")).tryIntoBool == Some(Bool(true)))

        assert(Num(42).tryIntoBool == Some(Bool(true)))
        assert(Num(4.2).tryIntoBool == Some(Bool(true)))
        assert(Num(4, 2).tryIntoBool == Some(Bool(true)))
        assert(InfPos().tryIntoBool == Some(Bool(true)))
        assert(InfNeg().tryIntoBool == Some(Bool(true)))

        assert(Num(0).tryIntoBool == Some(Bool(false)))
        assert(Num(0.0).tryIntoBool == Some(Bool(false)))
        assert(Num(0, 1).tryIntoBool == Some(Bool(false)))
        assert(NaN().tryIntoBool == Some(Bool(false)))

        assert(Sym("a").intoBoolOr(Bool(false)) == Bool(false))

        assert(Num(0).intoBool == Bool(false))
        assert(Num(32.5).intoBool == Bool(true))
    }

    test("tryIntoInt and variants") {
        assert(Sym("a").tryIntoInt == None)
        assert(Num(3).tryIntoInt == Some(3))
        assert(Num(3).intoInt == 3)
        assert(Num(3).intoIntOr(5) == 3)
        assert(Sym("a").intoIntOr(5) == 5)
        assert(Num(3.9).intoInt == 3)
        assert(Num(10, 3).intoInt == 3)
    }

    test("tryIntoLong and variants") {
        assert(Sym("a").tryIntoLong == None)
        assert(Num(3).tryIntoLong == Some(3L))
        assert(Num(3).intoLong == 3L)
        assert(Num(3).intoLongOr(5L) == 3L)
        assert(Sym("a").intoLongOr(5L) == 5L)
        assert(Num(3.9).intoLong == 3L)
        assert(Num(10, 3).intoLong == 3L)
    }

    test("tryIntoFloat and variants") {
        assert(Sym("a").tryIntoFloat == None)
        assert(Num(3).tryIntoFloat == Some(3.0f))
        assert(Num(3).intoFloat == 3.0)
        assert(Num(3).intoFloatOr(5L) == 3.0)
        assert(Sym("a").intoFloatOr(5.0) == 5.0)
        assert(Num(3.9).intoFloat == 3.9f)
        assert(Num(10, 3).intoFloat == (10.0/3.0).toFloat)
    }

    test("tryIntoDouble and variants") {
        assert(Sym("a").tryIntoDouble == None)
        assert(Num(3).tryIntoDouble == Some(3.0))
        assert(Num(3).intoDouble == 3.0)
        assert(Num(3).intoDoubleOr(5L) == 3.0)
        assert(Sym("a").intoDoubleOr(5.0) == 5.0)
        assert(Num(3.9).intoDouble == 3.9)
        assert(Num(10, 3).intoDouble == 10.0 / 3.0)
    }

    test("EntRef matching and isGround") {
        val m = Sym("m")
        val name = Sym("name")
        val otherName = Sym("other-name")
        val variable = Var("x")
        val alias = Sym("self")
        assert((EntRef(m, variable, alias) unify EntRef(m, name, alias)).isDefined)
        assert((EntRef(m, otherName, alias) unify EntRef(m, name, alias)).isEmpty)
        assert((EntRef(m, otherName, alias) unify otherName).isEmpty)

        assert(!EntRef(m, variable, alias).isGround)
        assert(EntRef(m, otherName, alias).isGround)
    }

    test("Term collection test") {
        val container = new Container
        val parser = new Parser(container)
        val term = parser.str("{x [x y] {x} k:x x:v [[[[x]]]]}")
        val answer = Term.collect(_ == Sym("x"))(term)
        assert(answer.length == 6)
    }

    test("Resolving non reference returns term itself") {
        val x = Sym("x")
        assert(x.resolve(new Container) == x)
    }

    test("Not implemented term conversions") {
        assertThrows[IllegalAccessError](Num(1).asSym)
        assertThrows[IllegalAccessError](Num(1).asList)
        assertThrows[IllegalAccessError](Num(1).asSet)
        assertThrows[IllegalAccessError](Num(1).asCol)
        assertThrows[IllegalAccessError](Num(1).asTup)
        assertThrows[IllegalAccessError](Num(1).asVar)
        assertThrows[IllegalAccessError](Num(1).asKvp)
        assertThrows[IllegalAccessError](Num(1).asEntRef)
        assertThrows[IllegalAccessError](Num(1).asFunRef)
        assertThrows[IllegalAccessError](Num(1).asStr)

        assertThrows[IllegalAccessError](Sym("X").asNum)
        assertThrows[IllegalAccessError](Sym("X").asInt)
        assertThrows[IllegalAccessError](Sym("X").asRat)
        assertThrows[IllegalAccessError](Sym("X").asReal)
        assertThrows[IllegalAccessError](Sym("X").asBool)

        assertThrows[IllegalAccessError](Sym("X")(Sym("Y")))
        assertThrows[IllegalAccessError](Sym("X")(0))
        assertThrows[IllegalAccessError](Sym("X").length)
    }
}
