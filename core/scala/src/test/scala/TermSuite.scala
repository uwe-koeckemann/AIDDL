import org.scalatest.funsuite.AnyFunSuite

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry

import org.aiddl.core.scala.function.Function

import org.aiddl.core.scala.representation._
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
}
