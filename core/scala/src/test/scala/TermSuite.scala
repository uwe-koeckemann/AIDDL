import org.scalatest.funsuite.AnyFunSuite

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry

import org.aiddl.core.scala.function.Function

import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.tools.ComboIterator

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
}
