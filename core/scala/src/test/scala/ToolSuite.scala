import org.scalatest.funsuite.AnyFunSuite

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry

import org.aiddl.core.scala.function.Function

import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.Substitution
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.Str
import org.aiddl.core.scala.representation.FunRef
import org.aiddl.core.scala.representation.SetTerm
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.util.ComboIterator

class ToolSuite extends AnyFunSuite {
    test("Combo iterator basic case") {
        val choices = List(List(Sym("a"), Sym("b"), Sym("c")), List(Num(1),  Num(2),  Num(3)))

        val comboIt = new ComboIterator(choices)

        var n = comboIt.next()
        assert(n == List(Sym("a"), Num(1)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("a"), Num(2)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("a"), Num(3)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("b"), Num(1)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("b"), Num(2)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("b"), Num(3)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("c"), Num(1)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("c"), Num(2)))
        assert(comboIt.hasNext)

        n = comboIt.next()
        assert(n == List(Sym("c"), Num(3)))
        assert(!comboIt.hasNext)
    }
    test("Combo iterator empty choice case") {
        val choices = List(List(Sym("a"), Sym("b"), Sym("c")), List(), List(Num(1),  Num(2),  Num(3)))

        val comboIt = new ComboIterator(choices)
        assert(!comboIt.hasNext)
    }
}