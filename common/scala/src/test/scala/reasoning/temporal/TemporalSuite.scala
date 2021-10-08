import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.temporal.StpSolver
import org.aiddl.common.scala.reasoning.temporal.AllenInterval2Stp

class TemporalSuite extends AnyFunSuite {
    val fStp = new StpSolver
    val fAllen2Stp = new AllenInterval2Stp

    test("Stp solver satisfiable problems") {
        val c = new Container()

        val m = Parser.parseInto("../../../pub/common/test/reasoning/temporal/stp.aiddl", c)
        var es = c.getMatchingEntries(m, Var(), Tuple(Sym("stp-consistent"), Var()))
        
        es.foreach( e => {
            val stp = e.v
            assert( fStp(stp) != NIL )
        })
    }

    test("Stp solver unsatisfiable problems") {
        val c = new Container()

        val m = Parser.parseInto("../../../pub/common/test/reasoning/temporal/stp.aiddl", c)
        var es = c.getMatchingEntries(m, Var(), Tuple(Sym("stp-inconsistent"), Var()))
        
        es.foreach( e => {
            val stp = e.v
            assert( fStp(stp) == NIL )
        })
    }

    test("Allen cosntraints satisfiable problems") {
        val c = new Container()

        val m = Parser.parseInto("../../../pub/common/test/reasoning/temporal/allen-interval-constraints.aiddl", c)
        var es = c.getMatchingEntries(m, Var(), Tuple(Sym("consistent"), Var()))

        es.foreach( e => {
            val acs = e.v
            assert( fStp(fAllen2Stp(acs)) != NIL )
        })
    }

    test("Allen cosntraints unsatisfiable problems") {
        val c = new Container()

        val m = Parser.parseInto("../../../pub/common/test/reasoning/temporal/allen-interval-constraints.aiddl", c)
        var es = c.getMatchingEntries(m, Var(), Tuple(Sym("inconsistent"), Var()))

        es.foreach( e => {
            val acs = e.v
            val stp = fAllen2Stp(acs)
            val answer = fStp(stp)
            assert( answer == NIL )
        })
    }
}

