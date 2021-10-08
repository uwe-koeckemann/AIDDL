import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Num


class EvaluatorSuite extends AnyFunSuite {
  test("Evaluator on simple equation") {
    val c = new Container()
    Function.loadDefaultFunctions(c)

    val eval = new Evaluator(c)

    val eqn = Parser.parse("(org.aiddl.eval.numerical.add 1 2 3)").head

    assert(Num(6) == eval.apply(eqn))
  }
}

