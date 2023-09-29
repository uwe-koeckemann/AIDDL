package my.org.aiddl_sbt_project.scala;

import my.org.aiddl_sbt_project.java.SuccessorAlgorithm
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Num, Term, Tuple}

class SumAlgorithm extends Function {
  val successorAlgorithm = new SuccessorAlgorithm

  override def apply(x: Term): Term = {
    var result = 0
    for (number <- x.asCol) {
      result += successorAlgorithm.apply(number.intoInt)
    }
    Num(result)
  }
}
