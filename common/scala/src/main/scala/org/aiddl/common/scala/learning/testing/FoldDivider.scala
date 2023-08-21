package org.aiddl.common.scala.learning.testing

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

class FoldDivider extends Function {
  override def apply(x: Term): Term = {
    val fold = x(Sym("fold")).intoInt
    val numFolds = x.getOrElse(Sym("num-folds"), Num(10)).intoInt
    val data = x(Sym("data")).asList

    val (test, train) = this(fold, numFolds, data)
    Tuple(test, train)
  }

  def apply(fold: Int, numFolds: Int, data: ListTerm): (ListTerm, ListTerm) = {
    val (test, train) = data
      .zipWithIndex
      .partition( (x, i) => i%numFolds == fold )

    (
      ListTerm(test.map((x, _) => x)),
      ListTerm(train.map((x, _) => x))
    )
  }

}
