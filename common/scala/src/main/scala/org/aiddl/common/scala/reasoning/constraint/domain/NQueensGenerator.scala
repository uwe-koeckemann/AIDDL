package org.aiddl.common.scala.reasoning.constraint.domain

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*

class NQueensGenerator extends Function {

  override def apply(arg: Term): Term = {
    val n: Int = arg.asNum.toInt
    val xs = ListTerm((1 to n).map( i => Var(s"x$i") ).toList)

    val ds = ListTerm((1 to n).map( i => KeyVal(Var(s"x$i"), ListTerm((1 to n).map( j => Num(j) )))).toVector)
    val cs = ListTerm((1 to n).flatMap( i => (i+1 to n).map( j => Tuple(
      Tuple(Var(s"x$i"), Var(s"x$j")),
      new FunRef(Sym(s"#n-queens-con-$i-$j"), _ => x => {
        Bool((x(0).isInstanceOf[Var] || x(1).isInstanceOf[Var])
          || (x(0) != x(1)) && (x(0).asNum - x(1).asNum).abs != Num(j-i))
    }) ))))
    ListTerm(
      KeyVal(Variables, xs),
      KeyVal(Domains, ds),
      KeyVal(Constraints, cs)
    )
  }
}
