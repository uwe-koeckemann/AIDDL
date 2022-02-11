package org.aiddl.core.scala.function.`type`

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.BoolImplicits.*
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermUnpackImplicits.term2int
import org.aiddl.core.scala.tools.Logger

object GenericTypeChecker {
  private  var nextFreeId = 0
}

class GenericTypeChecker(uri: Sym, pattern: Term, typeDef: Term, e: Evaluator, c: Container) extends Function {
  def apply(args: Term): Term = {
    pattern unify args match {
      case Some(s) => {
        GenericTypeChecker.nextFreeId += 1
        val uri = this.uri + Sym(s"${GenericTypeChecker.nextFreeId}")
        val fType = new TypeFunction(typeDef\s, e)
        c.addFunction(uri, fType)
        FunRef(uri, fType)
      }
      case None => throw IllegalArgumentException(s"Pattern $args does not match pattern $typeDef of generic type $uri")
    }
  }
}
