package org.aiddl.core.scala.function.`type`

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*

protected[scala] class GenericTypeChecker(uri: Sym, pattern: Term, typeDef: Term, e: Evaluator, c: Container) extends Function {
  private  var nextFreeId = 0

  def apply(args: Term): Term = {
    pattern unify args match {
      case Some(s) => {
        this.nextFreeId += 1
        val uri = this.uri + Sym(s"${this.nextFreeId}")
        val fType = new TypeFunction(typeDef\s, e)
        c.addFunction(uri, fType)
        FunRef(uri, fType)
      }
      case None => throw IllegalArgumentException(s"Pattern $args does not match pattern $typeDef of generic type $uri")
    }
  }
}
