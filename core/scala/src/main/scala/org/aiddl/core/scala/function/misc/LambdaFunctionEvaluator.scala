package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*

object LambdaFunctionEvaluator {
  var NextID = 0
}

class LambdaFunctionEvaluator(c: Container) extends Function with LazyFunction {

  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  class LambdaFunction(x: Term, f: Term, eval: Evaluator) extends Function {

    override def apply(arg: Term): Term = x unify arg match {
      case Some(s) => eval(f \ s)
      case None => throw new IllegalArgumentException(arg.toString + " does not match argument pattern of " + this.toString())
    }

    override def toString(): String = "(org.aiddl.eval.lambda " + this.x + " " + this.f + ")"
  }

  override def apply(x: Term): Term = x match {
    case Tuple(arg, fun) => {
      val f = new LambdaFunction(arg, fun, eval)
      val uri = Sym("#lambda_" + LambdaFunctionEvaluator.NextID)
      LambdaFunctionEvaluator.NextID += 1
      c.addFunction(uri, f)
      FunRef(uri, f)
    }
    case _ => x
  }
}