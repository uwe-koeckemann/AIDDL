package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*

object LambdaFunctionEvaluator {
  private var NextID = 0
}

protected[function] class LambdaFunctionEvaluator(c: Container) extends Function with LazyFunction {
  private class LambdaFunction(x: Term, f: Term, eval: Evaluator) extends Function {

    override def apply(arg: Term): Term = x unify arg match {
      case Some(s) => eval(f \ s)
      case None => throw new IllegalArgumentException(arg.toString + " does not match argument pattern of " + this.toString())
    }

    override def toString(): String = "(org.aiddl.eval.lambda " + this.x + " " + this.f + ")"
  }

  override def apply(x: Term): Term = x match {
    case Tuple(arg, fun) => {
      val f = new LambdaFunction(arg, fun, c.eval)
      val uri = Sym("#lambda_" + LambdaFunctionEvaluator.NextID)
      LambdaFunctionEvaluator.NextID += 1
      c.addFunction(uri, f)
      FunRef(uri, f)
    }
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. Expected tuple (arg fun).")
  }
}