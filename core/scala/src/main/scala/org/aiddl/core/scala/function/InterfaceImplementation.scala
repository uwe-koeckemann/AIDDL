package org.aiddl.core.scala.function

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.`type`.TypeFunction
import org.aiddl.core.scala.representation.*

object InterfaceImplementation {
  val Name = Sym("name")
  val Input = Sym("input")
  val Output = Sym("output")
}

/**
 * Trait for functions that implement interfaces.
 */
trait InterfaceImplementation {
  /**
   * URI of the interface that will be used to look up its definition.
   */
  val interfaceUri: Sym

  /**
   * Check if a term satisfies the output type of the interface. Used to confirm that a function satisfies its output.
   * @param c container used to evaluate the type
   * @param x term to check
   * @return <code>true</code> if <code>x</code> satisfies the output type of this interface, <code>false</code> otherwise
   */
  def checkOutput(c: Container)(x: Term): Boolean = checkInterfaceType(c.interface(interfaceUri)(InterfaceImplementation.Output), x, c.eval)

  /**
   * Check if a term satisfies the input type of the interface. Used to confirm that a function satisfies its output.
   *
   * @param c container used to evaluate the type
   * @param x term to check
   * @return <code>true</code> if <code>x</code> satisfies the input type of this interface, <code>false</code> otherwise
   */
  def checkInput(c: Container)(x: Term): Boolean = checkInterfaceType(c.interface(interfaceUri)(InterfaceImplementation.Input), x, c.eval)


  private def checkInterfaceType(it: Term, x: Term, eval: Evaluator): Boolean = it match {
    case fr: FunRef => fr(x).asBool.v
    case t: Term => {
      eval.followRefs = true
      val typeDef = eval(t)
      eval.followRefs = false
      val fType = new TypeFunction(typeDef, eval)
      fType(x).asBool.v
    }
  }

}
