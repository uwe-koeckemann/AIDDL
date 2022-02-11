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

trait InterfaceImplementation {
  val interfaceUri: Sym

  def checkOutput(c: Container)(x: Term): Boolean = checkInterfaceType(c.interfaceReg(interfaceUri)(InterfaceImplementation.Output), x, c.eval)
  def checkInput(c: Container)(x: Term): Boolean = checkInterfaceType(c.interfaceReg(interfaceUri)(InterfaceImplementation.Input), x, c.eval)


  def checkInterfaceType(it: Term, x: Term, eval: Evaluator): Boolean = it match {
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
