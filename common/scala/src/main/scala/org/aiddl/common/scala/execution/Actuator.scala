package org.aiddl.common.scala.execution

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Sym, Term}

object Actuator {
  val WAITING = Sym("waiting")
  val RUNNING = Sym("running")
  val DONE = Sym("done")
  val FAILED = Sym("failed")
}

trait Actuator extends  Function {
  def doBlock( a: Term ): Int
  def doQueue( a: Term ): Int

  def status: Term
  def status( id: Int ): Term


  def queue: List[Term]
  def idle: Boolean = this.queue.isEmpty
}


