package org.aiddl.common.scala.execution

import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Sym, Term}

object Sensor {
  type SeqId = Long
}

trait Sensor extends Tickable {
  import Sensor.SeqId

  var callbacks: List[(SeqId, Term) => Unit] = Nil

  def value: Term
  def seqId: SeqId

  def read: (SeqId, Term) = (seqId, value)
  def tickThenRead: (SeqId, Term) = {
    tick
    read
  }

  def registerCallback( f: (SeqId, Term) => Unit ): Unit =
    callbacks = f :: callbacks

  def tick = {
    callbacks.foreach( f => f(read._1, read._2) )
  }
}
