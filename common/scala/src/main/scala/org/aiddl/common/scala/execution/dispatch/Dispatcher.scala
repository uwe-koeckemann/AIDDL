package org.aiddl.common.scala.execution.dispatch

import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.Actor.Status
import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.core.scala.representation.Term

trait Dispatcher extends Tickable {
  var actors: List[Actor] = Nil

  def ignoreError(id: Term, action: Term, actor: Actor, error: Status) = {}

  def printAndIgnoreError(id: Term, action: Term, actor: Actor, error: Status): Unit = {
    println(s"Failed $action (id=$id) with $error in actor $actor will be ignored.")
  }

  def isIdle: Boolean


  var errorHandler: (Term, Term, Actor, Status) => Unit = ignoreError
}
