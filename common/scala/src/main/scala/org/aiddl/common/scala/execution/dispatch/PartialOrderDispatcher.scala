package org.aiddl.common.scala.execution.dispatch

import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.Actor.ActionInstanceId
import org.aiddl.common.scala.execution.Actor.Status.*
import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.core.scala.representation.Term

import scala.collection.immutable.Queue
import scala.collection.mutable

class PartialOrderDispatcher extends Dispatcher {
  private val open: mutable.Set[Term] = mutable.HashSet.empty
  private val finished: mutable.Set[Term] = mutable.HashSet.empty
  private val idActionMap: mutable.Map[Term, Term] = mutable.HashMap.empty
  private val predecessors: mutable.Map[Term, Set[Term]] = mutable.Map.empty

  //private val id: Map[Term, Term] = Map.empty

  private val running: mutable.Map[Term, List[(Actor, ActionInstanceId)]] = mutable.Map.empty

  def add( id: Term, action: Term, pres: Iterable[Term] ) =
    open.add(id)
    idActionMap.put(id, action)
    predecessors.put(id, pres.toSet)
    
  def tick = {
    var atLeastOneFinished = false
    // tick all actors and collect finished actions
    running.foreach( (id, actorList) => {
      val remainingActors = actorList.filter( actInfo => {
        val (actor, instId) = actInfo
        actor.tick
        actor.status(instId) match {
          case Succeeded => false
          case Recalled => false
          case Preempted => false
          case err@Error(_, _) => {
            this.errorHandler(id, idActionMap(id), actor, err)
            false
          }
          case _ => true
        }
      })
      if ( remainingActors.isEmpty ) {
        atLeastOneFinished = true
        running.remove(id)
        finished.add(id)
      } else {
        running.put(id, remainingActors)
      }
    })

    if ( atLeastOneFinished || running.isEmpty ) {
      // find actions without predecessors
      val unlocked = open.filter(id => predecessors(id).subsetOf(finished))
      // collect actors
      unlocked.foreach(id => {
        val runningActors = actors
          .map(a => (a, a.dispatch(idActionMap(id))))
          .collect({ case (actor, Some(aId)) => {

            (actor, aId)
          } })
        running.put(id, runningActors)
      })
      open --= unlocked
    }
  }
}
