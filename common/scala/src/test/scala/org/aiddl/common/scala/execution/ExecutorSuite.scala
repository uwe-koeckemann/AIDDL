package org.aiddl.common.scala.execution

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.execution.Actor.Status.*
import org.aiddl.common.scala.execution.Actor.{ActionInstanceId, Status}
import org.aiddl.common.scala.execution.Sensor.SeqId
import org.aiddl.common.scala.execution.dispatch.{PartialOrderDispatcher, QueueDispatcher}
import org.aiddl.common.scala.execution.{Actor, Sensor}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

import scala.io.Source

class ExecutorSuite extends AnyFunSuite {
  test("Sequential sensor with callbacks gives correct values") {
    var counter = 0
    var readings: List[Term] = Nil

    def cb(seqId: SeqId, value: Term) = {
      counter += 1
      readings = value :: readings
    }

    object SequenceSensor extends Sensor {
      private var x: Long = 0
      this.currentValue = Num(0)

      override def sense: Term = {
        this.x += 1
        Num(x)
      }

      registerCallback(cb)
    }

    assert(SequenceSensor.latest == (0, Num(0)))
    SequenceSensor.tick
    assert(SequenceSensor.latest == (1, Num(1)))
    assert(SequenceSensor.latestSequenceId == 1)
    assert(SequenceSensor.latestValue == Num(1))
    SequenceSensor.tick
    assert(SequenceSensor.latest == (2, Num(2)))
    SequenceSensor.tick
    assert(SequenceSensor.latest == (3, Num(3)))

    assert(readings == List(Num(3), Num(2), Num(1)))
    assert(counter == 3)
  }

  test("Countdown actor works") {
    object CountdownActor extends Actor {
      var countdown: Map[ActionInstanceId, Long] = Map.empty
      var idActionMap: Map[ActionInstanceId, Term] = Map.empty

      override def supported(action: Term): Boolean = action.isInstanceOf[Integer]

      override def dispatch(action: Term): Option[ActionInstanceId] = action match {
        case Integer(x) => {
          val id = super.nextId
          super.update(id, Active)
          idActionMap = idActionMap.updated(id, action)
          countdown = countdown.updated(id, x)
          Some(id)
        }
        case _ => None
      }

      def tick = {
        countdown = countdown.map((id, c) => {
          if (c == 1) super.update(id, Succeeded)
          (id, c - 1)
        }).toMap
      }
    }

    var counter = 0

    def cb(aId: ActionInstanceId, a: Term, s: Status) = {
      counter += 1
    }

    CountdownActor.registerCallback(cb)

    assert(CountdownActor.supported(Num(4)))
    assert(!CountdownActor.supported(Sym("x")))
    assert(CountdownActor.dispatch(Sym("x")) == None)

    assert(CountdownActor.idle)
    val id = CountdownActor.dispatch(Num(4)) match {
      case Some(id) => id
      case None => -1
    }
    assert(id != -1)
    assert(!CountdownActor.idle)
    assert(CountdownActor.status(id) == Active)
    CountdownActor.tick
    assert(CountdownActor.status(id) == Active)
    CountdownActor.tick
    assert(CountdownActor.status(id) == Active)
    CountdownActor.tick
    assert(CountdownActor.status(id) == Active)
    CountdownActor.tick
    assert(CountdownActor.status(id) == Succeeded)
    assert(CountdownActor.idle)

    val id2 = CountdownActor.dispatchBlock(Num(10)).get
    assert(CountdownActor.status(id2) == Succeeded)
    assert(CountdownActor.idle)

    assert(counter == 4) // 4 status updates in total
  }

  test("Queue dispatcher with instantaneous actions") {
    object InstantActor extends Actor {
      override def supported(action: Term): Boolean = true
      override def dispatch(action: Term): Option[ActionInstanceId] = {
        val id = super.nextId
        actionIdMap.put(id, action)
        super.update(id, Succeeded)
        Some(id)
      }

      def tick = {}
    }
    var execList: List[Term] = Nil

    def cb(aId: ActionInstanceId, a: Term, s: Status) = {
      execList = a :: execList
    }

    InstantActor.registerCallback(cb)

    val dispatcher = new QueueDispatcher
    val actions = (1 to 10).map(i => Num(i)).toList
    dispatcher.enqueueAll(actions)
    dispatcher.actors = List(InstantActor)

    // 10 ticks to execute 10 instantaneous actions
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick
    dispatcher.tick

    assert(execList.reverse == actions)
  }

  test("Partial-order dispatcher with instantaneous actions") {
    object InstantActor extends Actor {
      override def supported(action: Term): Boolean = true

      override def dispatch(action: Term): Option[ActionInstanceId] = {
        val id = super.nextId
        actionIdMap.put(id, action)
        super.update(id, Succeeded)
        Some(id)
      }

      def tick = {}
    }

    var execSet: Set[Term] = Set.empty
    def cb(aId: ActionInstanceId, a: Term, s: Status) = {
      execSet = execSet + a
    }

    InstantActor.registerCallback(cb)

    val dispatcher = new PartialOrderDispatcher
    dispatcher.actors = List(InstantActor)
    dispatcher.add(Num(1), Num(1), Set.empty)
    dispatcher.add(Num(2), Num(2), Set.empty)
    dispatcher.add(Num(3), Num(3), Set(Num(1)))
    dispatcher.add(Num(4), Num(4), Set(Num(2), Num(3)))
    dispatcher.add(Num(5), Num(5), Set(Num(1), Num(3)))

    dispatcher.tick
    dispatcher.tick
    dispatcher.tick

    // after three ticks all five actions have been executed
    assert(execSet == Set(Num(1), Num(2), Num(3), Num(4), Num(5)))
  }
}