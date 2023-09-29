package org.aiddl.common.scala.execution

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.execution.Actor.{ActionInstanceId, Status}
import org.aiddl.common.scala.execution.{Actor, Sensor}
import org.aiddl.common.scala.execution.Controller
import org.aiddl.common.scala.execution.Controller.{Instruction, Signal}
import org.aiddl.common.scala.execution.Controller.Signal.*
import org.aiddl.common.scala.execution.simulation.{Simulation, SimulationEvent}
import org.aiddl.core.scala.representation
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class ControllerSuite extends AnyFunSuite {

  test("Simple controller achieves goal") {
    object NumberSimulator extends Simulation(Num(0)) {
      private var currentAction: Option[Term] = None
      private var currentActionId: ActionInstanceId = 0L


      override def supported(action: Term): Boolean =
        action == Sym("+") || action == Sym("-")

      override def handleActions(): Term =
        this.update(this.currentActionId, Status.Succeeded)
        val nextState = currentAction match
          case Some(Sym("+")) => this.state.asNum + Num(2)
          case Some(Sym("-")) => this.state.asNum - Num(1)
          case _ => state
        this.currentAction = None
        nextState

      override def dispatch(action: Term): Option[ActionInstanceId] = {
        action match
          case Sym("+") =>
            this.currentAction = Some(action)
            this.currentActionId = nextIdWithStatus(Status.Active)
            Some(this.currentActionId)
          case Sym("-") =>
            this.currentAction = Some(action)
            this.currentActionId = nextIdWithStatus(Status.Active)
            Some(this.currentActionId)
          case _ => None
      }

      override def sense: Term =
        state
    }

    object NumberController extends Controller {
      override val actor: Actor = NumberSimulator
      override val sensor: Sensor = NumberSimulator

      override def decide(state: Term): List[Instruction] = {
        if ( state.asNum < currentGoal.asNum ) {
          List(Instruction(Sym("+")))
        } else if ( state.asNum > currentGoal.asNum ) {
          List(Instruction(Sym("-")))
        } else {
          this.callback(GoalReached(this.currentGoal))
          Nil
        }
      }
    }

    object NumberEvent extends SimulationEvent {
      var isPossible = true
      override def probability: Num = Num(1)
      override def possible: Boolean = isPossible
      override def applicable(state: Term): Boolean = true
      override def apply(state: Term): Term = {
        isPossible = false
        state.asNum + Num(1)
      }
    }

    var expectedSignalStream: List[Signal] = List(
      GoalUpdated(Num(0)),
      GoalReached(Num(0)),
      GoalUpdated(Num(5)),
      Dispatched(0,Num(0), Sym("+")),
      Dispatched(1,Num(1), Sym("+")),
      Disabled,
      Skipped,
      Enabled,
      Dispatched(2,Num(2), Sym("+")),
      Dispatched(3,Num(3), Sym("-")),
      GoalReached(Num(5)),
      GoalUpdated(Num(4)),
      Dispatched(4,Num(4), Sym("-")),
      GoalReached(Num(4)),
      GoalReached(Num(4)),
      Dispatched(5,Num(5), Sym("-")),
      Dispatched(6,Num(6), Sym("-")),
      GoalReached(Num(4))
    )

    NumberController.registerCallback(
      (signal: Signal) => {
        val expectedSignal = expectedSignalStream.head
        assert(signal == expectedSignal)
        expectedSignalStream = expectedSignalStream.tail
      }
    )

    NumberController.setGoal(Num(0))
    assert(NumberController.decide(NumberSimulator.sense) == Nil)
    NumberController.setGoal(Num(5))
    assert(NumberController.currentGoal == Num(5))
    assert(NumberSimulator.sense == Num(0))
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(2))
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(4))
    NumberController.disable
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(4))
    NumberController.enable
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(6))
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(5))
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(5))
    NumberController.setGoal(Num(4))
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(4))
    NumberController.tick
    NumberSimulator.tick
    NumberSimulator.addEvent(NumberEvent)
    NumberController.tick
    assert(NumberSimulator.sense == Num(4))
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(5))
    NumberController.tick
    NumberSimulator.tick
    assert(NumberSimulator.sense == Num(4))
  }
}
