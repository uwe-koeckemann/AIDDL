package org.aiddl.common.scala.execution

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.execution.Actor.{ActionInstanceId, Status}
import org.aiddl.common.scala.execution.{Actor, Sensor}
import org.aiddl.common.scala.execution.Controller
import org.aiddl.common.scala.execution.Controller.{Instruction, Signal}
import org.aiddl.common.scala.execution.Controller.Signal.*
import org.aiddl.core.scala.representation
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class ControllerSuite extends AnyFunSuite {

  test("Simple controller achieves goal") {
    object Simulation extends Actor with Sensor {
      private var state: Num = Num(0)

      override def supported(action: Term): Boolean =
        action == Sym("+") || action == Sym("-")

      override def dispatch(action: Term): Option[ActionInstanceId] = {
        action match
          case Sym("+") =>
            state = state + Num(2)
            Some(this.nextIdWithStatus(Status.Succeeded))
          case Sym("-") =>
            state = state - Num(1)
            Some(this.nextIdWithStatus(Status.Succeeded))
          case _ => None
      }
      override def sense: Term =
        state
    }

    object NumberController extends Controller {
      override val actor: Actor = Simulation
      override val sensor: Sensor = Simulation

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


    def callback(signal: Signal): Unit = {
      println(signal)
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
    assert(NumberController.decide(Simulation.sense) == Nil)
    NumberController.setGoal(Num(5))
    assert(NumberController.currentGoal == Num(5))
    assert(Simulation.sense == Num(0))
    Simulation.tick
    NumberController.tick
    assert(Simulation.sense == Num(2))
    Simulation.tick
    NumberController.tick
    assert(Simulation.sense == Num(4))
    NumberController.disable
    Simulation.tick
    NumberController.tick
    assert(Simulation.sense == Num(4))
    NumberController.enable
    Simulation.tick
    NumberController.tick
    assert(Simulation.sense == Num(6))
    Simulation.tick
    NumberController.tick
    assert(Simulation.sense == Num(5))
    Simulation.tick
    NumberController.tick
    assert(Simulation.sense == Num(5))
    NumberController.setGoal(Num(4))
    Simulation.tick
    NumberController.tick
    assert(Simulation.sense == Num(4))
    Simulation.tick
    NumberController.tick
  }
}
