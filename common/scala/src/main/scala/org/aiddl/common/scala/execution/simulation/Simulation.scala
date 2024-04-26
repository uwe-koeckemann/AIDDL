package org.aiddl.common.scala.execution.simulation

import org.aiddl.common.scala.execution.Actor.ActionInstanceId
import org.aiddl.common.scala.execution.sensor.{Sensor, SensorMode, SensorValue}
import org.aiddl.common.scala.execution.Actor
import org.aiddl.core.scala.representation.{Num, Term}

import scala.util.Random

trait Simulation(initialState: Term) extends Actor with Sensor {
  protected var random: Random = Random
  protected var state: Term = initialState
  protected var events: List[SimulationEvent] = Nil

  override val sensorMode: SensorMode = SensorMode.Frequency

  def addEvent(event: SimulationEvent): Unit =
    this.events = event :: this.events

  override def performSense: Term = state
  override def supported(action: Term): Boolean
  override def dispatch(action: Term): Option[ActionInstanceId]

  /**
   * Handle received actions and return a new state
   * @return the state after handling actions
   */
  def handleActions(): Term

  override def tick: Unit = {
    this.handleEvents()
    this.state = handleActions()
    super.tick
  }

  private def handleEvents(): Unit = {
    this.events = this.events.filter(_.possible)
    this.state = events
      .filter(e => Num(random.nextDouble()) <= e.probability && e.applicable(this.state))
      .foldLeft(state)((state, event) => {
        event.apply(state)
      })
  }
}
