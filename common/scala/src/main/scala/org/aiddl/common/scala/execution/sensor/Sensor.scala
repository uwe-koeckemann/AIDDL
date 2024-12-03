package org.aiddl.common.scala.execution.sensor

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.common.scala.execution.sensor.{SensorMode, SensorValue}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Sym, Term}

trait Sensor extends Tickable {
  /**
   * Value of the sensor before the first read
   */
  private var currentValue: SensorValue = {
    SensorValue (Common.NIL, -1, 0)
  }
  private var nextSequenceId: Long = 1

  val sensorMode: SensorMode

  var callbacks: List[SensorValue => Unit] =
    Nil

  /**
   * Sense the current state. This method is called by the tick
   * method so if the system is ticked on some frequency,
   * the latestValue method should be used
   * @return
   */
  protected def performSense: Term

  /**
   * Get latest sequence ID and sensor value
   * @return
   */
  def sense: SensorValue =
    this.sensorMode match {
      case SensorMode.OnDemand => this.performSenseAndUpdate
      case SensorMode.Frequency => this.getLatestSensorValue
      case SensorMode.Mixed => this.performSenseAndUpdate
    }

  def getLatestSensorValue: SensorValue =
    if currentValue != SensorValue(Common.NIL, -1, 0)
    then this.currentValue
    else throw new IllegalAccessException(s"Sensor in mode Frequency needs call to tick method before it has a value.")


  private def performSenseAndUpdate: SensorValue = {
    val newValue = this.performSense
    this.currentValue = SensorValue(newValue, nextSequenceId, System.nanoTime())
    this.nextSequenceId += 1
    this.currentValue
  }

  def registerCallback( f: SensorValue => Unit ): Unit =
    callbacks = f :: callbacks

  /**
   * Sense a new value, update latest value and sequence ID and perform callbacks
   */
  override def tick: Unit = {
    this.sensorMode match {
      case SensorMode.OnDemand => {}
      case SensorMode.Frequency => this.performSenseAndUpdate
      case SensorMode.Mixed => this.performSenseAndUpdate
    }
    callbacks.foreach(f => f(this.currentValue))
  }
}
