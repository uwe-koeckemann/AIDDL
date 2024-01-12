package org.aiddl.common.scala.execution

import org.aiddl.common.scala.execution.sensor.SensorMode.Mixed
import org.aiddl.common.scala.execution.sensor.{Sensor, SensorMode}
import org.aiddl.core.scala.representation.{Sym, Term}
import org.scalatest.funsuite.AnyFunSuite

class SensorSuite extends AnyFunSuite {
  test("Mixed sensor provides value without being ticked") {
    object TestSensor extends Sensor {
      override val sensorMode: SensorMode = Mixed
      override protected def performSense: Term = Sym("x")
    }

    val firstSenseResult = TestSensor.sense
    assert(firstSenseResult.value == Sym("x"))
    assert(firstSenseResult.sequenceId == 1)
    TestSensor.tick
    val secondSenseResult = TestSensor.getLatestSensorValue
    assert(secondSenseResult.value == Sym("x"))
    assert(secondSenseResult.sequenceId == 2)
    val thirdSenseResult = TestSensor.sense
    assert(thirdSenseResult.value == Sym("x"))
    assert(thirdSenseResult.sequenceId == 3)
  }
}
