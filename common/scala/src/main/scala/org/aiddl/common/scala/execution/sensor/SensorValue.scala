package org.aiddl.common.scala.execution.sensor

import org.aiddl.core.scala.representation.Term

case class SensorValue(value: Term, sequenceId: Long, nanoTimestamp: Long)
