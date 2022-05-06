package org.aiddl.common.scala.execution

import org.aiddl.common.scala.execution.Sensor.SeqId

trait Timestamped {

  def timestamp(sId: SeqId): Long
}
