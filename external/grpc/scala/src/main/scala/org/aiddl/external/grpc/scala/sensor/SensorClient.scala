package org.aiddl.external.grpc.scala.sensor

import io.grpc.ManagedChannelBuilder
import org.aiddl.common.scala.execution.sensor.Sensor
import org.aiddl.common.scala.execution.sensor.SensorMode
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Term
import org.aiddl.external.grpc.sensor.SensorGrpc
import org.aiddl.external.grpc.empty.Empty
import org.aiddl.external.grpc.scala.converter.Converter

class SensorClient (mode: SensorMode, host: String, port: Int, container: Container) extends Sensor {
  val parser = new Parser(container)
  val converter = new Converter(container)

  override val sensorMode: SensorMode = mode

  val channel = ManagedChannelBuilder
    .forAddress(host, port)
    .usePlaintext()
    .build

  val blockingStub = SensorGrpc.blockingStub(channel)
  override def performSense: Term = {
    val latest = blockingStub.sense(Empty())
    converter.pb2aiddl(latest.getValue)
  }
}
