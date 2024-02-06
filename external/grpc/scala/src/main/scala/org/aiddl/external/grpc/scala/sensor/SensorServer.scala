package org.aiddl.external.grpc.scala.sensor

import com.google.rpc.context.AttributeContext.Request
import io.grpc.{Server, ServerBuilder}
import org.aiddl.common.scala.execution.sensor.Sensor
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Term
import org.aiddl.external.grpc.scala.converter.Converter
import org.aiddl.external.grpc.sensor.{SensorGrpc, SensorValue}
import org.aiddl.external.grpc.empty.Empty

import scala.concurrent.{ExecutionContext, Future}
import scala.concurrent.ExecutionContext.Implicits.global

class SensorServer (executionContext: ExecutionContext, port: Int, container: Container, sensor: Sensor) { self =>
  private[this] var server: Server = null
  val converter = new Converter(container)

  def start(): Unit = {
    val sensorImpl = new SensorImpl(sensor)
    server = ServerBuilder.forPort(port).addService(SensorGrpc.bindService(sensorImpl, executionContext)).build.start
    sys.addShutdownHook {
      System.err.println("*** shutting down gRPC server since JVM is shutting down")
      self.stop()
      System.err.println("*** server shut down")
    }
  }

  def stop(): Unit = {
    if (server != null) {
      server.shutdown()
    }
  }

  def blockUntilShutdown(): Unit = {
    if (server != null) {
      server.awaitTermination()
    }
  }

  private class SensorImpl(sensor: Sensor) extends SensorGrpc.Sensor {
    private val converter = new Converter(container)

    override def sense(request: Empty): Future[SensorValue] = {
      val sensorValue = self.sensor.sense
      Future(SensorValue(Some(converter.aiddl2pb(sensorValue.value)), sensorValue.sequenceId, sensorValue.nanoTimestamp))
    }

    override def getLatestSensorValue(request: Empty): Future[SensorValue] =
      val sensorValue = self.sensor.getLatestSensorValue
      Future(SensorValue(Some(converter.aiddl2pb(sensorValue.value)), sensorValue.sequenceId, sensorValue.nanoTimestamp))
  }
}
