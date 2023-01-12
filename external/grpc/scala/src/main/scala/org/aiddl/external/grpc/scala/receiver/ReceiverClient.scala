package org.aiddl.external.grpc.scala.receiver

import io.grpc.ManagedChannelBuilder
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{Num, Term}
import org.aiddl.external.grpc.aiddl.AiddlStr
import org.aiddl.external.grpc.receiver.Order.OLDEST_FIRST
import org.aiddl.external.grpc.receiver.{Order, Query, ReceiverGrpc}
import org.aiddl.external.grpc.scala.converter.Converter

class ReceiverClient(host: String, port: Int, c: Container,
                     sortBy: Order = OLDEST_FIRST,
                     pullOrder:Order = OLDEST_FIRST,
                     pullMax: Int = -1,
                     flushQueue: Boolean = false) {

  private var query: Query = _
  private val converter = new Converter(c)

  this.config_query(sortBy, pullOrder, pullMax, flushQueue)

  private val channel = ManagedChannelBuilder
    .forAddress(host, port)
    .usePlaintext()
    .build

  private val blockingStub = ReceiverGrpc.blockingStub(channel)

  def config_query(sortBy: Order = OLDEST_FIRST,
                   pullOrder:Order = OLDEST_FIRST,
                   pullMax: Int = -1,
                   flushQueue: Boolean = false): Unit = {
    this.query = Query()
      .withSortBy(sortBy)
      .withPullOrder(pullOrder)
      .withPullMax(pullMax)
      .withFlushQueue(flushQueue)
  }

  def receive: Seq[Term] = {
    val answer = blockingStub.receive(this.query)
    answer.messages.map( m => this.converter.pb2aiddl(m) )
  }
}
