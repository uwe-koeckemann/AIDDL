import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.Actor.ActionInstanceId
import org.aiddl.common.scala.execution.Actor.Status
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.external.grpc.scala.actor.{ActorClient, ActorServer}
import org.aiddl.external.grpc.scala.converter.Converter
import org.scalatest.funsuite.AnyFunSuite

import scala.concurrent.ExecutionContext


class ActorSuite extends AnyFunSuite {
  val serverContainer = new Container
  val clientContainer = new Container


  object TestActor extends Actor {
    override def supported(action: Term): Boolean =
      action != Sym("not-supported")

    override def dispatch(action: Term): Option[ActionInstanceId] =
      if action != Sym("not-supported")
      then Some(super.nextIdWithStatus(Status.Succeeded))
      else None

    override def tick: Unit = {}
  }

  test("Actor server and client talk to each other as intended") {
    val server = new ActorServer(ExecutionContext.global, 11011, serverContainer, TestActor)
    server.start()

    val client = new ActorClient("localhost", 11011, clientContainer)

    assert(client.supported(Sym("action")))
    val idOption = client.dispatch(Sym("action"))
    assert(idOption.isDefined)
    assert(client.getStatus(idOption.get) == Status.Succeeded)

    assert(!client.supported(Sym("not-supported")))
    assert(client.dispatch(Sym("not-supported")).isEmpty)

    server.stop()
  }
}