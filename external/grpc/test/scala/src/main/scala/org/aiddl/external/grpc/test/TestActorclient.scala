package org.aiddl.external.grpc.test

import org.aiddl.external.grpc.scala.actor.GrpcActor
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.{Sym, Term}

@main def testActor = {
  val c = new Container()
  val a = new GrpcActor("0.0.0.0", 8061, c)
  a.supported(Sym("action-test"))
  a.dispatch(Sym("action-test"))
}

