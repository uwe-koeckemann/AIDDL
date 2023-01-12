package org.aiddl.external.grpc.test

import org.aiddl.core.scala.util.StopWatch
import org.aiddl.external.grpc.scala.container.ContainerServer
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{ListTerm, Num, Sym, Term}

@main def testClient = {
  val c = new Container()
  val f = new ContainerServer("0.0.0.0", 8061, Sym("id"), c)


  StopWatch.start("Create x")
  val x = ListTerm((0 until (700*800)).map(Num(_)))
  //val x = Sym("hello")
  StopWatch.stop("Create x")

  StopWatch.start("f(x)")
  val r = f(x)
  StopWatch.stop("f(x)")

  // println(r)

  println(StopWatch.summary)
}
