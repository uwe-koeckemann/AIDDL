package org.aiddl.external.grpc.scala.sender
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.grpc.scala.function.FunctionClient

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

@main def testReceiver = {
  val c = new Container
  //val receiver = new GrpcReceiver("0.0.0.0", 8063, c)
  //val answer = receiver.receive

  val rosService = new FunctionClient("0.0.0.0", 8064, c)

  println("PING")
  val answer = rosService.call(Tuple())
  println("PONG")

  println(Logger.prettyPrint(answer(Sym("info")), 0))

  println("WRITE")
  Files.write(Paths.get("file.txt"), answer.toString.getBytes(StandardCharsets.UTF_8))

  println("DONE")
}