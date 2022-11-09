package org.aiddl.external.grpc.scala.container

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{FunRef, Sym, Term}

class GrpcFunctionLoader( c: Container ) extends Function {

  override def apply(x: Term): Term = {
    val uri = x.getOrPanic(Sym("uri")).asSym
    val host = x.getOrPanic(Sym("host")).asStr.value
    val port = x.getOrPanic(Sym("port")).intoInt

    val f = new GrpcFunction(host, port, uri, c)
    c.addFunction(uri, f)
    FunRef(uri, f)
  }

}
