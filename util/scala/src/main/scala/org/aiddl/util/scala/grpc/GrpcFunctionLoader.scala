package org.aiddl.util.scala.grpc

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{FunRef, Sym, Term}

class GrpcFunctionLoader( c: Container ) extends Function {

  override def apply(x: Term): Term = {
    val uri = x.getOrPanic(Sym("uri")).asSym
    val host = x.getOrPanic(Sym("host")).asStr.value
    val port = x.getOrPanic(Sym("port")).asInt.toInt

    val f = new GrpcFunction(host, port, uri, new Parser(c))
    c.addFunction(uri, f)
    FunRef(uri, f)
  }

}
