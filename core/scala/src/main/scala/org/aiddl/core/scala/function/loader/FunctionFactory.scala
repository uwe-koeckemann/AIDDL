package org.aiddl.core.scala.function.loader

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Configurable, Function, Initializable}
import org.aiddl.core.scala.representation.{Sym, Term}
import org.aiddl.core.scala.representation.conversion.{given_Conversion_Term_Sym}

import scala.language.implicitConversions
import java.lang.reflect.Constructor

protected[function] class FunctionFactory(constructor: Constructor[Function], c: Container) extends Function {
  override def apply(x: Term): Term = {
    val uri = x match {
      case Sym(name) => x
      case _ => x(0).asSym
    }
    val init = x.get(Sym("init"))
    val config = x.get(Sym("config"))

    val f = constructor.newInstance()

    if (config != None && f.isInstanceOf[Configurable]) {
      f.asInstanceOf[Configurable].config(config.get.asCol, c)
    }
    if (init != None && f.isInstanceOf[Initializable]) {
      f.asInstanceOf[Initializable].init(init.get)
    }
    c.addFunction(uri, f)
    uri
  }
}
