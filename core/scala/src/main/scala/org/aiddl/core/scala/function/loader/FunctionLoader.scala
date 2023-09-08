package org.aiddl.core.scala.function.loader

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Configurable, Function, Initializable}
import org.aiddl.core.scala.representation.{FunRef, Sym, Term}
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Sym

import scala.language.implicitConversions


protected[function] class FunctionLoader(c: Container) extends Function {
  override def apply(x: Term): Term = {
    val name = x.getOrPanic(Sym("name"))
    val module = x.getOrPanic(Sym("module"))
    val className = x.getOrPanic(Sym("class")).toString()
    val config = x.get(Sym("config"))
    val init = x.get(Sym("init"))

    val fInstance = Class.forName(className).getConstructor().newInstance().asInstanceOf[Function]
    if (config != None) {
      fInstance.asInstanceOf[Configurable].config(config.get.asCol, c)
    }
    if (init != None) {
      fInstance.asInstanceOf[Initializable].init(init.get)
    }

    val uri = module + name
    c.addFunction(uri, fInstance)
    FunRef(uri, fInstance)
  }
}
