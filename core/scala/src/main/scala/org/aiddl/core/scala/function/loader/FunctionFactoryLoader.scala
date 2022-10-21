package org.aiddl.core.scala.function.loader

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Sym, Term}
import org.aiddl.core.scala.representation.conversion.{given_Conversion_Term_Sym}
import scala.language.implicitConversions

import java.lang.reflect.Constructor

protected[function] class FunctionFactoryLoader(c: Container) extends Function {
  override def apply(x: Term): Term = {
    val name = x.getOrPanic(Sym("name"))
    val module = x.getOrPanic(Sym("module"))
    val className = x.getOrPanic(Sym("class")).toString()

    val constructor = Class.forName(className).getConstructor().asInstanceOf[Constructor[Function]]

    val uri = module + name
    val f = new FunctionFactory(constructor, c)
    c.addFunction(uri, f);
    uri
  }
}
