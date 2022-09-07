package org.aiddl.core.scala.representation

import org.aiddl.core.scala.container.Container

import scala.language.implicitConversions
import org.aiddl.core.scala.parser.Parser

object TermParseImplicits {
  implicit def string2term(x: String): Term = {
    val c = new Container()
    val p = new Parser(c)
    p.str(x)
  }
}
