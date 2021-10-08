package org.aiddl.core.scala.representation

import scala.language.implicitConversions
import org.aiddl.core.scala.parser.Parser

object TermParseImplicits {
  implicit def string2term(x: String): Term = Parser.str(x)
}
