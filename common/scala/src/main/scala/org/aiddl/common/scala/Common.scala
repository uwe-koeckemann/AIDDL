package org.aiddl.common.scala

import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Sym

object Common {
    val NIL = Sym("NIL")

    def addClassLoader(parser: Parser): Unit =
        parser.addClassLoader(Common.getClass.getClassLoader)
}