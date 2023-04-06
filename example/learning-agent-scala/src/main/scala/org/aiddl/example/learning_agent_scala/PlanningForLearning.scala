package org.aiddl.example.learning_agent_scala

import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser

@main def learningForPlanning = {
  val container = new Container
  val parser = new Parser(container)

  val moduleUri = parser.parseFile("./aiddl/learning-for-planning.aiddl")


}