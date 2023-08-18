package org.aiddl.example.learning_agent_scala

import org.aiddl.common.scala.Common
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.example.learning_agent_scala.function.HiddenModelCreator

@main def learningForPlanning = {
  val container = new Container
  val parser = new Parser(container)

  val moduleUri = parser.parseFile("./aiddl/planning-for-learning.aiddl")

  val hiddenModelCreator = new HiddenModelCreator

  def run(): Unit = {
    val model = hiddenModelCreator(Common.NIL)


  }

}