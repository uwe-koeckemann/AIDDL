package my.org.aiddl_sbt_project

import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser

@main def learningForPlanning = {
  val container = new Container
  val parser = new Parser(container)

  val moduleUri = parser.parseFile("./aiddl/learning-for-planning.aiddl")


}
