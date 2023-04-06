package my.org.aiddl_sbt_project

import org.aiddl.core.scala.representation.Sym
import my.org.aiddl_sbt_project.scala.{SumAlgorithm}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser

@main def mainFunction = {
  val container = new Container
  val parser = new Parser(container)

  // Load a file as a module into the container
  val moduleUri = parser.parseFile("./aiddl/example.aiddl")
  println(s"Loaded module is named: $moduleUri")

  // Get the value of an entry in the module we loaded above
  val numberSet = container.getProcessedValueOrPanic(moduleUri, Sym("numbers"))
  println(s"Processed entry has value: $numberSet")

  val typeCheckFunction = container.getFunctionOrPanic(Sym("my.org.example.set-of-numbers"))
  println(s"Type check for $numberSet -> ${typeCheckFunction(numberSet)}")

  // Create an instance of our algorithm
  val alg = new SumAlgorithm()

  // Apply the algorithm to the number set:
  val result = alg.apply(numberSet)
  println(s"$numberSet -> $result")

  val numberAndLetterSet = container.getProcessedValueOrPanic(moduleUri, Sym("numbers-and-letters"))
  println(s"Processed entry has value: $numberAndLetterSet")
  println(s"Type check for $numberAndLetterSet -> ${typeCheckFunction(numberAndLetterSet)}")
}
