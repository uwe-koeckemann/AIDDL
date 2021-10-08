import org.scalatest.funsuite._

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.Bool
import org.aiddl.core.scala.tools.UnitTestRunner

class AiddlUnitTestSuite extends AnyFunSuite {
  test("Running AIDDL test case set") {
    UnitTestRunner.setVerbose(2)
    assert(UnitTestRunner.testFiles(scala.List("../test/test.aiddl")))
  }

  test("Running AIDDL type tase case set") {
    assert(UnitTestRunner.testFiles(scala.List("../test/test-types.aiddl")))
  }
}