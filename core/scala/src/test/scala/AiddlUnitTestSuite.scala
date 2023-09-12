import org.scalatest.funsuite._

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.Bool
import org.aiddl.core.scala.util.UnitTestRunner

class AiddlUnitTestSuite extends AnyFunSuite {
  test("Running AIDDL test case set") {
    assert(UnitTestRunner.testFiles(scala.List("/aiddl-test/test.aiddl")))
  }

  test("Running AIDDL type test case set") {
    assert(UnitTestRunner.testFiles(scala.List("/aiddl-test/test-types.aiddl")))
  }
}