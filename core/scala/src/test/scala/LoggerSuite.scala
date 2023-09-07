import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.util.logger.Logger
import org.scalatest.funsuite.AnyFunSuite

import java.util.logging.Level

class LoggerSuite extends AnyFunSuite {
  test("Logger does not call handler if log level is not enough") {
    var checkVariable = false

    val logger = new Logger("Test", Level.INFO, _ => {
      checkVariable = true
    })
    logger.fine("Test 1")
    logger.finer("Test 2")
    logger.finest("Test 3")

    assert(!checkVariable)

    logger.info("Test 4")
    assert(checkVariable)
    checkVariable = false

    logger.warning("Test 5")
    assert(checkVariable)
    checkVariable = false

    logger.severe("Test 6")
    assert(checkVariable)
    checkVariable = false
  }

  test("Logger does not evaluate string if the log level is not enough") {
    var checkVariable = false

    val logger = new Logger("Test", Level.INFO, _ => {
      checkVariable = true
    })

    logger.fine(s"This ${checkVariable = true; "is"} a test")
    assert(!checkVariable)

    logger.info(s"This ${checkVariable = true; "is"} a test")
    assert( checkVariable)
    checkVariable = false
  }

  test("Verbose function configuration") {
    object TestFunction extends Verbose {
      object SubComponent extends Verbose {
        object SubSubComponent extends Verbose {
          def getLevel: Level = this.logger.level

          def getDepth: Int = this.logger.depth
        }
        override def logGetVerboseSubComponents: List[Verbose] = List(SubSubComponent)

        def getLevel: Level = this.logger.level
        def getDepth: Int = this.logger.depth


      }
      override def logGetVerboseSubComponents: List[Verbose] = List(SubComponent)
      def getLevel: Level = this.logger.level
      def getDepth: Int = this.logger.depth
    }

    TestFunction.logConfigRecursive(level=Level.FINEST)
    assert(TestFunction.getLevel == Level.FINEST)
    assert(TestFunction.SubComponent.getLevel == Level.FINEST)
    assert(TestFunction.SubComponent.SubSubComponent.getLevel == Level.FINEST)

    TestFunction.logConfig(level=Level.FINER)
    assert(TestFunction.getLevel == Level.FINER)

    assert(TestFunction.getDepth == 0)
    assert(TestFunction.SubComponent.getDepth == 0)
    TestFunction.logSetSubComponentDepthRecursive(2)
    assert(TestFunction.getDepth == 0)
    assert(TestFunction.SubComponent.getDepth == 2)
    assert(TestFunction.SubComponent.SubSubComponent.getDepth == 4)

    TestFunction.logSetSubComponentDepth(3)
    assert(TestFunction.getDepth == 0)
    assert(TestFunction.SubComponent.getDepth == 3)
    assert(TestFunction.SubComponent.SubSubComponent.getDepth == 4)

    TestFunction.logSetSubComponentDepthRecursive()
    assert(TestFunction.getDepth == 0)
    assert(TestFunction.SubComponent.getDepth == 1)

    TestFunction.logSetSubComponentDepthRecursive()
    assert(TestFunction.getDepth == 0)
    assert(TestFunction.SubComponent.getDepth == 1)

    TestFunction.logSetNameRecursive("A")
    assert(TestFunction.logName == "A")
    assert(TestFunction.SubComponent.logName == "A")
    TestFunction.logSetName("B")
    assert(TestFunction.logName == "B")

  }
}