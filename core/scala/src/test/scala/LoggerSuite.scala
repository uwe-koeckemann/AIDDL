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
}