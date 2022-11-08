import org.aiddl.core.scala.util.logger.Logger
import org.scalatest.funsuite.AnyFunSuite

import java.util.logging.Level

class LoggerSuite extends AnyFunSuite {
  test("Logger does not eval string until it is time") {
    var checkVariable = false

    val logger = new Logger("Test", Level.INFO, _ => {checkVariable = true})


    logger.fine("Test 1")
    logger.finer("Test 2")
    logger.finest("Test 3")

    assert(!checkVariable)

    logger.info("Test 4")
    assert( checkVariable)
    checkVariable = false

    logger.warning("Test 5")
    assert(checkVariable)
    checkVariable = false

    logger.severe("Test 6")
    assert(checkVariable)
    checkVariable = false

  }
}