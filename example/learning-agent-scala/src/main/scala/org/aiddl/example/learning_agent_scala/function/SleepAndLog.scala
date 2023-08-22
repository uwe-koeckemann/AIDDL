package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.util.logger.Logger

object SleepAndLog {

  def apply(iterationNumber: Int, sleepMs: Int, logger: Logger): Unit = {
    Thread.sleep(sleepMs)
    logger.info("================================================================================")
    logger.info(s"= Iteration $iterationNumber")
    logger.info("================================================================================")
  }

}
