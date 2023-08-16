package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.Verbose

object SleepAndLog extends Verbose {
  def apply(ms: Long, iteration: Int) = {
    Thread.sleep(ms)
    logger.info("================================================================================")
    logger.info(s"= Landmark $iteration")
    logger.info("================================================================================")
  }
}
