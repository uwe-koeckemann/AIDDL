package org.aiddl.core.scala.function

import org.aiddl.core.scala.util.logger.{LogEntry, LogHandler, Logger}

import java.time.{Instant, ZoneId, ZonedDateTime}
import java.util.logging.Level
import scala.annotation.tailrec

object Verbose {
  /**
   * Handler used by Verbose components when they are constructed.
   *
   * Change this before creating any Verbose components to set a global logging handler only once.
   */
  var defaultLoggingHandler = LogHandler.printBasic

}

/**
 * Trait for functions that use a logger
 */
trait Verbose {
  protected val logger: Logger = new Logger(this.getClass.getSimpleName(), Level.OFF, Verbose.defaultLoggingHandler)



  /**
   * Set the log-level and a handler to be used by the logger. Omitted values will not change.
   * @param level level to be used
   * @param handler handler that processes log messages (e.g., by printing them)
   */
  def logConfig(level: Level = this.logger.level, handler: LogEntry => Unit = this.logger.handler): Unit =
    logger.level = level
    logger.handler = handler

  /**
   * Configure this components logger and the loggers of all its sub-components recursively
   * @param level level to be used by this logger and the loggers of all verbose sub-components
   * @param handler handler that processes log messages (e.g., by printing them)
   */
  def logConfigRecursive(level: Level = this.logger.level, handler: LogEntry => Unit = this.logger.handler): Unit =
    this.logConfig(level, handler)
    logGetVerboseSubComponents.foreach( v => v.logConfigRecursive(level, handler) )

  /**
   * Set the depth of all verbose sub-components
   * @param depth new depth
   */
  def logSetSubComponentDepth(depth: Int): Unit =
    logGetVerboseSubComponents.foreach( v => v.logger.depth = depth )

  /**
   * Set the depth of all verbose sub-component to an increment of the parent component.
   * @param increment depth of each component will be increment plus parent depth, default value is 1
   */
  def logSetSubComponentDepthRecursive(increment: Int = 1): Unit =
    logGetVerboseSubComponents.foreach(v => {
      v.logger.depth = this.logger.depth + increment
      v.logSetSubComponentDepthRecursive(increment)
    })

  /**
   * Get name used by the logger of this component
   * @return logger name
   */
  def logName: String = this.logger.name

  /**
   * Set name used by this components logger
   * @param name new name used by the logger
   */
  def logSetName(name: String): Unit =
    this.logger.name = name

  /**
   * Set name of this components logger and use the same name for all verbose sub-components
   *
   * @param name new name used by the logger and its sub-components
   */
  def logSetNameRecursive(name: String): Unit =
    logger.name = name
    logGetVerboseSubComponents.foreach( v => v.logSetNameRecursive(name) )

  /**
   * Get all verbose sub-components. Can be overwritten by components that may contain other components that are verbose.
   * This allows to recursively configure all sub-components as long as this method can provide them in a list.
   *
   * @return list of all verbose sub-components of a verbose component
   */
  def logGetVerboseSubComponents: List[Verbose] = Nil
}
