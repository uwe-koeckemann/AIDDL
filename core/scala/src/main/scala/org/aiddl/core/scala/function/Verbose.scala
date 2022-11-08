package org.aiddl.core.scala.function

import org.aiddl.core.scala.util.logger.{LogEntry, Logger}

import java.util.logging.Level
import scala.annotation.tailrec

/**
 * Trait for functions that use a logger
 */
trait Verbose {
  protected val logger: Logger = new Logger(this.getClass.getSimpleName(), Level.OFF, Logger.printClassic)

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
   * Set name of this components logger
   * @param name new name used by the logger
   */
  def logSetName(name: String): Unit = logger.name = name

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



  private var verbosityLevel: Int = 0
  private var logName: String = this.getClass.getSimpleName()
  private var logMethod = Logger.msg(logName, verbosityLevel) _

  @deprecated
  def log(lvl: Int, msg: => String) = this.logMethod(lvl, msg)

  @deprecated
  def logInc(lvl: Int, msg: => String) = {
    log(lvl, msg)
    if ( lvl <= this.verbosityLevel ) Logger.incDepth
  }

  @deprecated
  def logInc(lvl: Int) = {
    if (lvl <= this.verbosityLevel) Logger.incDepth
  }

  @deprecated
   def logDec(lvl: Int, msg: => String) = {
    if ( lvl <= this.verbosityLevel ) Logger.decDepth
    log(lvl, msg)
  }

  @deprecated
  def logDec(lvl: Int) = {
    if (lvl <= this.verbosityLevel) Logger.decDepth
  }

  @deprecated
  def getLogName: String = this.logName

  @deprecated
  def verbosity: Int = verbosityLevel

  @deprecated
  def setVerbose(level: Int): Unit = this.setVerbose(this.logName, level)

  @deprecated
  def setVerbose(name: String): Unit = this.setVerbose(name, this.verbosityLevel)

  @deprecated
  def setVerbose(name: String, level: Int): Unit = {
    this.logName = name
    this.verbosityLevel = level
    this.logMethod = Logger.msg(name, level) _
  }
}
