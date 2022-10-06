package org.aiddl.core.scala.function

import org.aiddl.core.scala.tools.Logger

/**
 * Trait for functions that can produce debug output
 */
trait Verbose {
  private var verbosityLevel: Int = 0
  private var logName: String = this.getClass.getSimpleName()
  private var logMethod = Logger.msg(logName, verbosityLevel) _

  /**
   * Log a message at a log level
   * @param lvl log level of the message (higher level = more detail)
   * @param msg the message
   */
  def log(lvl: Int, msg: String) = this.logMethod(lvl, msg)

  /**
   * Log a message at a log level and then increment the indentation used by the logger
   * @param lvl log level of the message (higher level = more detail)
   * @param msg the message
   */
  def logInc(lvl: Int, msg: String) = {
    log(lvl, msg)
    if ( lvl <= this.verbosityLevel ) Logger.++
  }

  def logInc(lvl: Int) = {
    if (lvl <= this.verbosityLevel) Logger.++
  }

  /**
   * Log a message at a log level and then increment the indentation used by the logger
   * @param lvl log level of the message (higher level = more detail)
   * @param msg the message
   */
  def logDec(lvl: Int, msg: String) = {
    if ( lvl <= this.verbosityLevel ) Logger.--
    log(lvl, msg)
  }

  def logDec(lvl: Int) = {
    if (lvl <= this.verbosityLevel) Logger.--
  }

  def getLogName: String = this.logName

  /**
   * Returns the current level of verbosity.
   * @return
   */
  def verbosity: Int = verbosityLevel

  /**
   * Set the verbosity level
   * @param level target level
   */
  def setVerbose(level: Int): Unit = this.setVerbose(this.logName, level)

  /**
   * Set the logger name
   * @param name name used by logger for this function
   */
  def setVerbose(name: String): Unit = this.setVerbose(name, this.verbosityLevel)

  /**
   * Set logger name and verbosity level
   * @param name name used by logger for this function
   * @param level target level
   */
  def setVerbose(name: String, level: Int): Unit = {
    this.logName = name
    this.verbosityLevel = level
    this.logMethod = Logger.msg(name, level) _
  }
}
