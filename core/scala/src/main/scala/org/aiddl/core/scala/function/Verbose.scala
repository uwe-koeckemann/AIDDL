package org.aiddl.core.scala.function

import org.aiddl.core.scala.util.logger.{LogEntry, Logger}

import java.util.logging.Level

/**
 * Trait for functions that can produce debug output
 */
trait Verbose {
  protected val logger: Logger = new Logger(this.getClass.getSimpleName(), Level.OFF, Logger.printClassic)

  def loggerConfig(level: Level): Unit = logger.level = level
  def loggerConfig(sink: LogEntry => Unit): Unit = logger.handler = sink
  def loggerConfig(name: String): Unit = logger.name = name
  def loggerConfig(logger: Logger): Unit = this.logger.level = logger.level; this.logger.handler = logger.handler

  private var verbosityLevel: Int = 0
  private var logName: String = this.getClass.getSimpleName()
  private var logMethod = Logger.msg(logName, verbosityLevel) _

  def log(lvl: Int, msg: => String) = this.logMethod(lvl, msg)

  def logInc(lvl: Int, msg: => String) = {
    log(lvl, msg)
    if ( lvl <= this.verbosityLevel ) Logger.incDepth
  }

  def logInc(lvl: Int) = {
    if (lvl <= this.verbosityLevel) Logger.incDepth
  }

   def logDec(lvl: Int, msg: => String) = {
    if ( lvl <= this.verbosityLevel ) Logger.decDepth
    log(lvl, msg)
  }

  def logDec(lvl: Int) = {
    if (lvl <= this.verbosityLevel) Logger.decDepth
  }

  def getLogName: String = this.logName

  def verbosity: Int = verbosityLevel

  def setVerbose(level: Int): Unit = this.setVerbose(this.logName, level)

  def setVerbose(name: String): Unit = this.setVerbose(name, this.verbosityLevel)

  def setVerbose(name: String, level: Int): Unit = {
    this.logName = name
    this.verbosityLevel = level
    this.logMethod = Logger.msg(name, level) _
  }
}
