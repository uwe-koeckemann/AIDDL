package org.aiddl.core.scala.function

import org.aiddl.core.scala.util.logger.{LogEntry, Logger}

import java.util.logging.Level
import scala.annotation.tailrec

/**
 * Trait for functions that can produce debug output
 */
trait Verbose {
  protected val logger: Logger = new Logger(this.getClass.getSimpleName(), Level.OFF, Logger.printClassic)

  def logConfig(level: Level = this.logger.level, handler: LogEntry => Unit = this.logger.handler): Unit =
    logger.level = level
    logger.handler = handler

  def logConfigRecursive(level: Level = this.logger.level, handler: LogEntry => Unit = this.logger.handler): Unit =
    this.logConfig(level, handler)
    logSubVerboseList.foreach( v => v.logConfigRecursive(level, handler) )
  def logSetName(name: String): Unit = logger.name = name
  def logSubVerboseList: List[Verbose] = Nil



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
