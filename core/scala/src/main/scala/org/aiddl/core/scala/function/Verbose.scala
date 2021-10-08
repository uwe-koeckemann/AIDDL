package org.aiddl.core.scala.function

import org.aiddl.core.scala.tools.Logger

trait Verbose {
  private var verbosityLevel: Int = 0
  private var logName: String = this.getClass.getSimpleName
  private var logMethod = Logger.msg(logName, verbosityLevel) _

  def log(lvl: Int, msg: String) = this.logMethod(lvl, msg)

  def logInc(lvl: Int, msg: String) = {
    log(lvl, msg); Logger.++
  }

  def logDec(lvl: Int, msg: String) = {
    Logger.--; log(lvl, msg)
  }

  def verbosity: Int = verbosityLevel

  def setVerbose(level: Int): Unit = this.setVerbose(this.logName, level)

  def setVerbose(name: String): Unit = this.setVerbose(name, this.verbosityLevel)

  def setVerbose(name: String, level: Int): Unit = {
    this.logName = name
    this.verbosityLevel = level
    this.logMethod = Logger.msg(name, level) _
  }
}
