package org.aiddl.core.scala.util.logger

import org.aiddl.core.scala.representation.{KeyVal, ListTerm, SetTerm, Term, Tuple}

import java.util.logging.Level
import scala.annotation.targetName
import java.time.{Instant, ZoneId, ZonedDateTime}

/**
 * Logger object used by Verbose trait
 */
object Logger {

  private def simpleTabbing(n: Int): String = "| " * n

  /**
   * Convert a term to a string. Break long terms down to be more readable.
   *
   * @param t     term to convert
   * @param depth depth (tabbing to add in front of each line)
   * @return prettified string
   */
  def prettyPrint(t: Term, depth: Int): String = {
    val base = t.toString
    if (base.length <= 80) simpleTabbing(depth) + base
    else {
      val sB = new StringBuilder
      t match {
        case s: SetTerm => {
          sB.append(simpleTabbing(depth))
          sB.append("{\n")
          s.foreach(e => {
            sB.append(prettyPrint(e, depth + 1)); sB.append("\n")
          })
          sB.append(simpleTabbing(depth))
          sB.append("}\n")
        }
        case s: ListTerm => {
          sB.append(simpleTabbing(depth))
          sB.append("[\n")
          s.foreach(e => {
            sB.append(prettyPrint(e, depth + 1)); sB.append("\n")
          })
          sB.append(simpleTabbing(depth))
          sB.append("]\n")
        }
        case s: Tuple => {
          sB.append(simpleTabbing(depth))
          sB.append("(\n")
          s.foreach(e => {
            sB.append(prettyPrint(e, depth + 1)); sB.append("\n")
          })
          sB.append(simpleTabbing(depth))
          sB.append(")\n")
        }
        case s: KeyVal => {
          sB.append(prettyPrint(s.key, depth))
          sB.append(":\n")
          sB.append(prettyPrint(s.value, depth + 1))
        }
        case _ => {
          sB.append(simpleTabbing(depth))
          sB.append(base)
        }
      }
      sB.toString
    }
  }

  def printLog( logEntry: LogEntry ): Unit = logEntry match {
    case LogEntry(level, t, source, depth, msg, detailed) =>
      val instant = Instant.ofEpochMilli(t)
      val zonedDateTimeUtc = ZonedDateTime.ofInstant(instant, ZoneId.of("UTC"))
      println(s"[$source::$level@$zonedDateTimeUtc] ${Logger.simpleTabbing(depth)} ${msg.apply()} ${detailed.getOrElse("")}")
  }

  def printClassic(logEntry: LogEntry): Unit = logEntry match {
    case LogEntry(_, _, source, depth, msg, detailed) =>
      println(s"[$source] ${Logger.simpleTabbing(depth)}${msg.apply()} ${detailed.getOrElse("")}")
  }
}

class Logger( var name: String, var level: Level, var handler: LogEntry => Unit ) {
  var depth = 0

  def severe(msg: => String): Unit =
    this.message(Level.SEVERE, depth, msg)
  def warning(msg: => String): Unit =
    this.message(Level.WARNING, depth, msg)
  def info(msg: => String): Unit =
    this.message(Level.INFO, depth, msg)
  def fine(msg: => String): Unit =
    this.message(Level.FINE, depth, msg)
  def finer(msg: => String): Unit =
    this.message(Level.FINER, depth, msg)
  def finest(msg: => String): Unit =
    this.message(Level.FINEST, depth, msg)

  def severe(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.SEVERE, depth, msg, Some(() => detailedMsg))
  def warning(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.WARNING, depth, msg, Some(() => detailedMsg))
  def info(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.INFO, depth, msg, Some(() => detailedMsg))
  def fine(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.FINE, depth, msg, Some(() => detailedMsg))
  def finer(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.FINER, depth, msg, Some(() => detailedMsg))
  def finest(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.FINEST, depth, msg, Some(() => detailedMsg))

  def message(level: Level, depth: Int, msg: => String, detailedMsg: Option[() => String] = None): Unit = {
    val entry = LogEntry(level, System.currentTimeMillis(), name, depth, () => msg, detailedMsg)
    if (this.level.intValue() <= level.intValue())
      handler(entry)
  }
}
