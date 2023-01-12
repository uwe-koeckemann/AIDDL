package org.aiddl.core.scala.util.logger

import org.aiddl.core.scala.representation.{KeyVal, ListTerm, SetTerm, Term, Tuple}

import java.util.logging.Level
import scala.annotation.targetName
import java.time.{Instant, ZoneId, ZonedDateTime}

/**
 * Logger object used by Verbose trait
 */
object Logger {
  private def simpleTabbing(n: Int): String = "  " * n

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
          sB.append("}")
        }
        case s: ListTerm => {
          sB.append(simpleTabbing(depth))
          sB.append("[\n")
          s.foreach(e => {
            sB.append(prettyPrint(e, depth + 1)); sB.append("\n")
          })
          sB.append(simpleTabbing(depth))
          sB.append("]")
        }
        case s: Tuple => {
          sB.append(simpleTabbing(depth))
          sB.append("(\n")
          s.foreach(e => {
            sB.append(prettyPrint(e, depth + 1)); sB.append("\n")
          })
          sB.append(simpleTabbing(depth))
          sB.append(")")
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
}

/**
 * A named logger with a log-level and a handler. Used by the Verbose trait.
 *
 * The logger maintains a depth variable that can be used to display messages at
 * positive depth with an indentation.
 *
 * Detailed messages can be attached. This allows handlers to hide details behind shorted main messages
 * to make logs easier to browse (e.g., when using HTML or similar formats).
 *
 * @param name Name (e.g., of the component) of the logging entity
 * @param level Current level of the logger
 * @param handler Handler used to process log messages (e.g., format and print)
 */
class Logger( var name: String, var level: Level, var handler: LogEntry => Unit ) {
  /**
   * Depth of the log messages
   */
  var depth = 0

  /**
   * Send a severe message
   * @param msg
   */
  def severe(msg: => String): Unit =
    this.message(Level.SEVERE, depth, msg)

  /**
   * Send a warning message
   * @param msg
   */
  def warning(msg: => String): Unit =
    this.message(Level.WARNING, depth, msg)

  /**
   * Send a message at info granularity
   * @param msg
   */
  def info(msg: => String): Unit =
    this.message(Level.INFO, depth, msg)

  /**
   * Send a message at fine granularity
   * @param msg
   */
  def fine(msg: => String): Unit =
    this.message(Level.FINE, depth, msg)

  /**
   * Send a message at finer granulariy
   * @param msg
   */
  def finer(msg: => String): Unit =
    this.message(Level.FINER, depth, msg)

  /**
   * Send message at finest granularity
   * @param msg message
   */
  def finest(msg: => String): Unit =
    this.message(Level.FINEST, depth, msg)

  /**
   * Send a severe message with additional details
   *
   * @param msg         message
   * @param detailedMsg detailed message
   */
  def severe(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.SEVERE, depth, msg, Some(() => detailedMsg))

  /**
   * Send a warning message with additional details
   *
   * @param msg         message
   * @param detailedMsg detailed message
   */
  def warning(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.WARNING, depth, msg, Some(() => detailedMsg))

  /**
   * Send an info message with additional details
   *
   * @param msg         message
   * @param detailedMsg detailed message
   */
  def info(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.INFO, depth, msg, Some(() => detailedMsg))

  /**
   * Send a fine granularity message with additional details
   *
   * @param msg         message
   * @param detailedMsg detailed message
   */
  def fine(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.FINE, depth, msg, Some(() => detailedMsg))

  /**
   * Send a finer granularity message with additional details
   *
   * @param msg         message
   * @param detailedMsg detailed message
   */
  def finer(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.FINER, depth, msg, Some(() => detailedMsg))

  /**
   * Send a finest granularity message with additional details
   *
   * @param msg         message
   * @param detailedMsg detailed message
   */
  def finest(msg: => String, detailedMsg: => String): Unit =
    this.message(Level.FINEST, depth, msg, Some(() => detailedMsg))

  /**
   * Send a message by specifying all details directly.
   * @param level log-level of the message
   * @param depth depth of the message (default: 0)
   * @param msg message
   * @param detailedMsg optional detailed message
   */
  def message(level: Level, depth: Int, msg: => String, detailedMsg: Option[() => String] = None): Unit = {
    val entry = LogEntry(level, System.currentTimeMillis(), name, depth, () => msg, detailedMsg)
    if (this.level.intValue() <= level.intValue())
      handler(entry)
  }
}
