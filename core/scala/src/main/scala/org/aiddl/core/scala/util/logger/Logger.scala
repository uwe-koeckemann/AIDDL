package org.aiddl.core.scala.util.logger

import org.aiddl.core.scala.representation.{KeyVal, ListTerm, SetTerm, Term, Tuple}

import java.util.logging.Level
import scala.annotation.targetName
import java.time.{Instant, ZoneId, ZonedDateTime}

/**
 * Logger object used by Verbose trait
 */
object Logger {
  private var depth = 0
  private var incStr = "| "

  /**
   * Increase the depth/indentation of the logger
   */
  @targetName("incDepth")
  protected[scala] def incDepth: Unit = depth += 1

  /**
   * Decrease the depth/indentation of the logger
   */
  @targetName("decDepth")
  protected[scala] def decDepth: Unit = {
    depth -= 1
    if (depth < 0) throw new IllegalStateException("Logger depth decreased below zero.")
  }

  /**
   * Send a message from a component to the logger
   *
   * @param name name of the component
   * @param v    verbosity level currently used by component
   * @param l    verbosity level of the message
   * @param msg  lazy message string (only evaluated if message is actually logged)
   */
  def msg(name: String, v: Int)(l: Int, msg: => String) =
    if (l <= v) {
      println(incStr * depth + "[" + name + "] " + msg)
    }

  /**
   * Send a message from a component to the logger and then increase depth of logger.
   *
   * @param name name of the component
   * @param v    verbosity level currently used by component
   * @param l    verbosity level of the message
   * @param msg  lazy message string (only evaluated if message is actually logged)
   */
  def msgInc(name: String, v: Int)(l: Int, msg: => String) = {
    if (l <= v) {
      println(incStr * depth + "[" + name + "] " + msg); Logger.incDepth;
    }
  }

  /**
   * Decrease the depth of the logger and then send a message from a component to the logger.
   *
   * @param name name of the component
   * @param v    verbosity level currently used by component
   * @param l    verbosity level of the message
   * @param msg  lazy message string (only evaluated if message is actually logged)
   */
  def msgDec(name: String, v: Int)(l: Int, msg: => String) = {
    if (l <= v) {
      Logger.decDepth; println(incStr * depth + "[" + name + "] " + msg);
    }
  }

  private def simpleTabbing(n: Int): String = incStr * n

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
    case LogEntry(level, t, source, msg) =>
      val instant = Instant.ofEpochMilli(t)
      val zonedDateTimeUtc = ZonedDateTime.ofInstant(instant, ZoneId.of("UTC"))
      println(s"[$source::$level@$zonedDateTimeUtc] ${msg.apply()}")
  }

  def printClassic(logEntry: LogEntry): Unit = logEntry match {
    case LogEntry(_, _, source, msg) => println(s"[$source] ${Logger.simpleTabbing(Logger.depth)}${msg.apply()}")
  }
}

class Logger( var name: String, var level: Level, var handler: LogEntry => Unit ) {
  def severe(message: => String): Unit = this.message(Level.SEVERE, message)
  def warning(message: => String): Unit = this.message(Level.WARNING, message)
  def info(message: => String): Unit = this.message(Level.INFO, message)
  def fine(message: => String): Unit = this.message(Level.FINE, message)
  def finer(message: => String): Unit = this.message(Level.FINER, message)
  def finest(message: => String): Unit = this.message(Level.FINEST, message)

  def message(level: Level, message: => String): Unit = {
    val entry = LogEntry(level, System.currentTimeMillis(), name, () => message)
    if (this.level.intValue() <= level.intValue())
      handler(entry)
  }
}
