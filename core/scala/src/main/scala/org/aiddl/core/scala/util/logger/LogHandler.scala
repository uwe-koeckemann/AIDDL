package org.aiddl.core.scala.util.logger

import java.time.{Instant, ZoneId, ZonedDateTime}

/**
 * Collection of log-handlers
 */
object LogHandler {

  /**
   * Print a log message with its source followed by the indented Message and optional details.
   *
   * @param logEntry entry to be printed
   */
  def printBasic(logEntry: LogEntry): Unit = logEntry match {
    case LogEntry(_, _, source, depth, msg, detailed) =>
      println(s"[$source] ${"  " * depth}${msg.apply()} ${detailed.getOrElse("")}")
  }

  /**
   * Print a log message with source, level, and timestamp followed by indented message and optional details.
   * @param logEntry
   */
  def printDetailed(logEntry: LogEntry): Unit = logEntry match {
    case LogEntry(level, t, source, depth, msg, detailed) =>
      val instant = Instant.ofEpochMilli(t)
      val zonedDateTimeUtc = ZonedDateTime.ofInstant(instant, ZoneId.of("UTC"))
      println(s"[$source::$level@$zonedDateTimeUtc] ${"  " * depth} ${msg.apply()} ${detailed.getOrElse("")}")
  }
}
