package org.aiddl.core.scala.util.logger

import java.util.logging.Level

case class LogEntry(level: Level, timeStamp: Long, source: String, message: () => String)
