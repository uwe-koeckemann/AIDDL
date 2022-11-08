package org.aiddl.core.scala.util.logger

import java.util.logging.Level

/**
 * An entry created by a logger.
 *
 * @param level log level of the entry
 * @param timeStamp timestamp in milliseconds since mid-night January 1st 1970 UTC
 * @param source logger name of the message source
 * @param depth depth of the message (for indentation or similar purposes)
 * @param msg function returning the message itself (evaluated only when printed)
 * @param detailedMsg optional detailed message (evaluated only when printed, some formats may allow hiding details under a likely shorter main message)
 */
case class LogEntry(level: Level,
                    timeStamp: Long,
                    source: String,
                    depth: Int,
                    msg: () => String,
                    detailedMsg: Option[() => String]
                   )
