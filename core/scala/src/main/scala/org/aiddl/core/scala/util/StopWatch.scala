package org.aiddl.core.scala.util

import scala.collection.mutable.HashMap
import scala.collection.mutable

/**
 * Used to measure time elapsed between a start and a stop point.
 */
object StopWatch {
    private val startTimes = new HashMap[String, Long]

    /**
     * Recorded times
     */
    val recordedTimes = new HashMap[String, List[Long]]

    /**
     * Start the stop watch for a name
     * @param name name of the interval
     */
    def start( name: String ): Unit = startTimes.put(name, System.nanoTime)

    /**
     * Stop the stop watch for a name and record elapsed time in list of all recorded times for the given name.
     * @param name name of the interval
     * @return time in seconds since start was called
     */
    def stop(  name: String ): Double = {
        val list = recordedTimes.getOrElseUpdate(name, Nil)
        val duration = (System.nanoTime - startTimes(name))
        recordedTimes.put(name, duration :: list )
        duration / 1000000000.0
    }

    /**
     * Create a summary containing the sum and number of records for every recorded name
     * @return summary as a string
     */
    def summary: String = {
        val sb = new StringBuilder
        var keys = recordedTimes.keys.toSeq
        keys.sorted.foreach( k => { 
            val sum = recordedTimes(k).foldLeft(0.0)((c, t) => c + t/1000000000.0)
            sb.addAll(s"$k: $sum s (${recordedTimes(k).size} entries)\n")})

        sb.toString
    }

}
