package org.aiddl.core.scala.tools

import scala.collection.mutable.HashMap
import scala.collection.mutable

object StopWatch {
    val startTimes = new HashMap[String, Long]
    val recordedTimes = new HashMap[String, List[Long]]

    def start( name: String ) = startTimes.put(name, System.nanoTime)
    def stop(  name: String ): Double = {
        val list = recordedTimes.getOrElseUpdate(name, Nil)
        val duration = (System.nanoTime - startTimes(name))
        recordedTimes.put(name, duration :: list )
        duration / 1000000000.0
    }

    def summary: String = {
        val sb = new StringBuilder
        var keys = recordedTimes.keys.toSeq
        keys.sorted.foreach( k => { 
            val sum = recordedTimes(k).foldLeft(0.0)((c, t) => c + t/1000000000.0)
            sb.addAll(s"$k: $sum s (${recordedTimes(k).size} entries)\n")})

        sb.toString
    }

}
