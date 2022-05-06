package org.aiddl.common.scala.execution.clock

class TargetFrequencyTicker(targetDur: Long) {
  var target: Tickable = _

  def run: Unit = {
    var delay: Long = 0
    var st: Long = 0
    var dur: Long = 0

    while { true } do {
      st = System.nanoTime
      target.tick
      dur = System.nanoTime - st
      delay = Math.max(0, {{targetDur - dur}/1000000}.toInt)
      Thread.sleep(delay)
    }
  }
}
