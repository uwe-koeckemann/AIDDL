package org.aiddl.common.scala.simulation

class PeriodicController(n: Int) extends Tickable {
  var c = 0
  var list: List[Tickable] = Nil

 def tick = {
   if ( c == 0 )  list.foreach(_.tick)
   c = (c + 1) % n
 }
}
