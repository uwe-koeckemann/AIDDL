package org.aiddl.core.scala.representation

private[representation] trait StrImpl { self: Str =>


  override def +(s: Term): Str = s match { 
    case Str(s) => Str(this.value + s) 
    case _ => { println(this.toString + " + " + s); ??? }
  }

  override def toString: String = "\"" + value + "\""
  override def asStr: Str = this
}