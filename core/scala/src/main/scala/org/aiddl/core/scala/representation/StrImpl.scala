package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait StrImpl { self: Str =>

  /**
   * Concatenate this string with another string
   *
   * @param s another string
   * @return concatenated string
   */
  @targetName("concat")
  def +(s: Term): Str = s match {
    case Str(s) => Str(this.value + s) 
    case _ => { println(this.toString + " + " + s); ??? }
  }

  override def toString: String = "\"" + value + "\""
  override def asStr: Str = this
}