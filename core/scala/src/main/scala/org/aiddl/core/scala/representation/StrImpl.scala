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
    case _ => throw new IllegalArgumentException(s"Cannot concatenate non-string term $s to string term")
  }

  override def toString: String = "\"" + value + "\""
  override def asStr: Str = this

  override def tryIntoBool: Option[Bool] = Some(Bool(!self.value.isEmpty))
}