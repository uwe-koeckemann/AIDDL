package org.aiddl.common.scala.search

trait Reasoned {
  lazy val reason: Option[() => String]

  def reasonStr: String = reason match {
    case Some(s) => s.apply()
    case None => "None"
  }
}
