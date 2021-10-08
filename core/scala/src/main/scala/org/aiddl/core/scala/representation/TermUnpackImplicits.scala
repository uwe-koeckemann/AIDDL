package org.aiddl.core.scala.representation

import scala.language.implicitConversions

object TermUnpackImplicits {
  implicit def term2int(x: Term): Int = x.asInt.x.intValue

  implicit def term2long(x: Term): Long = x.asInt.x

  implicit def term2double(x: Term): Double = x.asReal.x.doubleValue

  implicit def term2set(x: Term): Set[Term] = x.asSet.set
}
