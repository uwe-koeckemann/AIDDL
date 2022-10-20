package org.aiddl.core.scala.representation

import scala.language.implicitConversions

object BoolImplicits {
  @deprecated
  implicit def bool2Boolean(x: Bool): Boolean = x.v
  @deprecated
  implicit def boolean2Bool(x: Boolean): Bool = Bool(x)
  @deprecated
  implicit def term2Boolean(x: Term): Boolean = term2Bool(x).v
  @deprecated
  implicit def term2Bool(x: Term): Bool = {
    x match {
      case ListTerm.empty => Bool(false)
      case SetTerm(s) if s.isEmpty => Bool(false)
      case Tuple() => Bool(false)
      case Integer(0) => Bool(false)
      case Real(0.0) => Bool(false)
      case Rational(0, 1) => Bool(false)
      case Str("") => Bool(false)
      case Bool(v) => Bool(v)
      case _ => Bool(true)
    }
  }
}
