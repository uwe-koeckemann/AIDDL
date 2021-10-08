package org.aiddl.core.scala.representation

import scala.language.implicitConversions

private[representation] trait BoolImpl { self: Bool =>

    def &&(x: Bool): Bool = Bool(this.v && x.v)
    def ||(x: Bool): Bool = Bool(this.v || x.v)
    def unary_! = Bool(!this.v)

    override def asBool: Bool = this
    override def asNum: Num = if (v) Num(1) else Num(0)
    override def toString: String = if (v) "true" else "false"
}


