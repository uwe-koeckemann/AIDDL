package org.aiddl.core.scala.representation

import scala.annotation.targetName
import scala.language.implicitConversions

private[representation] trait BoolImpl { self: Bool =>

    /** Boolean and with another Boolean term
     * @param x Boolean
     * @return true if both this and x are true, false otherwise
     */
    @targetName("and")
    def &&(x: Bool): Bool = Bool(this.v && x.v)

    /** Boolean or with another Boolean term
     *
     * @param x Boolean
     * @return true if either this or x are true, false otherwise
     */
    @targetName("or")
    def ||(x: Bool): Bool = Bool(this.v || x.v)

    /** Negate this term
     * @return negation of this term
     */
    @targetName("not")
    def unary_! : Bool = Bool(!this.v)

    override def asBool: Bool = this
    override def asNum: Num = if (v) Num(1) else Num(0)
    override def toString: String = if (v) "true" else "false"
}


