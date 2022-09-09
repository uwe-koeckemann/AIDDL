package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.core.scala.representation._

object Timepoint {
  val ST = Sym("ST")
  val ET = Sym("ET")

  def Est(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ST, i)).get(0).asNum
  def Lst(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ST, i)).get(1).asNum
  def Eet(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ET, i)).get(0).asNum
  def Let(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ET, i)).get(1).asNum
}