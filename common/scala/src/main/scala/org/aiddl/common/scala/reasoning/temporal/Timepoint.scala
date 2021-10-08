package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.representation.TermImplicits.term2Num

object Timepoint {
  val ST = Sym("ST")
  val ET = Sym("ET")

  def Est(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ST, i)).get(0)
  def Lst(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ST, i)).get(1)
  def Eet(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ET, i)).get(0)
  def Let(dom: CollectionTerm)(i: Term): Num = dom.get(Tuple(ET, i)).get(1)
}