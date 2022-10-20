package org.aiddl.common.scala.planning.state_variable

import scala.collection.mutable.Set
import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable

import org.aiddl.core.scala.container.Container

import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.planning.PlanningTerm._

class Expansion extends Function with Initializable with Configurable {
  var os: SetTerm = SetTerm()
  var addedTransitions: List[FunRef] = Nil
  val f_app = new ApplicableFunction()
  val f_trans = new StateTransition()

  def init( os: Term ): Unit = { this.os = os }
  def config( cfg: CollectionTerm, c: Container ): Unit =
    cfg.get(Sym("expand-hooks")) match {
      case Some(c) => c.asCol.foreach( f => f :: addedTransitions ) case None => {} }

  def apply( s: Term ): Term = {
    ListTerm(this.expand(s).map( (e, n) => Tuple(e, n) ))
  }

  def expand( s: Term ): Seq[(Term, Term)] = {
    os.filter( a => f_app(a, s).boolVal )
      .map( a => {
        val s_succ = addedTransitions.foldLeft(f_trans(a, s))( (s, f) => f(Tuple(a, s)) )
        (a(Name), s_succ)
      } ).toSeq
  }
}