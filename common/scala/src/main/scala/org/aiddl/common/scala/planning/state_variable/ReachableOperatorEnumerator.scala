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

import org.aiddl.core.scala.representation.given_Conversion_Term_KeyVal
import scala.language.implicitConversions

class ReachableOperatorEnumerator extends Function {
  def apply( Pi: Term ): Term = {
    val o = Pi.getOrPanic(Sym("operators"))
    val s0 = Pi.getOrPanic(Sym("initial-state"))
    this(o.asSet, s0.asSet)
  }

  def apply( o: SetTerm, s0: SetTerm ): Term = {
    val s_acc: Map[Term, Set[Term]] = new HashMap[Term, Set[Term]] //.withDefault( k => HashSet.empty )
    s0.foreach( sv => { val s = s_acc.getOrElseUpdate(sv.key, HashSet.empty); s_acc(sv.key).add(sv.value) } )
    val o_acc = new HashSet[Term]
    SetTerm(groundOperators(o, s0, o_acc, s_acc).toSet)
  }

  def groundOperators( os: SetTerm, s0: SetTerm, o_acc: Set[Term], s_acc: Map[Term, Set[Term]] ): Set[Term] = {
    val size_prev = o_acc.size
    val s_new = new HashMap[Term, Set[Term]]().withDefault( _ => HashSet.empty )
    os.foreach( o => {
      val o_ground = getGround(o, s_acc)
      o_ground.foreach( a => {
        a.getOrPanic(Sym("effects")).asCol.foreach( e => {
          val s = s_acc.getOrElseUpdate(e.key, HashSet.empty)
          s.add(e.value)
        })})
      o_acc.addAll(o_ground)
    })
    if ( size_prev == o_acc.size ) o_acc else groundOperators(os, s0, o_acc, s_acc)
  }


  def getGround( o: Term, s_acc: Map[Term, Set[Term]] ): Set[Term] = {
    val o_app: Set[Term] = HashSet.empty
    if ( o(Sym("name")).isGround ) {
      val pre_sat = o(Sym("preconditions")).asCol.forall( p => s_acc.contains(p.key) && s_acc(p.key).contains(p.value) )
      val unique_pre = o(Sym("preconditions")).asCol.groupBy(_.key).values.forall(_.size == 1)
      val unique_eff = o(Sym("effects")).asCol.groupBy(_.key).values.forall(_.size == 1)
      val non_redundant = o(Sym("preconditions")).asCol.forall( p => !o(Sym("effects")).asCol.contains(p))
      //if ( !unique_pre ) {                o(Sym("preconditions")).asCol.groupBy(_.key).foreach(println)            }
      //if ( !unique_eff ) {                o(Sym("effects")).asCol.groupBy(_.key).foreach(println)            }

      if (pre_sat && unique_eff && unique_pre && non_redundant) {
        // removed valid test checking that precondition and effect keys are unique
        o_app.addOne(o)
      }
      o_app
    } else {
      val selectedPre = o(Sym("preconditions")).asCol.find( p => ( !p.key.isGround || !p.value.isGround) )
      selectedPre match {
        case Some(KeyVal(sv, x)) => {
          HashSet.from(for {
            s_key <- s_acc.keys
            subKey = sv unify s_key
            if ( subKey != None )
            s_val <- s_acc(s_key)
            subVal = x unify s_val
            if ( subVal != None )
            subFull = subVal.get + subKey
            if ( subFull != None )
            o_subbed = o \ subFull.get
            o_ground <- getGround(o_subbed, s_acc)
          } yield o_ground)
        }
        case _ => o_app
      }
    }
  }
}