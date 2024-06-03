package org.aiddl.common.scala.planning.state_variable

import scala.collection.mutable
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.search.GenericTreeSearch
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal
import org.aiddl.core.scala.util.{ComboIterator, StopWatch}

import scala.annotation.tailrec
import scala.collection.mutable
import scala.language.implicitConversions

object ReachableOperatorEnumerator {
  def groundProblem(problem: Term): Term = {
    val o = problem.getOrPanic(Operators).asSet
    val s0 = problem.getOrPanic(InitialState).asSet
    val g = problem.getOrPanic(Goal)

    val grounder = new ReachableOperatorEnumerator
    val actions = grounder.apply(o, s0)

    SetTerm(
      KeyVal(InitialState, s0),
      KeyVal(Goal, g),
      KeyVal(Operators, actions)
    )
  }
}

class ReachableOperatorEnumerator extends Function {
  def apply( Pi: Term ): Term = {
    val o = Pi.getOrPanic(Sym("operators"))
    val s0 = Pi.getOrPanic(Sym("initial-state"))
    this(o.asSet, s0.asSet)
  }

  def apply( o: SetTerm, s0: SetTerm ): Term = {
    val s_acc: mutable.Map[Term, mutable.Set[Term]] = new mutable.HashMap[Term, mutable.Set[Term]] //.withDefault( k => HashSet.empty )
    s0.foreach( sv => {
      val s = s_acc.getOrElseUpdate(sv.key, mutable.HashSet.empty);
      s_acc(sv.key).add(sv.value)
    } )
    val o_acc = new mutable.HashSet[Term]
    SetTerm(groundOperators(o, s0, o_acc, s_acc).toSet)
  }

  def groundProblem( problem: Term ): Term = {
    val ops = this(problem)
    problem.asCol.put(KeyVal(Operators, ops))
  }

  @tailrec
  private def groundOperators( os: SetTerm, s0: SetTerm, o_acc: mutable.Set[Term], s_acc: mutable.Map[Term, mutable.Set[Term]] ): mutable.Set[Term] = {
    val size_prev = o_acc.size
    os.foreach( o => {
      val o_ground = getGround(o, s_acc)
      o_ground.foreach( a => {
        a.getOrPanic(Sym("effects")).asCol.foreach( e => {
          val s = s_acc.getOrElseUpdate(e.key, mutable.HashSet.empty)
          s.add(e.value)
        })})
      o_acc.addAll(o_ground)
    })
    if ( size_prev == o_acc.size ) o_acc
    else groundOperators(os, s0, o_acc, s_acc)
  }

  var count = 0


  private def getGround(o: Term, s_acc: mutable.Map[Term, mutable.Set[Term]]): mutable.Set[Term] = {
    if (o(Sym("name")).isGround) {
      mutable.HashSet.from(List(o))
    } else {
      var nonGroundPre = o(Sym("preconditions")).asCol
        .filter(p => !p.isGround).toList

      val preVarMap: Map[Term, Set[Term]] =
        nonGroundPre.map( p =>
          (p, Term.collect(_.isInstanceOf[Var])(p).toSet) ).toMap
      val vars = new mutable.HashSet[Term]
      var orderedPres: List[Term] = Nil

      while (nonGroundPre != Nil) {
        nonGroundPre = nonGroundPre.sortBy( p => -preVarMap(p).count(x => !(vars contains x)) )
        vars.addAll(preVarMap(nonGroundPre.head))
        orderedPres = nonGroundPre.head :: orderedPres
        nonGroundPre = nonGroundPre.tail
      }

      orderedPres = orderedPres.reverse

      vars.clear()
      val filteredPre = orderedPres.filter(pre => {
        if (preVarMap(pre).subsetOf(vars)) false
        else {
          vars.addAll(preVarMap(pre))
          true
        }
      }).toVector

      val groundOpSearch = new GenericTreeSearch[Substitution, Substitution] {
        //traceFlag = true
        val variables = filteredPre
        val nil = new Substitution()

        override def assembleSolution(choice: List[Substitution]): Option[Substitution] =
          choice.foldLeft(Some(new Substitution()): Option[Substitution])((c, a) => c.flatMap(_ + a))

        override def expand: Option[Seq[Substitution]] = {
          if (this.choice.length < this.variables.length) {
            val selectedVar = this.variables(this.choice.length)
            val values = selectedVar match {
              case KeyVal(sv, x) => {
                List.from(for {
                  s_key <- s_acc.keys
                  subKey = sv unify s_key
                  if (subKey != None)
                  s_val <- s_acc(s_key)
                  subVal = x unify s_val
                  if (subVal != None)
                  subFull = subVal.get + subKey
                  if (subFull != None)
                } yield subFull.get)
              }
              case _ => List.empty
            }
            Some(values)
          } else {
            None
          }
        }

        override def isConsistent: Boolean = {
          val accSub = choice.foldLeft(Some(new Substitution()): Option[Substitution])((c, a) => c.flatMap(_ + a))
          accSub match {
            case Some(sub) =>
              val oSub = o \ sub
              val groundPre = oSub(Sym("preconditions")).asCol.map(_ \ sub).filter(_.isGround).toSet
              val r = groundPre.forall(p => s_acc.contains(p.key) && s_acc(p.key).contains(p.value))
                && {
                val groundEff = oSub(Sym("effects")).asCol.filter(_.isGround).toSet
                groundPre.forall(p => !groundEff.contains(p))
                  && groundEff.groupBy(_.key).values.forall(_.size == 1)
                  && groundPre.groupBy(_.key).values.forall(_.size == 1) }
              r
            case None => false
          }
        }
      }

      val r = new mutable.HashSet[Term]()

      var solution = groundOpSearch.search
      while ( solution.isDefined ) {
        r.addOne(o \ solution.get )
        solution = groundOpSearch.search
      }
      //groundOpSearch.searchGraph2File("operator-grounding.dot")
      r
    }
  }
}