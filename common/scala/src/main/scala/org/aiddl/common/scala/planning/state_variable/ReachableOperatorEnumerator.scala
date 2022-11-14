package org.aiddl.common.scala.planning.state_variable

import scala.collection.mutable.Set
import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet
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
    val s_acc: Map[Term, Set[Term]] = new HashMap[Term, Set[Term]] //.withDefault( k => HashSet.empty )
    s0.foreach( sv => { val s = s_acc.getOrElseUpdate(sv.key, HashSet.empty); s_acc(sv.key).add(sv.value) } )
    val o_acc = new HashSet[Term]
    SetTerm(groundOperators(o, s0, o_acc, s_acc).toSet)
  }

  @tailrec
  private def groundOperators( os: SetTerm, s0: SetTerm, o_acc: Set[Term], s_acc: Map[Term, Set[Term]] ): Set[Term] = {
    val size_prev = o_acc.size
    val s_new = new HashMap[Term, Set[Term]]().withDefault( _ => HashSet.empty )
    os.foreach( o => {
      val o_ground = getGround3(o, s_acc)
      o_ground.foreach( a => {
        a.getOrPanic(Sym("effects")).asCol.foreach( e => {
          val s = s_acc.getOrElseUpdate(e.key, HashSet.empty)
          s.add(e.value)
        })})
      o_acc.addAll(o_ground)
    })
    if ( size_prev == o_acc.size ) o_acc else groundOperators(os, s0, o_acc, s_acc)
  }

  var count = 0


  private def getGround3(o: Term, s_acc: Map[Term, Set[Term]]): Set[Term] = {
    if (o(Sym("name")).isGround) {
      HashSet.from(List(o))
    } else {
      val nonGroundPre = o(Sym("preconditions")).asCol
        .filter(p => !p.isGround).toList
        .sortBy(sva => {
          -Term.collect(_.isInstanceOf[Var])(sva).size
        })
      val vars = new HashSet[Term]
      val filteredPre = nonGroundPre.filter(pre => {
        val preVars = Term.collect(_.isInstanceOf[Var])(pre).toSet
        if (preVars.subsetOf(vars)) false
        else {
          vars.addAll(preVars)
          true
        }
      }) .toVector

      val groundOpSearch = new GenericTreeSearch[Substitution, Substitution] {
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
            case Some(sub) => {
              val oSub = o \ sub

              StopWatch.start("CON CHECK")
              val groundPre = oSub(Sym("preconditions")).asCol.map(_ \ sub).filter(_.isGround).toSet
              val groundEff = oSub(Sym("effects")).asCol.filter(_.isGround).toSet

              val r = groundPre.forall(p => s_acc.contains(p.key) && s_acc(p.key).contains(p.value))
                && groundPre.forall(p => !groundEff.contains(p))
                && groundEff.groupBy(_.key).values.forall(_.size == 1)
                && groundPre.groupBy(_.key).values.forall(_.size == 1)
              StopWatch.stop("CON CHECK")
              r
            }
            case None => false
          }
        }
      }


      val r = new HashSet[Term]()

      var solution = groundOpSearch.search
      while ( solution.isDefined ) {
        r.addOne(o \ solution.get )
        solution = groundOpSearch.search
      }
      r
    }
  }

  var c1 = 0
  var c2 = 0
  var c3 = 0
  var c4 = 0


  private def getGround2( o: Term, s_acc: Map[Term, Set[Term]] ): Set[Term] = {
    if ( o(Sym("name")).isGround ) {
      HashSet.from(List(o))
    } else {
      val nonGroundPre = o(Sym("preconditions")).asCol
        .filter(p => !p.isGround).toList
        .sortBy(sva => {
          Term.collect(_.isInstanceOf[Var])(sva).size
        })
      val vars = new HashSet[Term]
      val filteredPre = nonGroundPre.filter(pre => {
        val preVars = Term.collect(_.isInstanceOf[Var])(pre).toSet
        if (preVars.subsetOf(vars)) false
        else {
          vars.addAll(preVars)
          true
        }
      })

      val subPerPre: List[List[Substitution]] = filteredPre.map(pre => {
        pre match {
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
      })

      if (subPerPre.isEmpty || subPerPre.find(_.isEmpty).isDefined) HashSet.empty
      else {
        val cIter = new ComboIterator(subPerPre)
        val workingSubs = cIter.flatMap(subSeq => {
          subSeq.tail.foldLeft(Some(Substitution.from(subSeq.head.asTerm)): Option[Substitution])((c, a) => c.flatMap(_ + a))
        })
        HashSet.from(workingSubs.map(s => o \ s).filter(a => {
          (a(Sym("preconditions")).asCol.forall(p => s_acc.contains(p.key) && s_acc(p.key).contains(p.value))
            && a(Sym("effects")).asCol.groupBy(_.key).values.forall(_.size == 1)
            && a(Sym("preconditions")).asCol.groupBy(_.key).values.forall(_.size == 1)
            && a(Sym("preconditions")).asCol.forall(p => !a(Sym("effects")).asCol.contains(p)))
        })
        )
      }
    }
  }

  private def getGround( o: Term, s_acc: Map[Term, Set[Term]] ): Set[Term] = {
    count += 1
    val o_app: Set[Term] = HashSet.empty
    if ( o(Sym("name")).isGround ) {
      if (o(Sym("preconditions")).asCol.forall( p => s_acc.contains(p.key) && s_acc(p.key).contains(p.value) )
        && o(Sym("effects")).asCol.groupBy(_.key).values.forall(_.size == 1)
        && o(Sym("preconditions")).asCol.groupBy(_.key).values.forall(_.size == 1)
        && o(Sym("preconditions")).asCol.forall( p => !o(Sym("effects")).asCol.contains(p))) {
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