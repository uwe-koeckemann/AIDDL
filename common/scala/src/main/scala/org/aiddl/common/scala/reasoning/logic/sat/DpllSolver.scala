package org.aiddl.common.scala.reasoning.logic.sat

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.GenericTreeSearch
import org.aiddl.core.scala.function.{Function, Initializable}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.conversion.{given_Conversion_Term_ListTerm, given_Conversion_Term_Num}

import scala.language.implicitConversions

class DpllSolver extends GenericTreeSearch[Term, List[Term]] with Function with Initializable {
    override def init( args: Term ) = {
        super.reset
        propagatedProblems = List(args)
        openVars = List(SetTerm(args.asCol.flatMap(c => c.asCol.map(l => l.abs) ).toSet))
        isConsistent
    }

    override def node(choices: Seq[Term]): Option[Term] =
        Some(ListTerm(
            propagatedProblems.reverse.head.asList
              .filterNot(clause =>
                clause.asList.containsAny(ListTerm(choices)))
              .map(clause => ListTerm(clause.asList.filterNot( literal => choices.contains(-literal))))))

    private def filterPhi(latestChoice: Term): ListTerm = {
        ListTerm(
            propagatedProblems.reverse.head.asList
              .filterNot(clause => clause.asList.contains(latestChoice))
              .map(clause => ListTerm(clause.asList.filterNot(literal => latestChoice == -literal)))
        )
    }

    private var propagatedProblems: List[Term] = Nil
    private var openVars: List[SetTerm] = Nil

    private def Phi = propagatedProblems.head
    private def Open: SetTerm = openVars.head

    override def assembleSolution( c: List[Term] ): Option[List[Term]] =
        Some(c)

    override def expand: Option[Seq[Term]] = {
        val unit = propagatedProblems.head.withFilter(c => c.asCol.size == 1).map(c => c(0)).toList
        if (unit.nonEmpty) {
            logger.info(s"Propagating unit literal: ${unit.head}")
            Some(ListTerm(unit.head))
        } else {
            val pure: List[Term] =
                propagatedProblems.head.flatMap(c => c.asCol.filter(_ < 0)).filter(x => !propagatedProblems.head.exists(c => c.asCol.contains(-x))).toList ++
                  propagatedProblems.head.flatMap(c => c.asCol.filter(_ > 0)).filter(x => !propagatedProblems.head.exists(c => c.asCol.contains(-x))).toList
            if (pure.nonEmpty) {
                logger.info(s"Propagating pure literal: ${pure.head}")
                Some(ListTerm(pure.head))
            } else {
                if Open.isEmpty
                then None
                else Some(ListTerm(-Open.head, Open.head))
            }
        }
    }

    override def backtrackHook: Unit = {
        //as = as.tail
        propagatedProblems = propagatedProblems.tail
        openVars = openVars.tail
    }

    override def isConsistent: Boolean = {
        var phi = Phi.asCol
        if choice.isEmpty
        then true
        else {
            phi = unitPropagate(ListTerm(choice.head), phi.asList)
            if (phi.exists(_.asCol.size == 0)) false
            else {
                propagatedProblems = phi :: propagatedProblems
                openVars = Open.remove(choice.head.asNum.abs).asSet :: openVars
                true
            }
        }
    }

    private def unitPropagate( a: CollectionTerm, phi: ListTerm ): ListTerm = 
        ListTerm(phi.withFilter( c => !c.asCol.containsAny(a) )
                    .map( c => ListTerm(c.asCol.filter( x => !a.contains(-x) ).toList )))

    override val nil: Term = ListTerm.empty

    override def apply(x: Term): Term =
        this.init(x)
        this.search match
            case Some(value) => ListTerm(value)
            case None => Common.NIL
}