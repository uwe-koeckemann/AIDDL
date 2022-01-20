package org.aiddl.common.scala.reasoning.temporal

import scala.annotation.tailrec
import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL

import org.aiddl.common.scala.reasoning.temporal.Timepoint._
import org.aiddl.common.scala.reasoning.temporal.AllenConstraint._
import org.aiddl.common.scala.reasoning.temporal.IntervalDistanceConstraint._
import org.aiddl.common.scala.reasoning.temporal.UnaryConstraint._

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2SetTerm
import org.aiddl.core.scala.representation.TermCollectionImplicits.set2Term

class AllenInterval2Stp extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.reasoning.temporal.stp.solver")

    val O = Sym("#ORIGIN#")

    def st( i: Term ): Term = Tuple(ST, i)
    def et( i: Term ): Term = Tuple(ET, i)
	
    def apply( acs: Term ): Term = {
        val cs: Set[Term] = acs.flatMap( ac => {
            ac match {
                case Tuple(uc, a, Tuple(l, u)) if Set(Release, Deadline, Duration) contains uc => Set(
                    Tuple(st(a), et(a), 0, InfPos()),
                    uc match
                        case Release => Tuple(O, st(a), l, u)
                        case Deadline => Tuple(O, et(a), l, u)
                        case Duration => Tuple(st(a), et(a), l, u)
                        case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac"))
                case Tuple(At, a, Tuple(l1, u1), Tuple(l2, u2)) =>
                    Set(Tuple(O, st(a), l1, u1), Tuple(O, et(a), l2, u2), Tuple(st(a), et(a), 0, InfPos()))
                case Tuple(bc, a, b) => Set(
                    Tuple(st(a), et(a), 0, InfPos()),
                    Tuple(st(b), et(b), 0, InfPos())) ++ {
                    bc match
                        case Equals => Set(Tuple(st(a), st(b), 0, 0), Tuple(et(a), et(b), 0, 0))
                        case Meets => Set(Tuple(et(a), st(b), 0, 0), Tuple(st(a), et(a), 0, InfPos()))
                        case MetBy => Set(Tuple(et(b), st(a), 0, 0), Tuple(st(a), et(a), 0, InfPos()))
                        case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac") }
                case Tuple(bc, a, b, Tuple(l, u)) => Set(
                    Tuple(st(a), et(a), 0, InfPos()),
                    Tuple(st(b), et(b), 0, InfPos())) ++ {
                    bc match
                        case Before => Set(Tuple(et(a), st(b), l, u))
                        case After => Set(Tuple(et(b), st(a), l, u))
                        case Starts => Set(Tuple(st(a), st(b), 0, 0), Tuple(et(a), et(b), l, u))
                        case StartedBy => Set(Tuple(st(a), st(b), 0, 0), Tuple(et(b), et(a), l, u))
                        case Finishes => Set(Tuple(et(b), et(a), 0, 0), Tuple(st(b), st(a), l, u))
                        case FinishedBy => Set(Tuple(et(a), et(b), 0, 0), Tuple(st(a), st(b), l, u))
                        case Overlaps => Set(Tuple(st(a), st(b), 1, InfPos()), Tuple(et(a), et(b), 1, InfPos()), Tuple(st(b), et(a), l, u))
                        case OverlappedBy => Set(Tuple(st(b), st(a), 1, InfPos()), Tuple(et(b), et(a), 1, InfPos()), Tuple(st(a), et(b), l, u))
                        case StSt => Set(Tuple(st(a), st(b), l, u))
                        case StEt => Set(Tuple(st(a), et(b), l, u))
                        case EtSt => Set(Tuple(et(a), st(b)))
                        case EtEt => Set(Tuple(et(a), et(b), l, u))
                        case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac") }
                case Tuple(bc, a, b, Tuple(l1, u1), Tuple(l2, u2)) => Set(
                    Tuple(st(a), et(a), 0, InfPos()),
                    Tuple(st(b), et(b), 0, InfPos())) ++ {
                    bc match
                        case During => Set(Tuple(st(b), st(a), l1, u1), Tuple(et(a), et(b), l2, u2))
                        case Contains => Set(Tuple(st(a), st(b), l1, u1), Tuple(et(b), et(a), l2, u2))
                        case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac") }
                case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac")
            }
        } ).toSet

        val xs = cs.flatMap( c => Set(c(0), c(1))  ).toSet
        Tuple(xs, cs)
    }
}