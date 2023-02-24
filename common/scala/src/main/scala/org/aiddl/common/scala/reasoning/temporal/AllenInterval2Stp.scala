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

class AllenInterval2Stp extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.reasoning.temporal.stp.solver")

    val O = Sym("#ORIGIN#")

    def st( i: Term ): Term = Tuple(ST, i)
    def et( i: Term ): Term = Tuple(ET, i)
	
    def apply( acs: Term ): Term = {
        val cs: SetTerm = SetTerm(acs.asCol.flatMap( this.convertSingleAc(_) ).toSet)
        val xs = SetTerm(cs.flatMap( c => Set(c(0), c(1))  ).toSet)
        Tuple(xs, cs)
    }

    def convertSingleAc(ac: Term): Set[Term] = {
        ac match {
            case Tuple(uc, a, Tuple(l, u)) if Set(Release, Deadline, Duration) contains uc => Set(
                Tuple(st(a), et(a), Num(0), InfPos()),
                uc match
                    case Release => Tuple(O, st(a), l, u)
                    case Deadline => Tuple(O, et(a), l, u)
                    case Duration => Tuple(st(a), et(a), l, u)
                    case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac"))
            case Tuple(At, a, Tuple(l1, u1), Tuple(l2, u2)) =>
                Set(Tuple(O, st(a), l1, u1), Tuple(O, et(a), l2, u2), Tuple(st(a), et(a), Num(0), InfPos()))
            case Tuple(bc, a, b) => Set(
                Tuple(st(a), et(a), Num(0), InfPos()),
                Tuple(st(b), et(b), Num(0), InfPos())) ++ {
                bc match
                    case Equals => Set(Tuple(st(a), st(b), Num(0), Num(0)), Tuple(et(a), et(b), Num(0), Num(0)))
                    case Meets => Set(Tuple(et(a), st(b), Num(0), Num(0)), Tuple(st(a), et(a), Num(0), InfPos()))
                    case MetBy => Set(Tuple(et(b), st(a), Num(0), Num(0)), Tuple(st(a), et(a), Num(0), InfPos()))
                    case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac")
            }
            case Tuple(bc, a, b, Tuple(l, u)) => Set(
                Tuple(st(a), et(a), Num(0), InfPos()),
                Tuple(st(b), et(b), Num(0), InfPos())) ++ {
                bc match
                    case Before => Set(Tuple(et(a), st(b), l, u))
                    case After => Set(Tuple(et(b), st(a), l, u))
                    case Starts => Set(Tuple(st(a), st(b), Num(0), Num(0)), Tuple(et(a), et(b), l, u))
                    case StartedBy => Set(Tuple(st(a), st(b), Num(0), Num(0)), Tuple(et(b), et(a), l, u))
                    case Finishes => Set(Tuple(et(b), et(a), Num(0), Num(0)), Tuple(st(b), st(a), l, u))
                    case FinishedBy => Set(Tuple(et(a), et(b), Num(0), Num(0)), Tuple(st(a), st(b), l, u))
                    case Overlaps => Set(Tuple(st(a), st(b), Num(1), InfPos()), Tuple(et(a), et(b), Num(1), InfPos()), Tuple(st(b), et(a), l, u))
                    case OverlappedBy => Set(Tuple(st(b), st(a), Num(1), InfPos()), Tuple(et(b), et(a), Num(1), InfPos()), Tuple(st(a), et(b), l, u))
                    case StSt => Set(Tuple(st(a), st(b), l, u))
                    case StEt => Set(Tuple(st(a), et(b), l, u))
                    case EtSt => Set(Tuple(et(a), st(b)))
                    case EtEt => Set(Tuple(et(a), et(b), l, u))
                    case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac")
            }
            case Tuple(bc, a, b, Tuple(l1, u1), Tuple(l2, u2)) => Set(
                Tuple(st(a), et(a), Num(0), InfPos()),
                Tuple(st(b), et(b), Num(0), InfPos())) ++ {
                bc match
                    case During => Set(Tuple(st(b), st(a), l1, u1), Tuple(et(a), et(b), l2, u2))
                    case Contains => Set(Tuple(st(a), st(b), l1, u1), Tuple(et(b), et(a), l2, u2))
                    case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac")
            }
            case _ => throw new IllegalArgumentException(s"Constraint not supported: $ac")
        }
    }

    /*def inverseMap(acs: CollectionTerm): Map[Term, Term] = {
        var m: Map[Term, Term] = Map.empty
        for ( ac <- acs ) {
            val dcs = this.convertSingleAc(ac)
            for ( dc <- dcs ) {
                m = m.update(dc, ac)
            }
        }
        m
    }*/
}