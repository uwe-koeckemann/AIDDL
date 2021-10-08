package org.aiddl.common.scala.math.graph

import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.immutable.Set
import scala.collection.immutable.HashSet
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common._

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.seq2ListTerm

class PathExtractor extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.math.graph.path-extractor")

    def apply( args: Term ): Term = args match {
        case Tuple(p, s, g) => { this(s, g, p) } case _ => ???
    }

    def apply( s: Term, g: Term, p: Function ): Term = {
        extract(s, g, p) match {
            case Some(path) if (path.length == 1) => ListTerm.empty
            case Some(path) => ListTerm(path.reverse)
            case None => NIL
        }
    }

    private def extract( s: Term, g: Term, p: Function ): Option[List[Term]] = {
        if ( s == g ) Some(List(s))
        else if (p(g) == NIL) None
        else extract(s, p(g), p) match {
            case Some(r) => Some(g :: r)
            case None => None
        }
    }
}