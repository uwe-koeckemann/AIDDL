package org.aiddl.core.scala.representation

import scala.language.implicitConversions
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.function.Function

import scala.collection.mutable
import org.aiddl.core.scala.container.Container

import scala.annotation.targetName

object Substitution {
  def from( c: CollectionTerm ): Substitution = {
    val s = new Substitution()
    c.foreach( e => e match { case KeyVal(k, v) => s.add(k, v) case _ => {} } )
    s
  }
}

class Substitution {
  private val map: mutable.Map[Term, Term] = new mutable.HashMap[Term, Term]()

  def this(from: Term, to: Term) = {
    this()
    this.map.put(from, to)
  }

  def add( from: Term, to: Term ): Option[Substitution] = {
    this.map.get(from) match {
      case None => this.map.addOne(from, to); Some(this)
      case Some(x) => if ( x == to ) Some(this) else None
    }
  }

  @targetName("add")
  def +(s: Substitution ): Option[Substitution] = {
    def incompatible(ft: (Term, Term)): Boolean = {
      ft match { case (from, to) =>
        s.map.get(from) match {
          case None => false
          case Some(x) => x != to
        }
      }
    }
    this.map.find(incompatible) match {
      case None => this.map.addAll(s.map); Some(this)
      case _ => None
    }
  }

  @targetName("add")
  def +(s: Option[Substitution] ): Option[Substitution] = s match { case Some(s) => this + s case None => None }

  def get( t: Term ) : Term = map.get(t) match { case Some(x) => x case None => t }

  def asTerm: SetTerm = SetTerm(map.map( (k, v) => KeyVal(k, v) ).toSet)

  def isEmpty: Boolean = this.map.isEmpty

  override def toString(): String = this.map.mkString("{", ",", "}")

  override def equals(obj: Any): Boolean = obj match {
    case o: Substitution => this.map == o.map
    case _ => false
  }
}

