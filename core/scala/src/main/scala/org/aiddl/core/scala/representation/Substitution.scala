package org.aiddl.core.scala.representation

import org.aiddl.core.scala.function.Function
import scala.collection.mutable
import org.aiddl.core.scala.container.Container
import scala.annotation.targetName

object Substitution {
  /**
   * Create a substitution from a collection of key value terms
   * @param c collection of key-value terms
   * @return substutution that maps keys in c to their values
   */
  def from( c: CollectionTerm ): Substitution = {
    val s = new Substitution()
    c.foreach( e => e match { case KeyVal(k, v) => s.add(k, v) case _ => {} } )
    s
  }
}

/**
 * Substitutions can be used to replace occurrences of terms in other terms
 */
class Substitution {
  private val map: mutable.Map[Term, Term] = new mutable.HashMap[Term, Term]()

  /**
   * Create new substitution containing a single replacement
   * @param from term to replace
   * @param to replacement term
   */
  def this(from: Term, to: Term) = {
    this()
    this.map.put(from, to)
  }

  /**
   * Add a new replacement
   * @param from term to replace
   * @param to replacement term
   * @return
   */
  def add( from: Term, to: Term ): Option[Substitution] = {
    this.map.get(from) match {
      case None => this.map.addOne(from, to); Some(this)
      case Some(x) =>
        if x == to
        then Some(this)
        else None
    }
  }

  /**
   * Add another substitution to this one if possible. If not possible, this substitution will not change.
   * @param s another substitution
   * @return combination of this and s if possible, None otherwise
   */
  @targetName("add")
  def +(s: Substitution ): Option[Substitution] = {
    def incompatible(ft: (Term, Term)): Boolean = {
      val (from, to) = ft
      s.map.get(from) match {
        case None => false
        case Some(x) => x != to
      }
    }
    this.map.find(incompatible) match {
      case None => this.map.addAll(s.map); Some(this)
      case _ => None
    }
  }

  /**
   * Add an optional substitution to this one
   * @param s optional substitution
   * @return optional substitution if possible, None otherwise
   */
  @targetName("add")
  def +(s: Option[Substitution] ): Option[Substitution] = s match { case Some(s) => this + s case None => None }

  /**
   * Get replacement for a term according to a substitution
   * @param t term to replace
   * @return replacement if specified in this substitution, t otherwise
   */
  def get( t: Term ) : Term = map.get(t) match { case Some(x) => x case None => t }

  /**
   * Turn this substitution into a collection of key-value pairs
   * @return collection term containing all replacements as key-value pairs
   */
  def asTerm: SetTerm = SetTerm(map.map( (k, v) => KeyVal(k, v) ).toSet)

  /**
   * Check if this substitution is emtpy
   * @return true if this substitution does not contain any replacements, false otherwise
   */
  def isEmpty: Boolean = this.map.isEmpty

  override def toString(): String = this.map.mkString("{", ",", "}")

  override def equals(obj: Any): Boolean = obj match {
    case o: Substitution => this.map == o.map
    case _ => false
  }
}

