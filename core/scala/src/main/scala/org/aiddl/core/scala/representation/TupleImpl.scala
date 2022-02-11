package org.aiddl.core.scala.representation

import scala.collection.mutable

import org.aiddl.core.scala.representation.TermImplicits._

private[representation] trait TupleImpl { self: Tuple =>
    private lazy val map = x.collect( { case KeyVal(k, v) => k -> v } ).toMap

    override def iterator: Iterator[Term] = x.iterator

    override def length: Int = x.length
    override def apply(k: Term): Term = map.get(k) match {
        case Some(t) => t
        case None => throw new IllegalArgumentException(s"Key $k not found in tuple $this")
    }

    override def apply(n: Int): Term = x(n)
   
    override def get(k: Term): Option[Term] = { map.get(k) }

    override def unify(t: Term): Option[Substitution] = t match {
        case Tuple(args @ _*) if ( args.length == x.length  ) =>
            val init: Option[Substitution] = Some(new Substitution())
            x.zip(args).foldLeft(init)( (c, x) => (c flatMap (_ + (x match { case (x1, x2) => x1 unify x2 })) ))
        case _ => None
    }

    override def \(s: Substitution): Term = Tuple(x.map(_\s): _*)
    override def isGround: Boolean = x.forall(_.isGround)

    def put(kvp: KeyVal): Tuple = 
        Tuple(x.filter( e => (e.isInstanceOf[KeyVal] && e.key != kvp.key)).appended(kvp): _*)

    override def toString(): String = x.mkString("(", " ", ")")

    override def asTup: Tuple = this
    override  def asList: ListTerm = ListTerm(this.x)
    override def asCol: CollectionTerm = ListTerm(this.x)

    override def equals( other: Any ): Boolean = other match {
        case Tuple(l @ _*) => this.x == l
        case _ => false
    }

    private lazy val myHash = 19 * this.x.##
    override def hashCode(): Int = myHash
}