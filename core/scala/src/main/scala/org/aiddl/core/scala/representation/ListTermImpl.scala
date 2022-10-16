package org.aiddl.core.scala.representation

import scala.collection.mutable
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.TermCollectionImplicits.seq2Tuple

import scala.annotation.targetName

private[representation] trait ListTermImpl { self: ListTerm =>
    override def isDefinedAt(n: Int): Boolean = list.isDefinedAt(n)

    override def apply(i: Int): Term = list.apply(i)
    override def apply( k: Term ): Term = this.get(k) match {
        case Some(t) => t
        case None => throw new IllegalArgumentException(s"Key $k not found in list $this")
    }

    override def length: Int = list.length

    override def iterator: Iterator[Term] = list.iterator

    private lazy val map = list.collect( { case KeyVal(k, v) => k -> v } ).toMap

    override def asCol: ListTerm = this
    override def asList: ListTerm = this
    override def asTup: Tuple = seq2Tuple(this.list)
    override def asSet: SetTerm = SetTerm(list.toSet)

    override def unify(t: Term): Option[Substitution] = t match {
        case ListTerm(t_list) if ( t_list.length == list.length  ) =>
            val init: Option[Substitution] = Some(new Substitution())
            list.zip(t_list).foldLeft(init)( (c, x) => (c flatMap (_ + (x match { case (x1, x2) => x1 unify x2 })) ))
        case _ => None
    }
    @targetName("substitute")
    override def \(s: Substitution): Term = ListTerm( list map (_ \ s) )
    override def isGround: Boolean = list.forall(_.isGround)

    override def get(k: Term): Option[Term] = this.map.get(k)
    
    override def contains(t: Term): Boolean = list.contains(t)

    override def containsAll(C: CollectionTerm): Boolean = C match {
        case ListTerm(c_list) => c_list.forall(x => list.contains(x))
        case SetTerm(c_set) => c_set.forall(x => list.contains(x))
        case _ => false
    }

    override def containsAny(C: CollectionTerm): Boolean = C match {
        case ListTerm(c_list) => c_list.exists(x => list.contains(x))
        case SetTerm(c_set) => c_set.exists(x => list.contains(x))
        case _ => false
    }

    override def containsUnifiable(t: Term): Boolean = list.exists(_ unifiable t)
    override def containsKey(k: Term): Boolean = this.map.isDefinedAt(k)
    override def add(t: Term): ListTerm = ListTerm(list.appended(t))
    override def addAll(t: CollectionTerm): ListTerm = ListTerm(list.appendedAll(t))
    override def putAll(c: CollectionTerm): CollectionTerm = 
        ListTerm(list.filter( e => (e.isInstanceOf[KeyVal] && !c.containsKey(e.asKvp.key))) 
                ++ c.filter(_.isInstanceOf[KeyVal]))
    override def put(kvp: KeyVal): CollectionTerm = 
        ListTerm(list.filter( e => (e.isInstanceOf[KeyVal] && e.asKvp.key != kvp.key)).appended(kvp))
    override def remove(t: Term): CollectionTerm = ListTerm(list.filter(_ != t))
    override def removeAll(c: CollectionTerm): CollectionTerm = ListTerm(list.filterNot(c.contains(_)))

    override def toString(): String = list.mkString("[", " ", "]")

    override def equals( other: Any ): Boolean = other match {
        case ListTerm(l) => this.list == l
        case _ => false
    }

    private lazy val myHash = 31 * this.list.##
    override def hashCode(): Int = myHash
}
