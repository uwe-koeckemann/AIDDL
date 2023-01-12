package org.aiddl.core.scala.representation

import scala.collection.mutable

import scala.annotation.targetName

private[representation] trait SetTermImpl { self: SetTerm =>
    private lazy val map = set.collect( { case KeyVal(k, v) => k -> v } ).toMap

    override def apply( n: Int ): Term = this.set.toList(n)

    override def apply( k: Term ): Term = this.get(k) match {
        case Some(t) => t
        case None => throw new IllegalArgumentException(s"Key $k not found in set $this")
    }

    override def length: Int = set.size

    override def iterator: Iterator[Term] = set.iterator

    override def asCol: SetTerm = this
    override def asSet: SetTerm = this
    override def asList: ListTerm = ListTerm(set.toList)

    override def get(k: Term): Option[Term] = map.get(k)

    override def contains(t: Term): Boolean = set.contains(t)

    override def containsAll(C: CollectionTerm): Boolean = C match {
        case ListTerm(c_list) => c_list.forall(x => set.contains(x))
        case SetTerm(c_set) => c_set.forall(x => set.contains(x))
        case _ => false
    }

    override def containsAny(C: CollectionTerm): Boolean = C match {
        case ListTerm(c_list) => c_list.exists(x => set.contains(x))
        case SetTerm(c_set) => c_set.exists(x => set.contains(x))
        case _ => false
    }
    override def containsUnifiable(t: Term): Boolean = this.set.exists(_ unifiable t)
    override def containsKey(k: Term): Boolean = this.map.isDefinedAt(k)
    
    override def add(t: Term): SetTerm = SetTerm(set + t)
    override def addAll(t: CollectionTerm): SetTerm = SetTerm(set ++ t)
    override def putAll(c: CollectionTerm): CollectionTerm = 
        SetTerm(set.filter( e => (e.isInstanceOf[KeyVal] && !c.containsKey(e.asKvp.key))) 
                ++ c.filter(_.isInstanceOf[KeyVal]))
    override def put(kvp: KeyVal): CollectionTerm = 
        SetTerm(set.filter( e => (e.isInstanceOf[KeyVal] && e.asKvp.key != kvp.asKvp.key))
                + kvp)                
    override def remove(t: Term): CollectionTerm = SetTerm(set - t)
    override def removeAll(c: CollectionTerm): CollectionTerm = SetTerm(set -- c)
    @targetName("substitute")
    override def \(s: Substitution): Term = SetTerm( set map (_ \ s) )
    override def isGround: Boolean = set.forall(_.isGround)

    override def toString(): String = set.mkString("{", " ", "}")

    private lazy val myHash = 17 * this.set.## 
    override def hashCode(): Int = myHash
}