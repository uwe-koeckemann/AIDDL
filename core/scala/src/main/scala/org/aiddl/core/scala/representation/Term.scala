package org.aiddl.core.scala.representation

import scala.collection.mutable
import scala.collection.IterableOps
import scala.collection.SeqOps
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Substitution
import org.aiddl.core.scala.representation.TermImplicits.*

import scala.collection.IterableFactory
import java.util.ArrayList
import scala.collection.immutable.StrictOptimizedSeqOps
import scala.collection.IndexedSeqView
import scala.reflect.ClassTag

sealed abstract class Term extends Function {
    def apply( t: Term ):Term = { println(this); ??? }
    def apply( i: Int ):Term = { println(this); ??? }

    def length: Int = { println(this); ??? }

    def unify( t: Term ) : Option[Substitution] = if ( this == t ) { Some(new Substitution()) } else { None }
    def unifiable( t: Term ):Boolean = !(None == (this unify t))
    def isGround: Boolean = true
    def resolve( C: Container ): Term = this

    def ::( t: Term ): KeyVal = t match {
        case x: KeyVal => ???
        case x: EntRef => ???
        case _ => KeyVal(t, this)
    }

    def \( s: Substitution ): Term = s.get(this)
    def +(x: Term): Term = { println(this); ??? }
    def -(x: Term): Term = { println(this); ??? }
    def unary_- : Num = { println(this); ??? }
    def *(x: Term): Term = { println(this); ??? }
    def /(x: Term): Term = { println(this); ??? }
    def floorDiv(x: Term): Term = { println(this); ??? }

    def get( key: Term ): Option[Term] = { None }
    def getOrElse( key: Term, e: Term ): Term = get(key) match { case Some(t) => t case None => e }    
    def getOrPanic( key: Term ): Term = this.get(key) match { case Some(t) => t case None => throw new IllegalArgumentException("Key " + key + " not found in " + this) }

    def asSym: Sym = { println(this); ??? }
    def asBool: Bool = { println(this); ??? }
    def asVar: Var = { println(this); ??? }
    def asStr: Str = { println(this); ??? }
    def asNum: Num = { println(this); ??? }
    def asInt: Integer = { println(this); ??? }
    def asRat: Rational = { println(this); ??? }
    def asReal: Real = { println(s"$this of class ${this.getClass.getSimpleName}"); ??? }
    def asKvp: KeyVal = { println(this); ??? }
    def asTup: Tuple = { println(this); ??? }
    def asList: ListTerm = { println(this); ??? }
    def asSet: SetTerm = { println(this); ??? }
    def asCol: CollectionTerm = { println(this); ??? }
    def asEntRef: EntRef = { println(this); ??? }
    def asFunRef: FunRef = { println(this); ??? }
    def boolVal: Boolean = this.asBool.v

    def isNan: Boolean = this match {
        case NaN() => true
        case _ => false
    }
}

abstract class CollectionTerm extends Term with Iterable[Term] {
    def get( k: Term ): Option[Term]
    def contains( t: Term ): Boolean
    def containsAll( C: CollectionTerm ): Boolean
    def containsAny( C: CollectionTerm ): Boolean
    def containsUnifiable( t: Term ): Boolean
    def containsKey( k: Term): Boolean 
    def put( kvp: KeyVal ): CollectionTerm
    def putAll( c: CollectionTerm ): CollectionTerm
    def add( t: Term ): CollectionTerm
    def addAll( t: CollectionTerm ): CollectionTerm
    def remove( t: Term ): CollectionTerm
    def removeAll( c: CollectionTerm ): CollectionTerm
}

final case class Sym(name: String) extends Term with SymImpl
final case class Var(name: String) extends Term with VarImpl
final case class Str(value: String) extends Term with StrImpl
final case class KeyVal(key: Term, value: Term) extends Term  with KeyValImpl
final case class EntRef(mod: Sym, name: Term, alias: Sym) extends Term with EntRefImpl
final case class Tuple(x: Term*) extends Term with Seq[Term] with TupleImpl
final case class Integer(x: Long) extends Num with IntegerImpl
final case class Rational(n: Long, d: Long) extends Num with RationalImpl
final case class Real(x: Double) extends Num with RealImpl
final case class InfPos() extends Num with InfPosImpl
final case class InfNeg() extends Num with InfNegImpl
final case class NaN() extends Num with NanImpl
final case class Bool(v: Boolean) extends Term with BoolImpl

object Num {
    def apply(n: Int): Num = Integer(n.toLong)

    def apply(n: Long): Num = Integer(n)

    def apply(n: Long, d: Long): Num = Rational(n, d).shorten()

    def apply(a: Double): Num = Real(a)
}

abstract class Num extends Term with Ordered[Num] {
    override def asNum: Num = this

    override def <(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.<(y)
    override def <=(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.<=(y)
    override def >(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.>(y)
    override def >=(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.>=(y)

    def abs: Num = if (this < Num(0)) -1 * this else this
    def min(o: Num): Num = if (this <= o) this else o
    def max(o: Num): Num = if (this >= o) this else o

    def isPos: Boolean = this > Num(0)
    def isZero: Boolean = this == Num(0)
    def isNeg: Boolean = this < Num(0)

    def isInf: Boolean = this match {
        case InfPos() => true
        case InfNeg() => true
        case _ => false
    }

    def isInfPos: Boolean = this match {
        case InfPos() => true
        case _ => false
    }

    def isInfNeg: Boolean = this match {
        case InfNeg() => true
        case _ => false
    }
}

final class FunRef(val uri: Sym, lookup : Sym=>Function) extends Term {
    lazy val f = lookup(uri)

    override def apply( x: Term ): Term = f(x)
    override def unify(t: Term): Option[Substitution] = if (t.isInstanceOf[FunRef] && t.asFunRef.uri == this.uri)  Some(new Substitution()) else None
    override def \(s: Substitution): Term = FunRef.create(uri\s, lookup)
    override def toString(): String = "^" + uri.toString()
    override def asFunRef: FunRef = this

    override def equals(other: Any): Boolean = other match {
        case fr : FunRef if fr.uri == this.uri => true
        case _ => false
    }

    override def hashCode(): Int = 17 * uri.##
}

object Sym {
    private val symTable = new mutable.HashMap[String, Long]
    private var nextSymId = 0L

    private[representation] def id( s: String ): Long =
        symTable.getOrElseUpdate(s, {nextSymId += 1L; nextSymId})
}

case object Var {
    var last_id = 0

    def apply(): Var = {
        Var.last_id += 1
        Var("_" + last_id.toString())
    }
}

case object FunRef {
    def create( uri: Sym, lu: Sym=>Function ): FunRef = new FunRef(uri, lu)
    def apply( uri: Sym, f: Function ): FunRef = new FunRef(uri, _ => f)

    def unapply( fr: FunRef ): Option[(Sym, Function)] = Some((fr.uri, fr.f))
}













