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
import scala.annotation.targetName
import scala.collection.immutable.StrictOptimizedSeqOps
import scala.collection.IndexedSeqView
import scala.reflect.ClassTag

object Term {
    /**
     * Recursively collect all terms in a term that satisfy a predicate.
     * @param p predicate defining which terms are collected
     * @param x target term
     * @return List of all terms that satisfy the predicate
     */
    def collect(p: Term => Boolean)(x: Term): List[Term] = {
        var sub = x match {
            case c: CollectionTerm => c.flatMap(y => collect(p)(y)).toList
            case t: Tuple => t.flatMap(y => collect(p)(y)).toList
            case KeyVal(k, v) => collect(p)(k) ++ collect(p)(v)
            case EntRef(mod, name, alias) => collect(p)(mod) ++ collect(p)(name) ++ collect(p)(alias)
            case _ => Nil
        }
        if p(x) then sub = x :: sub
        sub
    }
}

sealed abstract class Term extends Function {
    /**
     * Apply another term to this term.
     * @param t argument term
     * @return result of application
     */
    def apply( t: Term ):Term = { println(this); ??? }

    /**
     * Access this term at some index.
     * @param i the index
     * @return term found under the given index
     */
    def apply( i: Int ):Term = { println(this); ??? }

    /**
     * Get the length of this term if possible.
     * @return the length
     */
    def length: Int = { println(this); ??? }

    /**
     * Attempt to unify this term with another.
     * @param t another term
     * @return a substitution that makes this term equal to the other term if possible, <code>None</code> otherwise
     */
    def unify( t: Term ) : Option[Substitution] = if ( this == t ) { Some(new Substitution()) } else { None }

    /**
     * Test if this term can be unified with another term.
     * @param t another term
     * @return <code>true</code> if there exists a substitution that makes this term equal to <code>t</code>
     */
    def unifiable( t: Term ):Boolean = !(None == (this unify t))
    /**
     * Test if this term is ground. Ground terms do not contain any variables.
     * @param t another term
     * @return <code>true</code> if this term is not a variable and does not contain any variables
     */
    def isGround: Boolean = true

    /**
     * Resolve this term wrt. container <code>C</code> by replacing all references with the values of the corresponding
     * entries in <code>C</code>. Note: this will cause a stack overflow if used on an entry that references itself.
     * @param C a container used to look up entry values
     * @return a term with all entry references replaced by the values of their entries.
     */
    def resolve( C: Container ): Term = this

    /**
     * Create a key-value pair with this term as value and another term as key, if possible
     * @param key term to be used as value
     * @return a key-value term
     */
    def ::( key: Term ): KeyVal = key match {
        case x: KeyVal => throw new IllegalArgumentException("")
        case x: EntRef => ???
        case _ => KeyVal(key, this)
    }

    /**
     * Apply a substitution to this term
     *
     * @param s a substitution
     * @return term with every key that appears in the substitution replaced by its value
     */
    def \( s: Substitution ): Term = s.get(this)

    /**
     * Attempt to access this term at a key
     * @param key term we try to access
     * @return value of key if possible, <code>None</come> otherwise
     */
    def get( key: Term ): Option[Term] = { None }
    /**
     * Attempt to access this term at a key and return a default value if this is not possible
     * @param key term we try to access
     * @param e default term to return if the operation fails
     * @return value of key if possible, <code>e</come> otherwise
     */
    def getOrElse( key: Term, e: Term ): Term = get(key) match { case Some(t) => t case None => e }

    /**
     * Attempt to access this term at a key and throw an exception if this is not possible
     * @param key term we try to access
     * @return value of key if possible, throws <code>IllegalArgumentException</code> otherwise
     */
    def getOrPanic( key: Term ): Term = this.get(key) match { case Some(t) => t case None => throw new IllegalArgumentException("Key " + key + " not found in " + this) }

    /**
     * View this term as a symbol.
     * @return this term as a symbolic term
     */
    def asSym: Sym = { println(s"Cannot be viewed as Sym: $this"); ??? }
    /**
     * View this term as a Boolean term.
     * @return this term as a Boolean term
     */
    def asBool: Bool = { println(s"Cannot be viewed as Bool: $this"); ??? }
    /**
     * View this term as a variable term.
     * @return this term as a variable term
     */
    def asVar: Var = { println(s"Cannot be viewed as Var: $this"); ??? }
    /**
     * View this term as a string term.
     * @return this term as a string term
     */
    def asStr: Str = { println(s"Cannot be viewed as Str: $this"); ??? }
    /**
     * View this term as a numerical term.
     * @return this term as a numerical term
     */
    def asNum: Num = { println(s"Cannot be viewed as Num: $this"); ??? }
    /**
     * View this term as a integer term.
     * @return this term as a integer term
     */
    def asInt: Integer = { println(s"Cannot be viewed as Integer: $this"); ??? }
    /**
     * View this term as a rational term.
     * @return this term as a rational term
     */
    def asRat: Rational = { println(s"Cannot be viewed as Rational: $this"); ??? }
    /**
     * View this term as a real-valued term.
     * @return this term as a real-valued term
     */
    def asReal: Real = { println(s"Cannot be viewed as Real: $this"); ??? }
    /**
     * View this term as a key-value pair.
     * @return this term as a key-value pair
     */
    def asKvp: KeyVal = { println(s"Cannot be viewed as KeyVal: $this"); ??? }
    /**
     * View this term as a tuple term.
     * @return this term as a tuple term
     */
    def asTup: Tuple = { println(s"Cannot be viewed as Tuple: $this"); ??? }
    /**
     * View this term as a list term.
     * @return this term as a list term
     */
    def asList: ListTerm = { println(s"Cannot be viewed as ListTerm: $this"); ??? }
    /**
     * View this term as a set term.
     * @return this term as a set term
     */
    def asSet: SetTerm = { println(s"Cannot be viewed as SetTerm: $this"); ??? }
    /**
     * View this term as a collection term.
     * @return this term as a collection term
     */
    def asCol: CollectionTerm = { println(s"Cannot be viewed as CollectionTerm: $this"); ??? }
    /**
     * View this term as an entry reference term.
     * @return this term as an entry reference term
     */
    def asEntRef: EntRef = { println(s"Cannot be viewed as EntRef: $this"); ??? }
    /**
     * View this term as an function reference term.
     * @return this term as a function reference term
     */
    def asFunRef: FunRef = { println(s"Cannot be viewed as FunRef: $this"); ??? }
    /**
     * View this term as a Boolean value.
     * @return boolean value of this term
     */
    def boolVal: Boolean = this.asBool.v

    /**
     * Test if this term is a Not a Number (NaN) term.
     * @return <code>true</code> if this term is NaN, <code>false</false>
     */
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

    /**
     * Add another term to this term
     *
     * @param s another term
     * @return term resulting from addition
     */
    def +(x: Num): Num
    def -(x: Num): Num
    def unary_- : Num
    def *(x: Num): Num
    def /(x: Num): Num
    def floorDiv(x: Num): Num

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













