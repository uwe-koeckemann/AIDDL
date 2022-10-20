package org.aiddl.core.scala.representation

import scala.collection.mutable
import scala.collection.IterableOps
import scala.collection.SeqOps
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Substitution

import scala.collection.IterableFactory
import java.util.ArrayList
import scala.annotation.targetName
import scala.collection.immutable.StrictOptimizedSeqOps
import scala.collection.IndexedSeqView
import scala.reflect.ClassTag
import scala.language.implicitConversions

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

    given Conversion[Term, KeyVal] = _.asKvp
    given Conversion[Term, Sym] = _.asSym
    given Conversion[Term, Num] = _.asNum
    given Conversion[Term, CollectionTerm] = _.asCol
    given Conversion[Term, SetTerm] = _.asSet
    given Conversion[Term, ListTerm] = _.asList
    given Conversion[Term, Tuple] = _.asTup
}

/**
 * Main class for every AIDDL expression
 */
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
    @targetName("intoKeyVal")
    def ::( key: Term ): KeyVal = key match {
        case x: KeyVal => throw new IllegalArgumentException(s"$x is not a legal key for a key-value pair (KeyVal not allowed as key).")
        case x: EntRef => throw new IllegalArgumentException(s"$x is not a legal key for a key-value pair (EntRef not allowed as key).")
        case _ => KeyVal(key, this)
    }

    /**
     * Apply a substitution to this term
     * @param s a substitution
     * @return term with every key that appears in the substitution replaced by its value
     */
    @targetName("substitute")
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
    def asSym: Sym = { throw new IllegalAccessError(s"Cannot be viewed as Sym: $this") }
    /**
     * View this term as a Boolean term.
     * @return this term as a Boolean term
     */
    def asBool: Bool = { throw new IllegalAccessError(s"Cannot be viewed as Bool: $this") }
    /**
     * View this term as a variable term.
     * @return this term as a variable term
     */
    def asVar: Var = { throw new IllegalAccessError(s"Cannot be viewed as Var: $this") }
    /**
     * View this term as a string term.
     * @return this term as a string term
     */
    def asStr: Str = { throw new IllegalAccessError(s"Cannot be viewed as Str: $this") }
    /**
     * View this term as a numerical term.
     * @return this term as a numerical term
     */
    def asNum: Num = { throw new IllegalAccessError(s"Cannot be viewed as Num: $this") }
    /**
     * View this term as a integer term.
     * @return this term as a integer term
     */
    def asInt: Integer = { throw new IllegalAccessError(s"Cannot be viewed as Integer: $this") }
    /**
     * View this term as a rational term.
     * @return this term as a rational term
     */
    def asRat: Rational = { throw new IllegalAccessError(s"Cannot be viewed as Rational: $this") }
    /**
     * View this term as a real-valued term.
     * @return this term as a real-valued term
     */
    def asReal: Real = { throw new IllegalAccessError(s"Cannot be viewed as Real: $this") }
    /**
     * View this term as a key-value pair.
     * @return this term as a key-value pair
     */
    def asKvp: KeyVal = { throw new IllegalAccessError(s"Cannot be viewed as KeyVal: $this") }
    /**
     * View this term as a tuple term.
     * @return this term as a tuple term
     */
    def asTup: Tuple = { throw new IllegalAccessError(s"Cannot be viewed as Tuple: $this") }
    /**
     * View this term as a list term.
     * @return this term as a list term
     */
    def asList: ListTerm = { throw new IllegalAccessError(s"Cannot be viewed as ListTerm: $this") }
    /**
     * View this term as a set term.
     * @return this term as a set term
     */
    def asSet: SetTerm = { throw new IllegalAccessError(s"Cannot be viewed as SetTerm: $this") }
    /**
     * View this term as a collection term.
     * @return this term as a collection term
     */
    def asCol: CollectionTerm = { throw new IllegalAccessError(s"Cannot be viewed as CollectionTerm: $this") }
    /**
     * View this term as an entry reference term.
     * @return this term as an entry reference term
     */
    def asEntRef: EntRef = { throw new IllegalAccessError(s"Cannot be viewed as EntRef: $this") }
    /**
     * View this term as an function reference term.
     * @return this term as a function reference term
     */
    def asFunRef: FunRef = { throw new IllegalAccessError(s"Cannot be viewed as FunRef: $this") }
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

/**
 * A symbolic term
 * @param name name of the symbol
 */
final case class Sym(name: String) extends Term with SymImpl

/**
 * A variable term
 * @param name name of the variable
 */
final case class Var(name: String) extends Term with VarImpl

/**
 * A string term
 * @param value the value of the string
 */
final case class Str(value: String) extends Term with StrImpl

/**
 * A key value pair
 * @param key key term (must not be KeyVal or EntRef)
 * @param value value term
 */
final case class KeyVal(key: Term, value: Term) extends Term  with KeyValImpl

/**
 * A reference to an entry in a module
 * @param mod module name
 * @param name name of the entry
 * @param alias alias used to refer to the module in another module
 */
final case class EntRef(mod: Sym, name: Term, alias: Sym) extends Term with EntRefImpl

/**
 * A tuple term
 * @param x zero or more element terms of the tuple
 */
final case class Tuple(x: Term*) extends Term with Seq[Term] with TupleImpl

/**
 * An integer term
 * @param x value of the term
 */
final case class Integer(x: Long) extends Num with IntegerImpl

/**
 * A rational term
 * @param n nominator
 * @param d denominator
 */
final case class Rational(n: Long, d: Long) extends Num with RationalImpl

/**
 * A real valued term
 * @param x value of the term
 */
final case class Real(x: Double) extends Num with RealImpl

/**
 * Term representing positive infinity
 */
final case class InfPos() extends Num with InfPosImpl

/**
 * Term representing negative infinity
 */
final case class InfNeg() extends Num with InfNegImpl

/**
 * Term representing not-a-number
 */
final case class NaN() extends Num with NanImpl

/**
 * Term representing a Boolean value
 * @param v boolean value
 */
final case class Bool(v: Boolean) extends Term with BoolImpl

/**
 * Providing various shortcuts for creating numerical terms
 */
object Num {
    /**
     * Create an integer term from an integer
     * @param n integer value
     * @return integer term
     */
    def apply(n: Int): Num = Integer(n.toLong)

    /**
     * Create an integer term from a long
     *
     * @param n long value
     * @return integer term
     */
    def apply(n: Long): Num = Integer(n)

    /**
     * Create a rational term
     * @param n nominator
     * @param d denominator
     * @return rational term
     */
    def apply(n: Long, d: Long): Num = Rational(n, d).shorten()

    /**
     * Create an real term from a float
     *
     * @param n float value
     * @return real term
     */
    def apply(a: Float): Num = Real(a.toDouble)

    /**
     * Create an real term from a double
     *
     * @param n double value
     * @return real term
     */
    def apply(a: Double): Num = Real(a)
}

/**
 * Abstract class to cover common methods and behavior for all numerical terms
 */
abstract class Num extends Term with Ordered[Num] {
    override def asNum: Num = this

    override def <(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.<(y)
    override def <=(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.<=(y)
    override def >(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.>(y)
    override def >=(y: Num): Boolean = if ( this.isNan || y.isNan ) false else super.>=(y)

    /** Add another numerical term to this term
     *
     * @param x another term
     * @return this + x
     */
    @targetName("plus")
    def +(x: Num): Num

    /** Subtract another term to this term
     *
     * @param x another term
     * @return this - x
     */
    @targetName("minus")
    def -(x: Num): Num

    /** Negate this term
     *
     * @return -this
     */
    @targetName("negate")
    def unary_- : Num

    /** Multiply this term with another numerical term
     *
     * @param x another term
     * @return this * x
     */
    @targetName("times")
    def *(x: Num): Num

    /** Divide this term by another numerical term
     *
     * @param x another term
     * @return this / x
     */
    @targetName("dividedBy")
    def /(x: Num): Num

    /** Divide this term by another numerical term and round down to nearest integer
     *
     * @param x another term
     * @return floor(this / x)
     */
    def floorDiv(x: Num): Num

    /**
     * Get the absolute value of this numerical term
     * @return -this if this is negative, this otherwise
     */
    def abs: Num = if (this < Num(0)) -1 * this else this

    /** Get minimum of this term and another
     * @param o numerical term
     * @return smaller of the two terms
     */
    def min(o: Num): Num = if (this <= o) this else o

    /** Get maximum of this term and another
     *
     * @param o numerical term
     * @return larger of the two terms
     */
    def max(o: Num): Num = if (this >= o) this else o

    /** Test if numerical term is positive
     * @return true of positive, false otherwise
     */
    def isPos: Boolean = this > Num(0)

    /** Test if numerical term is zero
     *
     * @return true of zero, false otherwise
     */
    def isZero: Boolean = this == Num(0)

    /** Test if numerical term is negative
     *
     * @return true of negative, false otherwise
     */
    def isNeg: Boolean = this < Num(0)

    /** Test if this term is infinite
     * @return true if this term is positive of negative infinity, false otherwise
     */
    def isInf: Boolean = this match {
        case InfPos() => true
        case InfNeg() => true
        case _ => false
    }

    /** Test if this term is positive infinity
     *
     * @return true if this term is positive infinity, false otherwise
     */
    def isInfPos: Boolean = this match {
        case InfPos() => true
        case _ => false
    }

    /** Test if this term is negative infinity
     *
     * @return true if this term is negative infinity, false otherwise
     */
    def isInfNeg: Boolean = this match {
        case InfNeg() => true
        case _ => false
    }

    /** Attempt to convert this numerical to an Int value
     *
     * @return integer value
     */
    def toInt: Int

    /** Attempt to convert this numerical to an Long value
     *
     * @return long value
     */
    def toLong: Long

    /** Attempt to convert this numerical to an Float value
     *
     * @return float value
     */
    def toFloat: Float

    /** Attempt to convert this numerical to an Double value
     *
     * @return double value
     */
    def toDouble: Double

    /** Attempt to convert this numerical to an Int value
     *
     * @return Optional value if conversion possible, None otherwise
     */
    def tryToInt: Option[Int]

    /** Attempt to convert this numerical to an Long value
     *
     * @return Optional value if conversion possible, None otherwise
     */
    def tryToLong: Option[Long]

    /** Attempt to convert this numerical to an Float value
     *
     * @return Optional value if conversion possible, None otherwise
     */
    def tryToFloat: Option[Float]

    /** Attempt to convert this numerical to an Double value
     *
     * @return Optional value if conversion possible, None otherwise
     */
    def tryToDouble: Option[Double]

    /** Attempt to convert this numerical to an Int or get a default Int of not possible
     *
     * @param default default value
     * @return result of conversion or provided default
     */
    def toIntOr(default: Int): Int = tryToInt match {
        case Some(value) => value
        case None => default
    }

    /** Attempt to convert this numerical to an Long or get a default Int of not possible
     *
     * @param default default value
     * @return result of conversion or provided default
     */
    def toLongOr(default: Long): Long = tryToInt match {
        case Some(value) => value
        case None => default
    }

    /** Attempt to convert this numerical to an Float or get a default Int of not possible
     *
     * @param default default value
     * @return result of conversion or provided default
     */
    def toFloatOr(default: Float): Float = tryToInt match {
        case Some(value) => value
        case None => default
    }

    /** Attempt to convert this numerical to an Double or get a default Int of not possible
     *
     * @param default default value
     * @return result of conversion or provided default
     */
    def toDoubleOr(default: Double): Double = tryToInt match {
        case Some(value) => value
        case None => default
    }
}

/**
 * Extension methods to allow using Int, Long, Float, and Double with Num terms
 */
extension (a: Num)
    @targetName("plus")
    def +(b: Int): Num = a + Num(b)
    def -(b: Int): Num = a - Num(b)
    def *(b: Int): Num = a * Num(b)
    def /(b: Int): Num = a / Num(b)
    def floorDiv(b: Int): Num = a - Num(b)
    def <(b: Int): Boolean = a < Num(b)
    def <=(b: Int): Boolean = a <= Num(b)
    def >(b: Int): Boolean = a > Num(b)
    def >=(b: Int): Boolean = a >= Num(b)
    @targetName("plus")
    def +(b: Long): Num = a + Num(b)
    def -(b: Long): Num = a - Num(b)
    def *(b: Long): Num = a * Num(b)
    def /(b: Long): Num = a / Num(b)
    def floorDiv(b: Long): Num = a - Num(b)
    def <(b: Long): Boolean = a < Num(b)
    def <=(b: Long): Boolean = a <= Num(b)
    def >(b: Long): Boolean = a > Num(b)
    def >=(b: Long): Boolean = a >= Num(b)
    def +(b: Float): Num = a + Num(b)
    def -(b: Float): Num = a - Num(b)
    def *(b: Float): Num = a * Num(b)
    def /(b: Float): Num = a / Num(b)
    def floorDiv(b: Float): Num = a - Num(b)
    def <(b: Float): Boolean = a < Num(b)
    def <=(b: Float): Boolean = a <= Num(b)
    def >(b: Float): Boolean = a > Num(b)
    def >=(b: Float): Boolean = a >= Num(b)
    @targetName("plus")
    def +(b: Double): Num = a + Num(b)
    def -(b: Double): Num = a - Num(b)
    def *(b: Double): Num = a * Num(b)
    def /(b: Double): Num = a / Num(b)
    def floorDiv(b: Double): Num = a - Num(b)
    def <(b: Double): Boolean = a < Num(b)
    def <=(b: Double): Boolean = a <= Num(b)
    def >(b: Double): Boolean = a > Num(b)
    def >=(b: Double): Boolean = a >= Num(b)

/**
 * Extension methods for Int to use with Num
 */
extension (a: Int)
    def +(b: Num): Num = Num(a) + b
    def -(b: Num): Num = Num(a) - b
    def *(b: Num): Num = Num(a) * b
    def /(b: Num): Num = Num(a) / b
    def floorDiv(b: Num): Num = Num(a) - b
    def <(b: Num): Boolean = Num(a) < b
    def <=(b: Num): Boolean = Num(a) <= b
    def >(b: Num): Boolean = Num(a) > b
    def >=(b: Num): Boolean = Num(a) >= b

/**
 * Extension methods for Long to use with Num
 */
extension (a: Long)
    def +(b: Num): Num = Num(a) + b
    def -(b: Num): Num = Num(a) - b
    def *(b: Num): Num = Num(a) * b
    def /(b: Num): Num = Num(a) / b
    def floorDiv(b: Num): Num = Num(a) - b
    def <(b: Num): Boolean = Num(a) < b
    def <=(b: Num): Boolean = Num(a) <= b
    def >(b: Num): Boolean = Num(a) > b
    def >=(b: Num): Boolean = Num(a) >= b

/**
 * Extension methods for Float to use with Num
 */
extension (a: Float)
    def +(b: Num): Num = Num(a) + b
    def -(b: Num): Num = Num(a) - b
    def *(b: Num): Num = Num(a) * b
    def /(b: Num): Num = Num(a) / b
    def floorDiv(b: Num): Num = Num(a) - b
    def <(b: Num): Boolean = Num(a) < b
    def <=(b: Num): Boolean = Num(a) <= b
    def >(b: Num): Boolean = Num(a) > b
    def >=(b: Num): Boolean = Num(a) >= b

/**
 * Extension methods for Double to use with Num
 */
extension (a: Double)
    def +(b: Num): Num = Num(a) + b
    def -(b: Num): Num = Num(a) - b
    def *(b: Num): Num = Num(a) * b
    def /(b: Num): Num = Num(a) / b
    def floorDiv(b: Num): Num = Num(a) - b
    def <(b: Num): Boolean = Num(a) < b
    def <=(b: Num): Boolean = Num(a) <= b
    def >(b: Num): Boolean = Num(a) > b
    def >=(b: Num): Boolean = Num(a) >= b

/**
 * A reference to a function
 * @param uri symbolic name of the function
 * @param lookup a look-up function that can resolve the uri to an AIDDL function object when needed
 */
final class FunRef(val uri: Sym, lookup : Sym=>Function) extends Term {
    lazy val f = lookup(uri)

    override def apply( x: Term ): Term = f(x)
    override def unify(t: Term): Option[Substitution] = if (t.isInstanceOf[FunRef] && t.asFunRef.uri == this.uri)  Some(new Substitution()) else None
    @targetName("substitute")
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

    /**
     * Create an anonymous variable with a unique internal ID
     * @return
     */
    def apply(): Var = {
        Var.last_id += 1
        Var("_" + last_id.toString())
    }
}

/**
 * Collect some creation methods and unapply to support pattern matching
 */
case object FunRef {
    /**
     * Create function reference
     * @param uri
     * @param lu
     * @return
     */
    def create( uri: Sym, lu: Sym=>Function ): FunRef = new FunRef(uri, lu)
    def apply( uri: Sym, f: Function ): FunRef = new FunRef(uri, _ => f)

    def unapply( fr: FunRef ): Option[(Sym, Function)] = Some((fr.uri, fr.f))
}













