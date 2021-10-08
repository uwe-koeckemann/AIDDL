package org.aiddl.common.scala.math.linear_algebra


import scala.collection.immutable.ArraySeq
import scala.collection.{SeqView, mutable}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.double2Num
import org.aiddl.core.scala.representation.TermImplicits.term2Num
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2Tuple
import org.aiddl.core.scala.representation.TermCollectionImplicits.seq2Tuple

 object Matrix {
    def ~=(epsilon: Num)(a: Matrix, b: Matrix): Boolean = {
        a.m == b.m && a.n == b.n
        && (0 until a.m).forall( i => (0 until a.n).forall( j => (a(i, j) - b(i, j)).abs < epsilon) )
    }
}

trait Matrix {
    outer =>

    def create( in: Seq[Seq[Term]] ): Matrix

    def apply( i: Int, j: Int ): Term
    def apply( i: Int ): Term

    def apply(other: Matrix):Matrix = this * other

    def m: Int
    def n: Int

    def t: Matrix
    def term: Term

    def iterCol( j: Int ): Iterator[Term] = new Iterator[Term] {
        var idx = -1
        def hasNext = idx < n
        def next = { idx += 1; outer(idx, j) }  
    }

    def iterRow( i: Int ): Iterator[Term] = new Iterator[Term] {
        var idx = -1
        def hasNext = idx < m
        def next = { idx += 1; outer(i, idx) }
    }

    def iterCols = new Iterator[Matrix] {
        var idx = -1
        def hasNext = idx < n
        def next = { idx += 1; col(idx) }
    }

    def iterRows = new Iterator[Matrix] {
        var idx = -1
        def hasNext = idx < m
        def next = { idx += 1; row(idx) }
    }

    def cols = (0 until n).map(r => col(r))
    def rows = (0 until m).map(r => row(r))

    def row( i: Int ): Matrix = create { Vector(for { j <- 0 until n } yield this(i, j)) }
    def col( j: Int ): Matrix = create { for { i <- 0 until m } yield Vector(this(i, j)) }

    def scalarOp( f: Term => Term ) : Matrix = create {
            for { i <- 0 until m } yield
                for { j <- 0 until n } yield 
                    f(this(i, j))}

    def +( s: Num ) : Matrix = scalarOp(_ + s)
    def -( s: Num ) : Matrix = scalarOp(_ - s)
    def *( s: Num ) : Matrix = scalarOp(_ * s)
    def /( s: Num ) : Matrix = scalarOp(_ / s)

    def +:( s: Num ): Matrix = this + s
    def -:( s: Num ): Matrix = this - s
    def *:( s: Num ): Matrix = this * s
    def /:( s: Num ): Matrix = this / s

    def matrixOp( other: Matrix, f: (Num, Num) => Num ) : Matrix = {
        require( this.n == other.n && this.m == other.m )
        create {
            for { i <- 0 until m } yield 
                for { j <- 0 until n } yield 
                    f(this(i, j), other(i, j))}}
 
    def +( other: Matrix ) : Matrix = this.matrixOp(other, _ + _)
    def -( other: Matrix ) : Matrix = this.matrixOp(other, _ - _)

    def *( b: Matrix ): Matrix = {
        require( this.n == b.m )
        create((0 until m).map( i => 
               (0 until b.n).map( j => 
                (0 until n).foldLeft(Num(0))
                    ( (c, k) => c + (this(i, k) * b(k, j)) ))))
    }
}

