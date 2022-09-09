package org.aiddl.common.scala.math.linear_algebra

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

import scala.collection.mutable.HashMap

import scala.collection.mutable

class LupDecomposition extends Function {
  def apply( x: Term ): Term = ???

  def apply( a: Matrix ): (Matrix, Matrix, Matrix) = {
    require( a.m == a.n )
    val pi = Array.range(0, a.m).map(i => Num(i))
    val a_mut: mutable.Map[(Int, Int), Num] = new HashMap[(Int, Int), Num]().withDefault((i: Int , j: Int) => a(i, j).asNum)
    (0 until a.m).foreach( k => {
      val k_d = (k until a.m).maxBy( i => a_mut(i, k).abs )
      if ( a_mut(k_d, k) == Num(0) ) throw new IllegalArgumentException("Matrix is singular.")

      // fix permunation
      val tmp = pi(k)
      pi.update(k, pi(k_d))
      pi.update(k_d, tmp)
      // swap rows
      (0 until a.m).foreach( i => {
        val tmp = a_mut(k, i)
        a_mut.update((k, i), a_mut(k_d, i))
        a_mut.update((k_d, i), tmp)
      })
      (k+1 until a.m).foreach( i => {
        a_mut.update((i, k), a_mut(i, k) / a_mut(k, k)  )
        (k+1 until a.n).foreach( j => {
          a_mut.update((i, j), a_mut(i, j) - a_mut(i, k) * a_mut(k, j))
        })
      })
    })

    val L = Tuple((0 until a.m).map( i => {
      Tuple((0 until a.m).map( j =>
        if ( i > j ) a_mut(i, j) else if (i == j) Num(1.0) else Num(0.0) ): _*)}): _*)
    val U = Tuple((0 until a.m).map( i => {
      Tuple((0 until a.m).map( j =>
        if ( i > j ) Num(0.0) else a_mut(i, j) ): _*)}): _*)

    (AiddlMatrix(L), AiddlMatrix(U), AiddlMatrix.vec(pi: _*))
  }
}
