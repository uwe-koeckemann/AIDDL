package org.aiddl.common.scala.learning.unsupervised

import scala.collection.mutable.{ArraySeq, HashSet}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.container.Container
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.linear_algebra.{AiddlMatrix, Matrix, NormL}


object KMeansClustering {
  def apply( k: Int ): KMeansClustering = {
    val kMeans = new KMeansClustering
    kMeans.init(Num(k))
    kMeans
  }
}

class KMeansClustering extends Function {
  var k = 3;
  val l2 = new NormL(2)

  def init( k: Term ) = this.k = k.intoInt

  def apply( data: Matrix ): Term = {
    val m = new Array[Matrix](k)
    val mNew = new Array[Matrix](k)
    (0 until k).foreach( i => mNew.update(i, data.row(i)) )
    while({
      (0 until k).foreach( i => m.update(i, mNew(i)) )
      val s = (0 until data.m).map(r => data.row(r)).groupBy( x => (0 until k).minBy( i => l2.apply(m(i) - x) ) )
      (0 until k).foreach( i => {
        mNew.update(i, s(i).tail.foldLeft(s(i).head)( (c, t) =>
          c + t) * Num(1.0/s(i).size) ) } )
      (0 until k).exists( i => m(i) != mNew(i))
    })()
    Tuple( m.map( r => r.term(0) ): _* )
  }

  def apply( data: Term ): Term = {
    val m = new Array[Matrix](k)
    val mNew = new Array[Matrix](k)
    (0 until k).foreach( i => mNew.update(i, AiddlMatrix.row_vec(data(i).asList)) )
    while({
      (0 until k).foreach( i => m.update(i, mNew(i)) )
      val s = data.asList.groupBy( x => (0 until k).minBy( i => l2.apply(m(i) - AiddlMatrix.row_vec(x.asList)) ) )
      (0 until k).foreach( i => {
        mNew.update(i, s(i).tail.foldLeft(AiddlMatrix.row_vec(s(i).head.asList))( (c, t) =>
          c + AiddlMatrix.row_vec(t.asList)) * Num(1.0/s(i).size) ) } )
      (0 until k).exists( i => m(i) != mNew(i))
    })()
    Tuple( m.map( r => r.term(0) ): _* )
  }
}
