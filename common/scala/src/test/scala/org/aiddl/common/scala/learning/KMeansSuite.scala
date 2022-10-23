package org.aiddl.common.scala.learning

import org.aiddl.common.scala.learning.supervised.DataSplitter
import org.aiddl.common.scala.learning.supervised.decision_tree.ID3
import org.aiddl.common.scala.learning.unsupervised.KMeansClustering
import org.aiddl.common.scala.math.linear_algebra.{AiddlMatrix, Matrix}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.Logger
import org.scalatest.funsuite.AnyFunSuite

import scala.util.Random

class KMeansSuite extends AnyFunSuite {
  test("Running KMeans on three Gaussians") {
    val r = new Random(1)

    val c = List((-1.0, 1.0), (1.0, 1.0), (0.0, 0.0))
    val n = 1000

    val mean = 0.0
    val std = 0.1

    val data = ListTerm((1 to n).map( i => ListTerm(
      Num(c(i % 3)(0) + mean + std * r.nextGaussian()),
      Num(c(i % 3)(1) + mean + std * r.nextGaussian()) )))

    //println(Logger.prettyPrint(data, 0))

    val kMeans = KMeansClustering(3)

    val clusters = AiddlMatrix(kMeans(data))

    val ~= = Matrix.~=(Num(0.1))

    val clustersTrue = AiddlMatrix(
      Tuple(
        Tuple(Num(1.0), Num(1.0)),
        Tuple(Num(0.0), Num(0.0)),
        Tuple(Num(-1.0), Num(1.0))))

    assert( ~=(clusters, clustersTrue) )
  }
}
