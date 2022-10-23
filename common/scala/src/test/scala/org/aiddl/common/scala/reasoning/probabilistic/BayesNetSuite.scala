package org.aiddl.common.scala.reasoning.probabilistic

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.reasoning.probabilistic.McmcSampler
import org.aiddl.common.scala.reasoning.probabilistic.ProbabilisticTerm.*
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.conversion.{given_Conversion_Term_KeyVal, given_Conversion_Term_Num}
import org.aiddl.core.scala.util.StopWatch
import org.scalatest.funsuite.AnyFunSuite

import scala.language.implicitConversions

class BayesNetSuite extends AnyFunSuite {
  val q01 = {
    val c = new Container()
    val parser = new Parser(c)
    val m = parser.parseFile("../test/reasoning/probabilistic/bayesian-network.aiddl")
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("query-01")).get.value)
  }
  val q02 = {
    val c = new Container()
    val parser = new Parser(c)
    val m = parser.parseFile("../test/reasoning/probabilistic/bayesian-network.aiddl")
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("query-02")).get.value)
  }
  val q03 = {
    val c = new Container()
    val parser = new Parser(c)
    val m = parser.parseFile("../test/reasoning/probabilistic/bayesian-network.aiddl")
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("query-03")).get.value)
  }


  test("Markov Chain Monte Carlo algorithm for Bayes Network samples correctly (01)") {
    val sample = new McmcSampler
    sample.nSamples = 1000
    sample.init(q01(BayesNet))
    val r = sample(q01(Variable), q01(Evidence).asCol)
    assert(r(0).asKvp.value.asNum > Num(0.45))
    assert(r(0).asKvp.value.asNum < Num(0.7))
  }

  test("Markov Chain Monte Carlo algorithm for Bayes Network samples correctly (02)") {
    val sample = new McmcSampler
    sample.nSamples = 1000
    sample.init(q02(BayesNet))
    val r = sample(q02(Variable), q02(Evidence).asCol)
    assert(r(0).asKvp.value.asNum > Num(0.4))
    assert(r(0).asKvp.value.asNum < Num(0.6))
  }
  test("Markov Chain Monte Carlo algorithm for Bayes Network samples correctly (03)") {
    val sample = new McmcSampler
    sample.nSamples = 1000
    sample.init(q03(BayesNet))
    val r = sample(q03(Variable), q03(Evidence).asCol)
    assert(r(0).asKvp.value.asNum < Num(0.1))
  }
}