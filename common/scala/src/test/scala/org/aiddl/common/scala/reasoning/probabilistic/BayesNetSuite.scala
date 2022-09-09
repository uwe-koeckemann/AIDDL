package org.aiddl.common.scala.reasoning.probabilistic

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.parser.Parser
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.reasoning.probabilistic.McmcSampler
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.core.scala.tools.StopWatch
import org.aiddl.common.scala.reasoning.probabilistic.ProbabilisticTerm.*
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

import org.aiddl.core.scala.representation.given_Conversion_Term_KeyVal
import org.aiddl.core.scala.representation.given_Conversion_Term_Num

class BayesNetSuite extends AnyFunSuite {
  val q01 = {
    val c = new Container()
    val m = Parser.parseInto("../test/reasoning/probabilistic/bayesian-network.aiddl", c)
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("query-01")).get.v)
  }
  val q02 = {
    val c = new Container()
    val m = Parser.parseInto("../test/reasoning/probabilistic/bayesian-network.aiddl", c)
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("query-02")).get.v)
  }
  val q03 = {
    val c = new Container()
    val m = Parser.parseInto("../test/reasoning/probabilistic/bayesian-network.aiddl", c)
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("query-03")).get.v)
  }


  test("Markov Chain Monte Carlo algorithm for Bayes Network samples correctly (01)") {
    val sample = new McmcSampler
    sample.nSamples = 1000
    sample.init(q01(BayesNet))
    val r = sample(q01(Variable), q01(Evidence))
    assert(r(0).value > 0.45)
    assert(r(0).value < 0.7)
  }

  test("Markov Chain Monte Carlo algorithm for Bayes Network samples correctly (02)") {
    val sample = new McmcSampler
    sample.nSamples = 1000
    sample.init(q02(BayesNet))
    val r = sample(q02(Variable), q02(Evidence))
    assert(r(0).value > 0.4)
    assert(r(0).value < 0.6)
  }
  test("Markov Chain Monte Carlo algorithm for Bayes Network samples correctly (03)") {
    val sample = new McmcSampler
    sample.nSamples = 1000
    sample.init(q03(BayesNet))
    val r = sample(q03(Variable), q03(Evidence))
    assert(r(0).value < 0.1)
  }
}