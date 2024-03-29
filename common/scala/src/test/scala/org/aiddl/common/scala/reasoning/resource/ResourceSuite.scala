package org.aiddl.common.scala.reasoning.resource

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.reasoning.resource.{EstaScheduler, FlexibilityLossFunction, FlexibilityOrdering, PeakCollector}
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.StopWatch
import org.scalatest.funsuite.AnyFunSuite

class ResourceSuite extends AnyFunSuite {
  val p01 = {
    val c = new Container()
    val parser = new Parser(c)
    val m = parser.parseFile("aiddl-test/reasoning/resource/rcpsp-01.aiddl")
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("problem")).get.value)

  }
  val p02 = {
    val c = new Container()
    val parser = new Parser(c)
    val m = parser.parseFile("aiddl-test/reasoning/resource/rcpsp-02.aiddl")
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("problem")).get.value)
  }

  val esta = new EstaScheduler

  /*test("ESTA scheduler on unsolvable problem (01)") {
    import org.aiddl.common.scala.reasoning.resource.ResourceTerm._
    val sample = new PeakCollector
    val ordering = new FlexibilityOrdering
    val loss = new FlexibilityLossFunction
    val ac2stp = new AllenInterval2Stp
    val stp = new StpSolver

    val doms = stp(ac2stp(p01(Constraints)))
    val peaks = sample(p01(Capacity), p01(Usage), doms)
    peaks.foreach( p => println(s"$p -> ${loss(p, doms)}") )
    println(peaks)
    assert(esta(p01) == NIL)
    println(StopWatch.summary)
  }*/

  test("ESTA scheduler on unsolvable problem (01)") {
    assert(esta(p01) == NIL)
  }

  test("ESTA scheduler on solvable problem (01)") {
    assert(esta(p02) != NIL)
  }
}