import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.parser.Parser
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.reasoning.resource.{EstaScheduler, FlexibilityLossFunction, FlexibilityOrdering, PeakCollector}
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.core.scala.tools.StopWatch
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

class ResourceSuite extends AnyFunSuite {
  val p01 = {
    val c = new Container()
    val m = Parser.parseInto("../test/reasoning/resource/rcpsp-01.aiddl", c)
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("problem")).get.v)

  }
  val p02 = {
    val c = new Container()
    val m = Parser.parseInto("../test/reasoning/resource/rcpsp-02.aiddl", c)
    assert(c.typeCheckModule(m))
    c.resolve(c.getEntry(m, Sym("problem")).get.v)
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