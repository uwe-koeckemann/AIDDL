package org.aiddl.external.scala.coordination_oru

import org.aiddl.common.scala.Common
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.util.logger.Logger
import org.metacsp.multi.spatioTemporal.paths.Pose
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation
import se.oru.coordination.coordination_oru.CriticalSection
import se.oru.coordination.coordination_oru.RobotAtCriticalSection
import se.oru.coordination.coordination_oru.RobotReport

import java.util.Comparator
import com.vividsolutions.jts.geom.Coordinate
import org.aiddl.external.scala.coordination_oru.factory.CoordinatorFactory
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel
import se.oru.coordination.coordination_oru.Mission
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner
import se.oru.coordination.coordination_oru.util.BrowserVisualization
import se.oru.coordination.coordination_oru.util.Missions


@main def simpleExample = {
  val c = new Container()
  val parser = new Parser(c)
  val m = parser.parseFile("./test/map-example.aiddl")
  val coordinator = c.getProcessedValueOrPanic(m, Sym("coordinator"))
  val missions = c.getProcessedValueOrPanic(m, Sym("missions"))

  println(Logger.prettyPrint(coordinator, 0))

  val tec = CoordinatorFactory.fromAiddl(coordinator)

  var ids: List[Int] = Nil
  missions.asCol.foreach( m => {
    val id: Int = m(Sym("id")).intoInt
    val wps = m(Sym("waypoints")).asList.map(wp => {
      new Pose(wp(0).intoDouble, wp(1).intoDouble, wp(2).intoDouble)
    })
    val planner = tec.getMotionPlanner(id)

    ids = id :: ids

    planner.setStart(tec.getRobotReport(id).getPose)
    planner.setGoals(wps*)
    if (!planner.plan) throw new Error(s"No path for waypoints ${wps.mkString(", ")}")

    Missions.enqueueMission(new Mission(id, planner.getPath))
    Missions.enqueueMission(new Mission(id, planner.getPathInv))
  })

  // -----------------------------------
  Missions.startMissionDispatchers(tec, true, ids*)
}
