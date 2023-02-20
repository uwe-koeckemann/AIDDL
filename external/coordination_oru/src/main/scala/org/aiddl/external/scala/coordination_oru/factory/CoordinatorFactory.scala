package org.aiddl.external.scala.coordination_oru.factory

import org.aiddl.core.scala.representation.*
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.util.FilenameResolver
import org.aiddl.core.scala.util.logger.Logger
import org.metacsp.multi.spatioTemporal.paths.Pose
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation
import se.oru.coordination.coordination_oru.CriticalSection
import se.oru.coordination.coordination_oru.RobotAtCriticalSection
import se.oru.coordination.coordination_oru.RobotReport

import java.util.Comparator
import com.vividsolutions.jts.geom.Coordinate
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel
import se.oru.coordination.coordination_oru.Mission
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner
import se.oru.coordination.coordination_oru.util.BrowserVisualization
import se.oru.coordination.coordination_oru.util.Missions
import org.aiddl.external.scala.coordination_oru.util.Convert.{term2frame, term2pose}
import org.aiddl.external.scala.coordination_oru.MotionPlanningTerm.MapKey

object CoordinatorFactory {


  def fromAiddl( cfg: Term ): TrajectoryEnvelopeCoordinatorSimulation = {
    val MAX_ACCEL = cfg(Sym("max-accel")).intoDouble
    val MAX_VEL = cfg(Sym("max-vel")).intoDouble
    val map = cfg.get(MapKey).flatMap(f => Some(FilenameResolver(f).asStr.value))
    val tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL, MAX_ACCEL)

    tec.addComparator(new Comparator[RobotAtCriticalSection]() {
      override def compare(o1: RobotAtCriticalSection, o2: RobotAtCriticalSection): Int = {
        val cs = o1.getCriticalSection
        val robotReport1 = o1.getRobotReport
        val robotReport2 = o2.getRobotReport
        (cs.getTe1Start - robotReport1.getPathIndex) - (cs.getTe2Start - robotReport2.getPathIndex)
      }
    })
    tec.addComparator(new Comparator[RobotAtCriticalSection]() {
      override def compare(o1: RobotAtCriticalSection, o2: RobotAtCriticalSection): Int = o2.getRobotReport.getRobotID - o1.getRobotReport.getRobotID
    })

    //Need to setup infrastructure that maintains the representation
    tec.setupSolver(0, 100000000)
    //Start the thread that checks and enforces dependencies at every clock tick
    tec.startInference

    tec.setBreakDeadlocks(false, false, true)

    cfg(Sym("robots")).asCol.foreach( robotCfg => { // TODO: Allow to add later
      val id: Int = robotCfg(Sym("id")).intoInt // TODO: Term ID to create lookup
      val frame = term2frame(robotCfg(Sym("footprint")))

      tec.setFootprint(id, frame*)
      //TODO: Hard-coded for now
      tec.setForwardModel(id,
        new ConstantAccelerationForwardModel(
          MAX_ACCEL,
          MAX_VEL,
          tec.getTemporalResolution,
          tec.getControlPeriod,
          tec.getRobotTrackingPeriodInMillis(id)))
      val planner = MotionPlannerFactory.fromAiddl(robotCfg(Sym("motion-planner")))
      planner.setFootprint(frame*)
      map match {
        case Some(yamlFile) => {
          println(yamlFile)
          planner.setMap(yamlFile)
        }
        case None => {}
      }
      tec.setMotionPlanner(id, planner)
      val pose = term2pose(robotCfg(Sym("start-pose")))
      tec.placeRobot(id, pose)
    })

    val viz = new BrowserVisualization
    //viz.setInitialTransform(49, 5, 0)
    viz.setInitialTransform(20.0, 9.0, 2.0)
    viz.setFontScale(1.6)
    map match {
      case Some(yamlFile) => viz.setMap(yamlFile)
      case None => {}
    }
    tec.setVisualization(viz)
    tec.setUseInternalCriticalPoints(true)
    tec
  }
}
