package org.aiddl.external.scala.coordination_oru.coordinator

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
import org.aiddl.external.scala.coordination_oru.actor.CoordinationActor
import org.aiddl.external.scala.coordination_oru.factory.MotionPlannerFactory


class CoordinatorWrapper(var cfg: Term) {
  var robotIdMap: Map[Term, Int] = Map.empty
  var nextFreeRobotId = 0

  var max_velocity = 4.0
  var max_accel = 1.0

  val tec: TrajectoryEnvelopeCoordinatorSimulation =
    new TrajectoryEnvelopeCoordinatorSimulation(max_velocity, max_accel)

  tec.addComparator(new Comparator[RobotAtCriticalSection]() {
    override def compare(o1: RobotAtCriticalSection, o2: RobotAtCriticalSection): Int = {
      val cs = o1.getCriticalSection
      val robotReport1 = o1.getRobotReport
      val robotReport2 = o2.getRobotReport
      (cs.getTe1Start - robotReport1.getPathIndex) - (cs.getTe2Start - robotReport2.getPathIndex)
    }
  })
  tec.addComparator(new Comparator[RobotAtCriticalSection]() {
    override def compare(o1: RobotAtCriticalSection, o2: RobotAtCriticalSection): Int =
      o2.getRobotReport.getRobotID - o1.getRobotReport.getRobotID
  })

  val actor = new CoordinationActor(cfg(Sym("pattern")), cfg(Sym("variables")), this.robotIdMap, this.tec)

  //viz.setInitialTransform(49, 5, 0)

  //this.applyConfig(cfg)

  def applyConfig(cfg: Term): Unit = {
    this.cfg = cfg
    max_accel = cfg(Sym("max-accel")).intoDouble
    max_velocity = cfg(Sym("max-vel")).intoDouble
    val map = cfg.get(MapKey).flatMap(f => Some(FilenameResolver(f).asStr.value))
    //Need to setup infrastructure that maintains the representation
    tec.setupSolver(0, 100000000)
    //Start the thread that checks and enforces dependencies at every clock tick
    tec.startInference

    tec.setBreakDeadlocks(false, false, true)
    val viz = new BrowserVisualization
    map match {
      case Some(yamlFile) => viz.setMap(yamlFile)
      case None => {}
    }
    viz.setInitialTransform(20.0, 9.0, 2.0)
    viz.setFontScale(1.6)

    tec.setVisualization(viz)
    tec.setUseInternalCriticalPoints(false)

  }

  def addRobot(robotCfg: Term): Unit = {
    println(s"Adding robot:\n  $robotCfg")
    val id: Int = nextFreeRobotId
    println(s"  with ID $id")
    nextFreeRobotId += 1
    this.robotIdMap = this.robotIdMap.updated(robotCfg(Sym("name")), id)
    val map = this.cfg.get(MapKey).flatMap(f => Some(FilenameResolver(f).asStr.value))

    val frame = term2frame(robotCfg(Sym("footprint")))
    tec.setFootprint(id, frame *)
    //TODO: Hard-coded for now
    tec.setForwardModel(id,
      new ConstantAccelerationForwardModel(
        max_accel,
        max_velocity,
        tec.getTemporalResolution,
        tec.getControlPeriod,
        tec.getRobotTrackingPeriodInMillis(id)))
    val planner = MotionPlannerFactory.fromPlannerAndRobotCfg(
      cfg(Sym("planner-config")),
      robotCfg
    )
    map match {
      case Some(yamlFile) => {
        println(s"Setting robot map: $yamlFile")
        planner.setMap(yamlFile)
      }
      case None => {println("No map found")}
    }
    tec.setMotionPlanner(id, planner)
    val pose = term2pose(robotCfg(Sym("start-pose")))
    tec.placeRobot(id, pose)
    actor.addRobot(robotCfg(Sym("name")), id)

    //tec.getVisualization.asInstanceOf[BrowserVisualization].sendUpdate()
  }
}
