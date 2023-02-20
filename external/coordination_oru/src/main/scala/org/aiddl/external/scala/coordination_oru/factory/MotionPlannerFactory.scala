package org.aiddl.external.scala.coordination_oru.factory

import org.aiddl.core.scala.representation.*
import org.aiddl.external.scala.coordination_oru.util.Convert.term2frame
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner

object MotionPlannerFactory {
  def fromAiddl( cfg: Term ): AbstractMotionPlanner = {
    cfg(Sym("model")) match {
      case Sym("ReedsSheppCarPlanner") => {
        val radius: Double = cfg(Sym("radius")).intoDouble
        val turningRadius: Double = cfg(Sym("turning-radius")).intoDouble
        val distBetweenPoints: Double = cfg(Sym("distance-between-path-points")).intoDouble
        val alg: Sym = cfg(Sym("algorithm")).asSym
        val footprint = term2frame(cfg(Sym("footprint")))

        //motionPlanner = new ReedsSheppCarPlanner(PLANNING_ALGORITHM.RRTstar)
        val rsp = new ReedsSheppCarPlanner(ReedsSheppCarPlanner.PLANNING_ALGORITHM.valueOf(alg.toString))

        //val rsp = new ReedsSheppCarPlanner
        rsp.setRadius(radius)
        rsp.setTurningRadius(turningRadius)
        rsp.setFootprint(footprint*)
        rsp.setDistanceBetweenPathPoints(distBetweenPoints)
        rsp
      }
      case t => throw new IllegalArgumentException(s"Unknown motion planner type: $t")
    }
  }

  def fromPlannerAndRobotCfg(plannerCfg: Term, robotCfg: Term): AbstractMotionPlanner = {
    robotCfg(Sym("model")) match {
      case Sym("ReedsSheppCar") => {
        val turningRadius: Double = robotCfg(Sym("turning-radius")).intoDouble

        val radius: Double = plannerCfg(Sym("radius")).intoDouble
        val distBetweenPoints: Double = plannerCfg(Sym("distance-between-path-points")).intoDouble
        val alg: Sym = plannerCfg(Sym("algorithm")).asSym
        val footprint = term2frame(robotCfg(Sym("footprint")))

        val rsp = new ReedsSheppCarPlanner(ReedsSheppCarPlanner.PLANNING_ALGORITHM.valueOf(alg.toString))

        rsp.setRadius(radius)
        rsp.setTurningRadius(turningRadius)
        rsp.setFootprint(footprint*)
        rsp.setDistanceBetweenPathPoints(distBetweenPoints)
        rsp
      }
      case t => throw new IllegalArgumentException(s"Unknown motion planner type: $t")
    }
  }
}
