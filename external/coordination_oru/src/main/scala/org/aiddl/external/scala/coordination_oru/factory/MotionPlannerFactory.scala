package org.aiddl.external.scala.coordination_oru.factory

import org.aiddl.core.scala.representation.*
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
        //motionPlanner = new ReedsSheppCarPlanner(PLANNING_ALGORITHM.RRTstar)
        val rsp = new ReedsSheppCarPlanner(ReedsSheppCarPlanner.PLANNING_ALGORITHM.valueOf(alg.toString))

        //val rsp = new ReedsSheppCarPlanner
        rsp.setRadius(radius)
        rsp.setTurningRadius(turningRadius)
        rsp.setDistanceBetweenPathPoints(distBetweenPoints)
        rsp
      }
      case t => throw new IllegalArgumentException(s"Unknown motion planner type: $t")
    }
  }
}
