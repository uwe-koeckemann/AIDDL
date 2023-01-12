package org.aiddl.external.scala.coordination_oru.actor

import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.Actor.ActionInstanceId
import org.aiddl.core.scala.representation.*
import org.metacsp.multi.spatioTemporal.paths.{Pose, PoseSteering}
import se.oru.coordination.coordination_oru.Mission
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation
import se.oru.coordination.coordination_oru.util.{MissionDispatchingCallback, Missions}

import scala.collection.mutable

class CoordinationActor(pattern: Term, vars: Term, ids: Map[Term, Int], tec: TrajectoryEnvelopeCoordinatorSimulation) extends Actor {
  val nameLookup: Map[Int, Term] = ids.map( (t, i) => i -> t).toMap
  var active: mutable.Map[Term, ActionInstanceId] = new mutable.HashMap()
  private val selfRef = this

  val cb = new MissionDispatchingCallback() {
    override def beforeMissionDispatch(m: Mission) = { }
    override def afterMissionDispatch(m: Mission) = {
      selfRef.update(active(nameLookup(m.getRobotID)), Actor.Status.Active)
    }
  }
  ids.values.foreach( i => Missions.addMissionDispatchingCallback(i, cb) )

  override def supported(action: Term): Boolean = (pattern unify action) match {
    case Some(s) => (vars\s).isGround && ids.contains((vars\s)(0))
    case None => false
  }

  override def dispatch(action: Term): Option[ActionInstanceId] = {
    (pattern unify action) match {
      case Some(s) => {
        val args = vars\s
        if !args.isGround || !ids.contains(args(0)) || active.isDefinedAt(args(0)) then None
        else {
          val instanceId = super.nextId
          println(s"STARTING: $args with $instanceId")

          val path = args(1).asList.map(ps => new PoseSteering(ps(0)(0).intoDouble, ps(0)(1).intoDouble, ps(0)(2).intoDouble, ps(1).intoDouble)).toArray
          Missions.enqueueMission(new Mission(ids(args(0)), path))
          super.update(instanceId, Actor.Status.Pending)
          active.put(args(0), instanceId)
          Some(instanceId)
        }
      }
      case _ => None
    }
  }

  /**
   * TODO: check for status updates of each active mission
   * tec.getIdleRobots()
   * float progress = tec.getRobotReport(robotID).getPathIndex()/tec.getCurrentTrajectoryEnvelope(robotID)
   * tec.isFree(robotID)
   * Missions.addMissionDispatchingCallback(robotID, cb)
   *
   * MissionDispatchingCallback cb = new MissionDispatchingCallback() {

  @Override
  public void beforeMissionDispatch(Mission m) {
// TODO Auto-generated method stub

}

  @Override
  public void afterMissionDispatch(Mission m) {
// TODO Auto-generated method stub

}
};
   */
  override def tick: Unit = {
    // TODO: Track and update active instances (maybe keep active list in super?)
    tec.getIdleRobots.forEach(tecId => {
      val r = nameLookup(tecId)
      if active.isDefinedAt(r) && status(active(r)) == Actor.Status.Active then {
        val instanceId = active(r)
        println(s"Finished: $r $instanceId")
        super.update(instanceId, Actor.Status.Succeeded)
        active.remove(r)
      }
    })
  }
}
