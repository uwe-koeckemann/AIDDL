package org.aiddl.external.scala.coordination_oru.demo

import com.vividsolutions.jts.geom.Coordinate
import org.aiddl.common.scala.Common
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.FilenameResolver
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.grpc.scala.actor.ActorClient
import org.aiddl.external.scala.coordination_oru.factory.{CoordinatorFactory, MotionPlannerFactory}
import org.aiddl.external.grpc.scala.container.GrpcFunction
import org.aiddl.external.scala.coordination_oru.MotionPlanningTerm.MapKey
import org.metacsp.multi.spatioTemporal.paths.Pose
import se.oru.coordination.coordination_oru.*
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation
import se.oru.coordination.coordination_oru.util.{BrowserVisualization, Missions}

import java.util.Comparator

@main def grpcClientExample = {
  val c = new Container()
  val parser = new Parser(c)
  val m = parser.parseFile("./test/test-grpc-client.aiddl")

  val missions = c.getProcessedValueOrPanic(m, Sym("missions"))
  val poseMap = c.getProcessedValueOrPanic(m, Sym("pose-map"))
  val coordinatorCfg = c.getProcessedValueOrPanic(m, Sym("coordinator-config"))
  val plannerCfg = c.getProcessedValueOrPanic(m, Sym("planner-config"))
  val robotCfgs = c.getProcessedValueOrPanic(m, Sym("robot-configs")).asCol

  val map = coordinatorCfg.get(MapKey).flatMap(f => Some(FilenameResolver(f).asStr.value))



  val planners = robotCfgs.map( cfg => {
    cfg(Sym("name")) -> {
      val planner = MotionPlannerFactory.fromPlannerAndRobotCfg(plannerCfg, cfg)
      map match {
        case Some(yamlFile) => {
          println(s"Setting robot map: $yamlFile")
          planner.setMap(yamlFile)
        }
        case None => {
          println("No map found")
        }
      }
      planner
    }
  }).toMap

  val coordCfgFun = GrpcFunction("localhost", 8063, Sym("config"), c)
  val addRobotFun = GrpcFunction("localhost", 8063, Sym("add-robot"), c)
  val startMissionDispatch = GrpcFunction("localhost", 8063, Sym("start-dispatchers"), c)

  coordCfgFun(coordinatorCfg)
  robotCfgs.foreach(addRobotFun(_))
  startMissionDispatch(Sym("NIL"))

  val coordinationActor = new ActorClient("localhost", 8061, c)

  var robotPoseCurrent: Map[Term, Pose] = Map.empty
  robotCfgs.foreach(c => {
    val name = c(Sym("name"))
    val poseStart = c(Sym("start-pose"))
    val pose = new Pose(poseStart(0).intoDouble, poseStart(1).intoDouble, poseStart(2).intoDouble)
    robotPoseCurrent = robotPoseCurrent.updated(name, pose)
  })

  missions.asCol.foreach(m => {
    val id = m(Sym("name"))
    val start = robotPoseCurrent(id)
    val wps = m(Sym("waypoints")).asList.map(wp => {
      new Pose(wp(0).intoDouble, wp(1).intoDouble, wp(2).intoDouble)
    })
    val planner = planners(id)


    planner.setStart(start)
    planner.setGoals(wps *)
    if (!planner.plan) throw new Error(s"No path for waypoints ${wps.mkString(", ")}")
    val path = planner.getPath
    val pathAiddl = ListTerm(path.map( p => Tuple(Tuple(Num(p.getX), Num(p.getY), Num(p.getYaw)), Num(p.getSteering))))

    pathAiddl.foreach(println)

    robotPoseCurrent = robotPoseCurrent.updated(id, path.last.getPose)

    println(coordinationActor.dispatch(Tuple(Sym("follow"), id, pathAiddl)))
  })


}
