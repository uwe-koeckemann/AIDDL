package org.aiddl.external.scala.coordination_oru.grpc

import org.aiddl.external.scala.coordination_oru.grpc.CoordinationActorServer
import com.vividsolutions.jts.geom.Coordinate
import org.aiddl.common.scala.Common
import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.grpc.scala.container.ContainerServer
import org.aiddl.external.scala.coordination_oru.factory.CoordinatorFactory
import org.aiddl.external.scala.coordination_oru.coordinator.CoordinatorWrapper
import org.metacsp.multi.spatioTemporal.paths.Pose
import se.oru.coordination.coordination_oru.*
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation
import se.oru.coordination.coordination_oru.util.{BrowserVisualization, Missions}

import scala.concurrent.ExecutionContext

object CoordinationServer {

  def run(cfg: Term): Unit = {
    /**
     * 1) Function to setup TEC and start GUI (via Service)
     * 2) Function to spawn robots (via Service)
     * 3) Function to send missions (via Actor Server)
     * 4) Available through
     */
    val c = new Container()
    val coordWrapper = CoordinatorWrapper(cfg)

    object AddRobot extends Function {
      override def apply(x: Term): Term = {
        coordWrapper.addRobot(x)
        NIL
      }
    }

    object CoordinatorConfig extends Function {
      override def apply(x: Term): Term = {
        coordWrapper.applyConfig(x)
        NIL
      }
    }

    object StartMissionDispatchers extends Function {
      override def apply(x: Term): Term = {
        println(s"Starting MDs for ${coordWrapper.robotIdMap.values.toList.mkString(", ")}")
        Missions.startMissionDispatchers(coordWrapper.tec, false, coordWrapper.robotIdMap.values.toList*)
        NIL
      }
    }



    c.addFunction(Sym("add-robot"), AddRobot)
    c.addFunction(Sym("config"), CoordinatorConfig)
    c.addFunction(Sym("start-dispatchers"), StartMissionDispatchers)

    val containerServer = new ContainerServer(ExecutionContext.global, 8063, c)
    containerServer.start()

    /*while ( !ready ) {
      Thread.sleep(50)
    }*/
    CoordinationActorServer.runAiddlGrpcServer(8061, c, coordWrapper)
  }
}
