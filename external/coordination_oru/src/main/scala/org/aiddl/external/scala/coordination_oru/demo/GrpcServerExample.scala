package org.aiddl.external.scala.coordination_oru.demo

import com.vividsolutions.jts.geom.Coordinate
import org.aiddl.common.scala.Common
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.scala.coordination_oru.factory.CoordinatorFactory
import org.metacsp.multi.spatioTemporal.paths.Pose
import se.oru.coordination.coordination_oru.*
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation
import se.oru.coordination.coordination_oru.util.{BrowserVisualization, Missions}
import org.aiddl.external.scala.coordination_oru.grpc.CoordinationServer
import java.util.Comparator

@main def grpcServerExample = {
  val c = new Container()
  val parser = new Parser(c)

  val coordinatorConfigDefault = parser.str("[" +
    "max-accel:1.0 " +
    "max-vel:4.0 " +
    "pattern:(follow ?ID ?PATH) " +
    "variables:[?ID ?PATH] ]")


  CoordinationServer.run(coordinatorConfigDefault)
}
