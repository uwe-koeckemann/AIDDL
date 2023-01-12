package org.aiddl.external.scala.coordination_oru.util

import com.vividsolutions.jts.geom.Coordinate
import org.aiddl.core.scala.representation.*
import org.metacsp.multi.spatioTemporal.paths.{Pose, PoseSteering}

object Convert {
  def term2frame(t: Term): Array[Coordinate] = t.asList.map( c => new Coordinate(c(0).intoDouble, c(1).intoDouble)).toArray

  def term2pose(t: Term): Pose = new Pose(t(0).intoDouble, t(1).intoDouble, t(2).intoDouble)

  def poseSteering2term(p: PoseSteering): Term = {
    Tuple(
      Tuple(Num(p.getX), Num(p.getY), Num(p.getTheta)),
      Num(p.getSteering)
    )
  }
}
