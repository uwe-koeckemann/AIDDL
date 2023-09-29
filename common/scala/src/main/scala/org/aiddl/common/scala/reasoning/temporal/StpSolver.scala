package org.aiddl.common.scala.reasoning.temporal

import scala.annotation.tailrec
import scala.collection.mutable.Map
import scala.collection.mutable.HashMap

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Num

import scala.language.implicitConversions


class StpSolver extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.reasoning.temporal.stp.solver")
	
    def apply( stp: Term ): Term = {
        val cs = stp(1).asCol
        val tOrigin = Num(0)
        val tHorizon = InfPos()

        var next_id = 0
        val id_map = new HashMap[Term, Int]()
        val name_map = new HashMap[Int, Term]()

        val dMapIn = new HashMap[Int, List[(Int, Int, Num, Num)]]().withDefaultValue(Nil)
        val dMapOut = new HashMap[Int, List[(Int, Int, Num, Num)]]().withDefaultValue(Nil)

        cs.foreach( c => {
            val c_0 = id_map.getOrElseUpdate(c(0), {
                val id = next_id
                next_id += 1
                name_map.put(id, c(0))
                id
            })
            val c_1 = id_map.getOrElseUpdate(c(1), {
                val id = next_id
                next_id += 1
                name_map.put(id, c(1))
                id
            })

            dMapOut.put(c_0, (c_0, c_1, c(2).asNum, c(3).asNum) :: dMapOut(c_0))
            dMapIn.put(c_1, (c_0, c_1, c(2).asNum, c(3).asNum) :: dMapOut(c_1))
        })

        val xs = stp(0).asCol.withFilter(id_map.contains(_)).map(id_map(_))

        val lower = new HashMap[Int, Num]().withDefaultValue(tOrigin)
        val upper = new HashMap[Int, Num]().withDefaultValue(tHorizon)
        val update = new HashMap[Int, Boolean]().withDefaultValue(true)
        val pl = new HashMap[Int, Option[Int]].withDefaultValue(None)
        val pu = new HashMap[Int, Option[Int]].withDefaultValue(None)

        var consistent = true
        var updated = true
        while ( consistent && updated ) {
            updated = false
            consistent = xs.forall( x => {
                if ( update(x) ) {
                    // println(s"$x (${lower(x)} ${upper(x)})\n\t$pl\n\t$pl")
                    dMapIn(x).forall( c => {
                        propagate(c(0), c(1), c(2), c(3), lower, upper, pl, pu, update) match
                            case (con, change) => updated = updated || change; con
                    }) &&
                    dMapOut(x).forall( c => {
                        propagate(c(0), c(1), c(2), c(3), lower, upper, pl, pu, update) match
                            case (con, change) => updated = updated || change; con
                    })       
                } else true
            })
        }

        if ( !consistent ) NIL
        else ListTerm(xs.map(x => KeyVal(name_map(x), Tuple(lower(x), upper(x)))).toSeq)
    }

    private def propagate( 
        x: Int, y: Int, a: Num, b: Num,
        lower: Map[Int, Num], upper: Map[Int, Num],
        pl: Map[Int, Option[Int]], pu: Map[Int, Option[Int]],
        update: Map[Int, Boolean]
    ): (Boolean, Boolean) = {
        val cxl = {
            val pt = lower(y) - b
            if ( pt > lower(x) ) { pl.put(x, Some(y)); lower.put(x, pt); true } 
            else false }
        val cxu = {
            val pt = upper(y) - a
            if ( pt < upper(x) ) { pu.put(x, Some(y)); upper.put(x, pt); true } 
            else false
        }
        val cyl = {
            val pt = lower(x) + a
            if ( pt > lower(y) ) { pl.put(y, Some(x)); lower.put(y, pt); true } 
            else false }
        val cyu = {
            val pt = upper(x) + b
            if ( pt < upper(y) ) { pu.put(y, Some(x)); upper.put(y, pt); true } 
            else false
        }

        update.put(x, cxl || cxu || update(x))
        update.put(y, cyl || cyu || update(y))

        (
            (lower(x) <= upper(x) && lower(y) <= upper(y))
                && !(cxl && hasCycle(x, y, pl)) 
                && !(cxu && hasCycle(x, y, pu)) 
                && !(cyl && hasCycle(y, x, pl))
                && !(cyu && hasCycle(y, x, pu)),
            cxl || cxu || cyl || cyu
        )
    }

    @tailrec
    private def hasCycle( s: Int, c: Int, p: Map[Int, Option[Int]] ): Boolean =
        p(c) match {
            case Some(x) if x == s => true
            case None => false
            case Some(x) => hasCycle(s, x, p)
        }
}