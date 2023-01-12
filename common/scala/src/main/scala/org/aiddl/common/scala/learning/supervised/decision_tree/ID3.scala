package org.aiddl.common.scala.learning.supervised.decision_tree

import org.aiddl.core.scala.container.Container

import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._

import org.aiddl.core.scala.util.logger.Logger

import org.aiddl.common.scala.Common.NIL

import org.aiddl.common.scala.learning.LearningTerm._
import org.aiddl.common.scala.learning.supervised.Learner

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_ListTerm
import scala.language.implicitConversions


class ID3 extends Learner with Verbose {

    var includeAllLeafs = false
    var decisionTree: Term = NIL

    def fit(x: ListTerm, y: ListTerm): Term = {
        val examples = ListTerm(
            y.zip(x).map( { (y, x) => ListTerm(y +: x.asList.list) } )
        )

        val labelIdx = 0
        val numAtts = x.head.length
        val attributes = (1 until numAtts).toArray
        val init = Array.fill[Set[Term]](numAtts)(Set.empty)
        val values = examples.foldLeft(init)( (c, e) => c.zip(e.asList).map( x => x match { case (ci, ei) => ci + ei } )  )

        this.includeAllLeafs = this.parameters.getOrElse(Sym("includeAllLeafs"), Bool(false)).asBool.v

        this.decisionTree = runID3(examples, labelIdx, attributes, values)
        decisionTree
    }

    def predict(x: ListTerm): ListTerm = ListTerm(x.map( x_i => this.resolve(this.decisionTree, x_i)))

    val LOG2 = Math.log(2);

    def entropy( examples: Seq[Term], labelIdx: Int ):Double = {
        val counts = examples.groupBy( x => x(labelIdx) ).map( p => p match { case (k, s) => s.length })
        val n = examples.length.toDouble
        counts.foldLeft(0.0)( (c, p) => { val pi = (p/n); c + (-pi * (Math.log(pi)/LOG2)) })
    }

    def information_gain(examples: ListTerm, attIdx: Int, labelIdx: Int, values: Set[Term] ): Double = {
        val partitions = examples.groupBy( e => e(attIdx) )
        val n = examples.length.toDouble
        partitions.values.foldLeft(entropy(examples, labelIdx))( (c, p) => { c - (p.length.toDouble/n) * entropy(p, labelIdx)} )
    }

    def runID3( examples: ListTerm, labelIdx: Int, atts: Array[Int], values: Array[Set[Term]]): Term = {
        val partitions = examples.groupBy( e => e(labelIdx) )
        val max = partitions.keySet.maxBy(k => partitions(k).length)
        val leftovers: Set[Term] = partitions.values.filter( k => k.length > 0 ).map( r => ListTerm(r)).toSet

        logger.info("Remaining attributes: " + { atts.mkString("[", ",", "]") })

        if ( atts.isEmpty || partitions.get(max).get.length == examples.length  ) {
        logger.info("- Leaf: " + max)
        max
        } else {
        val choice = atts.maxBy( i => information_gain(examples, i, labelIdx, values(i) ) )
        val choicePartitions = examples.groupBy( e => e(choice) )

        ListTerm(values(choice).map( x => {
            Tuple(Tuple(Sym("="), Num(choice-1), x), 
                if ( !choicePartitions.contains(x) ) {
                    val r = if ( includeAllLeafs && leftovers.nonEmpty ) { SetTerm(leftovers) } else { max }
                    logger.info("Leaf: " + r)
                    r
                } else {
                    logger.depth += 1
                    val r = runID3(ListTerm(choicePartitions(x)), labelIdx, atts.filter(_ != choice), values)
                    logger.depth -= 1
                    logger.info("Subtree: " + r)
                    r
                })
            }).toList)
        }
    }

    def resolve( dt: Term, x: ListTerm ): Term = dt match {
        case Sym(_) => dt
        case _=> resolve(dt.asList.find( b => x(b(0)(1).asInt.x.intValue) == b(0)(2)).get(1), x) 
    }
}