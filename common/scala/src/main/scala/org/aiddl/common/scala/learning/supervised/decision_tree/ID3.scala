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
    val LOG2 = Math.log(2);

    var includeAllLeafs = false
    var decisionTree: Term = NIL

    var values: Array[Set[Term]] = _
    var labelIdx: Int = 0

    def fit(x: ListTerm, y: ListTerm): Term = {
        val examples = ListTerm(
            y.zip(x).map( { (y, x) => ListTerm(y +: x.asList.list) } )
        )

        val numAtts = x.head.length
        val attributes = (1 until numAtts).toArray
        val init = Array.fill[Set[Term]](numAtts)(Set.empty)

        this.values = examples.foldLeft(init)( (c, e) => c.zip(e.asList).map( x => x match { case (ci, ei) => ci + ei } )  )

        this.includeAllLeafs = this.parameters.getOrElse(Sym("includeAllLeafs"), Bool(false)).boolVal

        this.decisionTree = runID3(examples, attributes)
        decisionTree
    }

    def predict(x: ListTerm): ListTerm =
        ListTerm(x.map( x_i => this.resolve(this.decisionTree, x_i)))

    private def runID3(examples: ListTerm, unusedAttributes: Array[Int] ): Term = {
        val partitions = examples.groupBy( e => e(labelIdx) )
        val mostCommonClass = partitions.keySet.maxBy(k => partitions(k).length)

        logger.info("Remaining attributes: " + { unusedAttributes.mkString("[", ",", "]") })

        if ( unusedAttributes.isEmpty || partitions(mostCommonClass).length == examples.length ) { //
            logger.info("- Leaf: " + mostCommonClass)
            assembleLeaf(mostCommonClass)
        } else {
            val choice = unusedAttributes.maxBy(i => computeInformationGain(examples, i) )
            val choicePartitions = examples.groupBy( e => e(choice) )

            ListTerm(values(choice).map( x => {
                Tuple(Tuple(Sym("="), Num(choice-1), x),
                    if ( !choicePartitions.contains(x) ) {
                        val leftovers: Set[Term] = partitions.values.filter( k => k.length > 0 ).map( r => ListTerm(r)).toSet
                        val r = if ( includeAllLeafs && leftovers.nonEmpty ) { SetTerm(leftovers) } else { mostCommonClass }
                        logger.info("Leaf: " + r)
                        assembleLeaf(r)
                    } else {
                        logger.depth += 1
                        val r = runID3(ListTerm(choicePartitions(x)), unusedAttributes.filter(_ != choice))
                        logger.depth -= 1
                        logger.info("Subtree: " + r)
                        r
                    })
                }).toList)
        }
    }

    private def assembleLeaf(value: Term): Term =
        SetTerm(KeyVal(Class, value))

    private def computeInformationGain(examples: ListTerm, attIdx: Int): Double = {
        val baseEntropy = computeEntropy(examples)
        val n = examples.length.toDouble
        val entropyOfAttributeGroups =
            examples
              .groupBy(_(attIdx))
              .values
              .map(p => (p.length.toDouble / n) * computeEntropy(p))
              .sum
        baseEntropy - entropyOfAttributeGroups
    }

    private def computeEntropy(examples: Seq[Term]): Double = {
        val n = examples.length.toDouble
        examples
          .groupBy(x => x(labelIdx))
          .values
          .map(p => {
            val pi = p.length / n
            -pi * (Math.log(pi) / LOG2)
          }).sum
    }

    private def resolve( dt: Term, x: ListTerm ): Term = dt match {
        case SetTerm(_) => {
            dt(Class)
        }
        case _ => resolve(dt.asList.find( b => x(b(0)(1).asInt.x.intValue) == b(0)(2)).get(1), x)
    }
}