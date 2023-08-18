package org.aiddl.common.scala.learning.supervised

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.learning.LearningTerm._

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_ListTerm

import scala.language.implicitConversions

class DataSplitter extends Function {

    def apply( ml: Term ): Term = {
        val atts = ml(Attributes).asList
        val label = ml(Label)
        val data = ml(Data)
        val (x, y) = this(atts, label, data)
        Tuple(x, y)
    }

    def apply(attributes: ListTerm, label: Term, data: CollectionTerm): (ListTerm, ListTerm) = {
        val labelIdx = attributes.indexWhere(x => x == label)
        val r = this.split(data, labelIdx)
        (r._1, r._2)
    }

    def split( data: ListTerm, idx: Int ): (ListTerm, ListTerm) = {
        val x = ListTerm(data.map( x => { val s = x.asList.splitAt(idx); ListTerm(s._1 ++ s._2.tail) } ))
        val y = ListTerm(data.map(x => x(idx)))
        (x, y)
    }
}

