package org.aiddl.common.scala.learning

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.parser.Parser

import org.aiddl.common.scala.learning.supervised.decision_tree.ID3
import org.aiddl.common.scala.learning.supervised.DataSplitter

import org.aiddl.core.scala.tools.Logger

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm

class Id3Suite extends AnyFunSuite {
    test("Running ID3 on book example") {
        val c = new Container()
        val m = Parser.parseInto("../test/learning/classification/problem-01.aiddl", c)
        val p = c.resolve(c.getEntry(m, Sym("problem")).get.v)

        assert(c.typeCheckModule(m))

        val f_split = new DataSplitter
        val f_id3 = new ID3

        val data = f_split(p)
        val dt = f_id3.fit(data(0).asList, data(1).asList)
        val example = Parser.parse("[[Sunny Cool Normal Weak]]").head

        assert( ListTerm(Sym("Yes")) == f_id3.predict(example.asList) )
        //[((= Outlook Sunny) [((= Humidity High) No) ((= Humidity Normal) Yes)]) ((= Outlook Overcast) Yes) ((= Outlook Rain) [((= Wind Weak) Yes) ((= Wind Strong) No)])]
        //[((= Outlook Sunny) [((= Humidity High) No) ((= Humidity Normal) Yes)]) ((= Outlook Overcast) Yes) ((= Outlook Rain) [((= Wind Weak) Yes) ((= Wind Strong) No)])]
    }
}
