package org.aiddl.common.scala.learning

import org.aiddl.common.scala.learning.supervised.DataSplitter
import org.aiddl.common.scala.learning.supervised.decision_tree.ID3
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger
import org.scalatest.funsuite.AnyFunSuite

import java.util.logging.Level

class Id3Suite extends AnyFunSuite {
    test("Running ID3 on book example") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/learning/classification/problem-01.aiddl")
        val p = c.getProcessedValueOrPanic(m, Sym("problem"))

        assert(c.typeCheckModule(m))

        val f_split = new DataSplitter
        val f_id3 = new ID3

        val data = f_split(p)
        val dt = f_id3.fit(data(0).asList, data(1).asList)

        val dtTypeChecker = c.getFunctionOrPanic(Sym("org.aiddl.common.learning.supervised.decision-tree.DecisionTree"))

        assert( dtTypeChecker(dt).boolVal )

        val example = parser.str("[[Sunny Cool Normal Weak]]")

        assert( ListTerm(Sym("Yes")) == f_id3.predict(example.asList) )
        //[((= Outlook Sunny) [((= Humidity High) No) ((= Humidity Normal) Yes)]) ((= Outlook Overcast) Yes) ((= Outlook Rain) [((= Wind Weak) Yes) ((= Wind Strong) No)])]
        //[((= Outlook Sunny) [((= Humidity High) No) ((= Humidity Normal) Yes)]) ((= Outlook Overcast) Yes) ((= Outlook Rain) [((= Wind Weak) Yes) ((= Wind Strong) No)])]
    }
}
