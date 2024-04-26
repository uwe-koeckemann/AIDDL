package org.aiddl.common.scala.reasoning.logic

import org.aiddl.common.scala.math.graph.Graph2Dot
import org.aiddl.common.scala.math.graph.GraphType.Directed
import org.aiddl.common.scala.math.graph.Terms.Nodes
import org.aiddl.common.scala.reasoning.logic.horn.HornClauseReasoner
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

import java.util.logging.Level

class HornClauseReasonerSuite extends AnyFunSuite {
  val container = new Container
  val parser = new Parser(container)

  test("KB directly contains query fact") {
    val kb = parser.str("[(p A B) (q C D) a]").asList
    val query = parser.str("[(q ?x D)]").asList

    val reasoner = new HornClauseReasoner(kb)
    reasoner.init(query)
    val answer = reasoner.search
    assert(answer.get == Substitution.from(parser.str("{?x:C}").asCol))
    assert(reasoner.search.isEmpty)
  }

  test("KB does not contain query fact") {
    val kb = parser.str("[(p A B) (q C D)]").asList
    val query = parser.str("[(z ?x D)]").asList

    val reasoner = new HornClauseReasoner(kb)
    reasoner.init(query)
    assert(reasoner.search.isEmpty)
  }

  test("There are multiple answers to query") {
    val kb = parser.str("[(p A B) (q C D) (q D D) (q E D)]").asList
    val query = parser.str("[(q ?x D)]").asList

    val reasoner = new HornClauseReasoner(kb)
    reasoner.init(query)

    assert(reasoner.search.get == Substitution.from(parser.str("{?x:C}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?x:D}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?x:E}").asCol))
    assert(reasoner.search.isEmpty)
  }

  test("Query with two goals") {
    val kb = parser.str("[(p A B) (p A C) (q C D) (q D D) (q E D)]").asList
    val query = parser.str("[(p A ?y) (q ?x D)]").asList

    val reasoner = new HornClauseReasoner(kb)
    reasoner.init(query)

    assert(reasoner.search.get == Substitution.from(parser.str("{?y:B ?x:C}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?y:B ?x:D}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?y:B ?x:E}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?y:C ?x:C}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?y:C ?x:D}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?y:C ?x:E}").asCol))
    assert(reasoner.search.isEmpty)
  }

  test("Query requires a rule") {
    val kb = parser.str("[(p A) (p B) (q ?x):[(p ?x)]]").asList
    val query = parser.str("[(q ?x)]").asList

    val reasoner = new HornClauseReasoner(kb)
    reasoner.init(query)

    assert(reasoner.search.get == Substitution.from(parser.str("{?x:A}").asCol))
    assert(reasoner.search.get == Substitution.from(parser.str("{?x:B}").asCol))
    assert(reasoner.search.isEmpty)
  }

  test("Query requires a recursive rule") {
    val kb = parser.str("[(p A B) (p B C) (t ?A ?B):[(p ?A ?B)] (t ?A ?B):[(p ?A ?Z) (t ?Z ?B)]]").asList
    val query = parser.str("[(t A C)]").asList

    val reasoner = new HornClauseReasoner(kb)
    reasoner.init(query)
    assert(reasoner.search.get == Substitution.from(parser.str("{}").asCol))
    assert(reasoner.search.isEmpty)
  }

  test("Query requires a rule (flip)") {
    val kb = parser.str("[(p A B) (t ?A ?B):[(p ?A ?B)] (t ?A ?B):[(p ?B ?A)]]").asList
    val query = parser.str("[(t A B)]").asList

    val reasoner = new HornClauseReasoner(kb) {
      traceFlag = true
    }
    reasoner.init(query)
    assert(reasoner.search.get == Substitution.from(parser.str("{}").asCol))
    assert(reasoner.search.isEmpty)
    assert(reasoner.graph(Nodes).asCol.size == 4)
  }
}
