package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, KeyVal, ListTerm, SetTerm, Sym, Term, Tuple, Num, Var}
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.core.scala.util.ComboIterator

class SampleDataExtractor extends Function with Verbose {
  private val DataKey = Sym("new-data")
  private val StateKey = Sym("new-state")

  private val COLLECTED = Sym("collected")
  private val DATA = Sym("data")
  private val SAMPLE = Sym("sample")
  private val EMPTY = Sym("empty")
  private val MATCH = KeyVal(Tuple(COLLECTED, Tuple(DATA, Var("L"), Var("C"))), Sym("true"))
  private val MATCH_SAMPLE = KeyVal(Tuple(SAMPLE, Var()), Var())
  private val DATA_KEY = Tuple(Var("L"), Var("C"))

  override def apply(x: Term): Term = {
    val state = x(Sym("state")).asCol
    val model = x(Sym("model")).asCol
    var data = x(Sym("data")).asList.list

    var s_updated: Set[Term] = Set.empty
    var new_data: List[Term] = Nil

    for (p <- state) {
      val p_sv = p.asKvp.key
      val p_a = p.asKvp.value
      val sva = KeyVal(p_sv, p_a)
      MATCH unify sva match {
        case Some(s) => {
          val key = DATA_KEY \ s
          val observed = model(key)
          new_data = ListTerm(key(0), key(1), observed) :: new_data
        }
        case None => {
          MATCH_SAMPLE unify p match
            case Some(s) => s_updated = s_updated + KeyVal(p_sv, EMPTY)
            case None => s_updated = s_updated + p
        }
      }
    }

    this.logger.info("Extracted " + new_data.size + " samples.")
    for (t <- new_data) {
      this.logger.fine("  " + t.toString)
    }

    ListTerm(
      KeyVal(DataKey, ListTerm(data ++ new_data)),
      KeyVal(StateKey, SetTerm(s_updated)))
  }
}
