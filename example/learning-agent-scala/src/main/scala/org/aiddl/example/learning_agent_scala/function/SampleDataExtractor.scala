package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple, Var}
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.core.scala.util.ComboIterator

import java.util.logging.Level

class SampleDataExtractor(val hiddenModel: Term) extends Function with Verbose {
  this.logConfig(level=Level.FINE)
  private val DataKey = Sym("new-data")
  private val StateKey = Sym("new-state")

  private val COLLECTED = Sym("collected")
  private val DATA = Sym("data")
  private val SAMPLE = Sym("sample")
  private val EMPTY = Sym("empty")
  private val MATCH = KeyVal(Tuple(COLLECTED, Tuple(DATA, Var("L"), Var("C"))), Bool(true))
  private val RESET_COL = KeyVal(Tuple(COLLECTED, Tuple(DATA, Var("L"), Var("C"))), Bool(false))
  private val MATCH_SAMPLE = KeyVal(Tuple(SAMPLE, Var()), Var())
  private val DATA_KEY = Tuple(Var("L"), Var("C"))

  override def apply(x: Term): Term = {
    val state = x(Sym("state")).asCol
    val model = x(Sym("model")).asCol
    val data = x(Sym("data")).asList

    val (newData, nextState) = this(state, data)
    ListTerm(
      KeyVal(DataKey, data),
      KeyVal(StateKey, state)
    )
  }

  def apply(state: CollectionTerm, data: ListTerm): (ListTerm, SetTerm) = {
    var s_updated: Set[Term] = Set.empty
    var new_data: List[Term] = Nil

    for (p <- state) {
      val p_sv = p.asKvp.key
      val p_a = p.asKvp.value
      val sva = KeyVal(p_sv, p_a)
      //println(p)
      MATCH unify sva match {
        case Some(s) => {
          //println("MATCH: $s")
          val key = DATA_KEY \ s
          val observed = hiddenModel(key)
          new_data = ListTerm(key(0), key(1), observed) :: new_data
          s_updated = s_updated + RESET_COL \ s
        }
        case None => {
          //println("NO MATCH")
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

    (data.addAll(ListTerm(new_data)), SetTerm(s_updated))
  }
}
