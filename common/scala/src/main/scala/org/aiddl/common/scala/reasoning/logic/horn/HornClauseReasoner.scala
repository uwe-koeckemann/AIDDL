package org.aiddl.common.scala.reasoning.logic.horn

import org.aiddl.common.scala.search.GenericTreeSearch
import org.aiddl.core.scala.function.{Function, Initializable}
import org.aiddl.core.scala.representation.*

class HornClauseReasoner(kb: ListTerm) extends GenericTreeSearch[ProofStep, Substitution] with Initializable {
  private var currentVarId = 0
  private var openGoals: List[ListTerm] = Nil
  private var query: ListTerm = ListTerm.empty

  override def init(args: Term): Unit = {
    this.openGoals = List(args.asList)
    this.query = args.asList
    this.reset
    this.currentVarId = 0
  }

  private def nextVarId: Int = {
    currentVarId += 1
    currentVarId
  }

  override def node(choices: Seq[ProofStep]): Option[Term] = {
    val currentGoals = choices.reverse.foldLeft(this.query)((curr, step) => applyProofStep(curr, step))
    Some(currentGoals)
  }

  private def applyProofStep(goals: ListTerm, step: ProofStep): ListTerm = {
    val remainingGoals = goals.tail
    val derivationOpenGoals = step.goals
    (ListTerm(derivationOpenGoals ++ remainingGoals) \ step.substitution).asList
  }

  override val nil: ProofStep =
    ProofStep(ListTerm.empty, new Substitution())

  override def expand: Option[Seq[ProofStep]] =
    if openGoals.head.isEmpty
    then None
    else {
      val goal = openGoals.head.head
      val derivatives =
        kb.flatMap( statement =>
          this.makeVariablesUnique(statement) match {
            case fact: Tuple =>
              (goal unify fact).flatMap(substitution => Some(ProofStep(ListTerm.empty, substitution)))
            case rule: KeyVal =>
              (rule.key unify goal).flatMap(substitution => Some(ProofStep((rule.value \ substitution).asList, substitution)))
            case _ => None
          })
      Some(derivatives)
    }

  private def makeVariablesUnique(term: Term): Term = {
    val variables = Term.collect(_.isInstanceOf[Var])(term).toSet
    val substitution = new Substitution()
    variables.foreach(x => substitution.add(x, Var(s"#$nextVarId")))
    term \ substitution
  }

  override def expandHook: Unit = {
    openGoals = ListTerm.empty :: openGoals
  }

  override def choiceHook: Unit = {
    val lastChoice = this.searchSpace.head(this.searchIdx.head)
    val remainingGoals = this.openGoals.tail.head.tail
    val derivationOpenGoals = lastChoice.goals
    val newOpenGoals = (ListTerm(derivationOpenGoals ++ remainingGoals) \ lastChoice.substitution).asList
    openGoals = newOpenGoals :: openGoals.tail
  }

  override def backtrackHook: Unit =
    val numDropped = openGoals.length - (choice.length + 1)
    openGoals = openGoals.drop(numDropped)

  override def assembleSolution(choice: List[ProofStep]): Option[Substitution] = {
    val initialGoal: Term = openGoals.reverse.head
    var subbedGoal = initialGoal
    for (c <- choice.reverse) {
      subbedGoal = subbedGoal \ c.substitution
    }
    initialGoal unify subbedGoal
  }
}
