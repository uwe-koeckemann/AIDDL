package org.aiddl.common.scala.reasoning.constraint.domain

import org.aiddl.common.scala.reasoning.constraint.AllDifferentConstraint
import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.{Constraints, Domains, Variables}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

object Sudoku {
  private def createVar(i: Int, j: Int): Var =
    Var(s"X$i$j")

  def assignment(i: Int, j: Int, value: Int): KeyVal =
    KeyVal(createVar(i, j), Num(value))

  def print(assignment: CollectionTerm): String =
    (1 to 9).map( i => (1 to 9).map(j => {
      assignment.get(createVar(i, j)) match {
        case Some(value) => s"$value"
        case None => "-"
      }}).mkString(" ")).mkString("\n")
}

class Sudoku(known: ListTerm) extends Function {
  override def apply(x: Term): Term = {
    ListTerm(
      KeyVal(Variables, this.variables),
      KeyVal(Domains, domains),
      KeyVal(Constraints,
        ListTerm(constraints))
    )
  }

  private val knownSub = Substitution.from(known)

  val variables: ListTerm = ListTerm((1 to 9).flatMap(i => {
    (1 to 9).map(j => Sudoku.createVar(i, j))
  }).filter(!known.containsKey(_)).toList)

  val domains: ListTerm = ListTerm(
    variables.map(x => KeyVal(x, ListTerm((1 to 9).map(Num(_))))).toList
  )

  private val rowConstraints =
    (1 to 9).map(i => {
      val scope = Tuple((1 to 9).map(j => Sudoku.createVar(i, j)): _*) \ knownSub
      Tuple(scope, FunRef(Sym(s"row-$i"), new AllDifferentConstraint(9)))
    }).toList

  private val colConstraints =
    (1 to 9).map(j => {
      val scope = Tuple((1 to 9).map(i => Sudoku.createVar(i, j)): _*) \ knownSub
      Tuple(scope, FunRef(Sym(s"col-$j"), new AllDifferentConstraint(9)))
    }).toList

  private val blockConstraints =
    (1 to 3).flatMap(b_i => {
      (1 to 3).map(b_j => {
        val scope = Tuple((b_i*3-2 to (b_i*3)).flatMap(i => {
          (b_j*3-2 to (b_j*3)).map(j => {
            Sudoku.createVar(i, j)
          })
        }): _*) \ knownSub
        Tuple(scope, FunRef(Sym(s"block-$b_i-$b_j"), new AllDifferentConstraint(9)))
      })
    }).toList

  val constraints: Seq[Tuple] =
    this.colConstraints ++ this.rowConstraints ++ this.blockConstraints
}
