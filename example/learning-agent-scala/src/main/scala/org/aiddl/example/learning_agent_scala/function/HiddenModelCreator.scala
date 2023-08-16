package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{KeyVal, ListTerm, Sym, Term, Tuple}

import scala.util.Random

class HiddenModelCreator extends Function {

  private val rng = new Random()

  override def apply(x: Term): Term = {
    val locations = x(Sym("locations")).asCol
    val configs = x(Sym("configs")).asCol
    val label = x(Sym("label")).asList
    var list: List[Term] = Nil

    for ( l <- locations ) {
      for ( c <- configs ) {
        list = KeyVal(
          Tuple(l, c),
          label(rng.nextInt(label.size))
        ) :: list
      }
    }
    ListTerm(list)
  }

}
