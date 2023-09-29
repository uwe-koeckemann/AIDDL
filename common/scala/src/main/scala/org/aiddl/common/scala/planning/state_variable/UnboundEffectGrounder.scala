package org.aiddl.common.scala.planning.state_variable

import org.aiddl.core.scala.function.Function
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.ComboIterator

/**
 * Ground unbound effects of operators based on variable domains
 */
class UnboundEffectGrounder extends Function {
  def apply(x: Term): Term = {
    val ops = x(Operators).asCol
    val domains = x(Domains).asCol

    var added: Set[Term] = Set.empty
    var removed: Set[Term] = Set.empty

    ops.foreach( o => {
      val signature = o(Signature)
      val preVars = Term.collect(x => x.isInstanceOf[Var])(o(Preconditions))
      val unboundEffectVars = Term.collect(x => x.isInstanceOf[Var] && !preVars.contains(x))(o(Effects))

      //println(s"PreVars: $preVars")
      //println(s"Unbound Eff: $unboundEffectVars")

      if ( !unboundEffectVars.isEmpty ) {
        val choices = unboundEffectVars.filter(signature.asCol.containsKey(_)).map(x => domains(signature(x)).asList.map(v => KeyVal(x, v)).toList)
        val comboIt = new ComboIterator(choices)

        removed = removed + o

        comboIt.foreach(a => {
          //println(s"Choice: ${a}")
          val s = Substitution.from(ListTerm(a))
          added = added + o\s
        })
      }})

    val newOps = ops.removeAll(SetTerm(removed)).addAll(SetTerm(added))
    x.asCol.put(KeyVal(Operators, newOps))
  }
}
