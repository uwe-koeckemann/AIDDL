package org.aiddl.common.scala.tools.java

import org.aiddl.core.container.Container
import org.aiddl.core.function.FunctionRegistry
import org.aiddl.core.interfaces.{Function => JFunction}
import org.aiddl.core.representation.{BooleanTerm => JBool, FunctionReferenceTerm => JFun, InfinityTerm => JInf, IntegerTerm => JInt, KeyValueTerm => JKvp, ListTerm => JListTerm, RationalTerm => JRat, RealTerm => JReal, ReferenceTerm => JRef, SetTerm => JSetTerm, StringTerm => JString, SymbolicTerm => JSym, Term => JTerm, TupleTerm => JTuple, VariableTerm => JVar}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._
import org.aiddl.core.tools.{LockableList, LockableSet}

import scala.collection.JavaConverters._

class JavaFunctionWrapper( f_j: JFunction ) extends Function {
  def apply( x: Term ): Term = AiddlJavaConverter.j2s(f_j(AiddlJavaConverter.s2j(x)))
}
