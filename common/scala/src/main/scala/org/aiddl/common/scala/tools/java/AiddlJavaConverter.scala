package org.aiddl.common.scala.tools.java

import org.aiddl.core.container.Container
import org.aiddl.core.function.FunctionRegistry
import org.aiddl.core.interfaces.{Function => JFunction}
import org.aiddl.core.representation.{BooleanTerm => JBool, FunctionReferenceTerm => JFun, InfinityTerm => JInf, IntegerTerm => JInt, KeyValueTerm => JKvp, ListTerm => JListTerm, RationalTerm => JRat, RealTerm => JReal, ReferenceTerm => JRef, SetTerm => JSetTerm, StringTerm => JString, SymbolicTerm => JSym, Term => JTerm, TupleTerm => JTuple, VariableTerm => JVar}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._
import org.aiddl.core.tools.{LockableList, LockableSet}

import scala.collection.JavaConverters._

object AiddlJavaConverter {
  def s2j(t: Term): JTerm = t match {
    case Bool(v) => JTerm.bool(v)
    case Sym(name) => JTerm.sym(name)
    case Str(str) => JTerm.string(str)
    case Var(name) => JTerm.`var`(name)
    case Integer(n) => JTerm.integer(n)
    case Rational(n, d) => JTerm.rational(n, d)
    case Real(x) => JTerm.real(x)
    case InfPos() => JTerm.infPos
    case InfNeg() => JTerm.infNeg
    case KeyVal(k, v) => JTerm.keyVal(s2j(k), s2j(v))
    case FunRef(uri, f) => {
      val juri = s2j(uri).asSym
      val f_j = ScalaFunctionWrapper(f)
      val freg = new FunctionRegistry()
      freg.addFunction(juri, f_j)
      JTerm.fref(juri, freg)
    }
    case Tuple(args@_*) => {
      val l = new LockableList
      args.foreach(x => l.add(s2j(x)))
      JTerm.tuple(l)
    }
    case ListTerm(list) => {
      val l = new LockableList
      list.foreach(x => l.add(s2j(x)))
      JTerm.list(l)
    }
    case SetTerm(set) => {
      val s = new LockableList
      set.foreach(x => s.add(s2j(x)))
      JTerm.set(s)
    }
    case _ => throw new IllegalArgumentException("Conversion not supported: " + t + "\n-> Consider resolving references first.");
  }

  def j2s(t: JTerm): Term = t match {
    case b: JBool => Bool(b.getBooleanValue)
    case s: JSym => Sym(s.getStringValue)
    case s: JString => Str(s.getStringValue)
    case v: JVar => if (v.toString != "_") Var(v.getStringValue) else Var()
    case n: JInt => Num(n.getIntValue)
    case r: JRat => Rational(r.getNumerator, r.getDenominator)
    case x: JReal => Real(x.getDoubleValue)
    case inf: JInf => if (inf.isPositive) InfPos() else InfNeg()
    case kvp: JKvp => KeyVal(j2s(kvp.getKey), j2s(kvp.getValue))
    case fr: JFun => FunRef(j2s(fr.getFunRefTerm).asSym, new JavaFunctionWrapper(fr.getFunction))
    case t: JTuple => {
      val args = for (i <- 0 until t.size) yield j2s(t.get(i))
      Tuple(args: _*)
    }
    case l: JListTerm => {
      val args = for (i <- 0 until l.size) yield j2s(l.get(i))
      ListTerm(args)
    }
    case s: JSetTerm => {
      var args: Set[Term] = Set()
      s.forEach(x => {
        args = args + j2s(x); ()
      })
      SetTerm(args)
    }
    case _ => throw new IllegalArgumentException("Conversion not supported: " + t + "\n-> Consider resolving references first.");
  }

}
