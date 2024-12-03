package org.aiddl.external.grpc.scala.converter

import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.container.Container

import org.aiddl.external.grpc.aiddl.{
  Term => PbTerm,
  EntRef => PbEntRef,
  Rational => PbRat,
  CollectionType,
  OtherNumerical,
  CollectionTerm => PbCol,
  KeyVal => PbKvp
}

class Converter(c: Container) {

  /**
   * Recusively convert protobuf messages to AIDDL Terms
   * @param x a Protobuf message
   * @return corresponding AIDDL term
   */
  def pb2aiddl(x: PbTerm): Term = {
    x.termType match {
      case PbTerm.TermType.Sym(name) =>
        Sym(name)
      case PbTerm.TermType.Var("_") =>
        Var()
      case PbTerm.TermType.Var(name) =>
        Var(name)
      case PbTerm.TermType.Str(value) =>
        Str(value)
      case PbTerm.TermType.Boolean(value) =>
        Bool(value)
      case PbTerm.TermType.Int(x) =>
        Num(x)
      case PbTerm.TermType.Real(x) =>
        Num(x)
      case PbTerm.TermType.Kvp(PbKvp(Some(k), Some(v), _)) =>
        KeyVal(pb2aiddl(k), pb2aiddl(v))
      case PbTerm.TermType.Rational(PbRat(n, d, _)) =>
        Num(n, d)
      case PbTerm.TermType.OtherNumerical(OtherNumerical.INF_POS) =>
        InfPos()
      case PbTerm.TermType.OtherNumerical(OtherNumerical.INF_NEG) =>
        InfNeg()
      case PbTerm.TermType.OtherNumerical(OtherNumerical.NAN) =>
        NaN()
      case PbTerm.TermType.Collection(PbCol(CollectionType.LIST, data, _)) =>
        ListTerm(data.map(this.pb2aiddl(_)))
      case PbTerm.TermType.Collection(PbCol(CollectionType.TUPLE, data, _)) =>
        Tuple(data.map(this.pb2aiddl(_))*)
      case PbTerm.TermType.Collection(PbCol(CollectionType.SET, data, _)) =>
        SetTerm(data.map(this.pb2aiddl(_)).toSet)
      case PbTerm.TermType.EntRef(PbEntRef(Some(module), Some(name), Some(alias), _)) =>
        EntRef(pb2aiddl(module).asSym, pb2aiddl(name), pb2aiddl(alias).asSym)
      case PbTerm.TermType.FunRef(uri) =>
        FunRef.create(Sym(uri), f => c.getFunctionOrPanic(f))
      case _ => {
        throw new IllegalArgumentException(s"Failed Protobuf to AIDDL conversion for: $x")
      }
    }
  }

  /**
   * Convert AIDDL terms to a Protobuf message
   * @param x AIDDL term
   * @return corresponding Protobuf message
   */
  def aiddl2pb(x: Term): PbTerm = {
    x match {
      case term: CollectionTerm =>
        val data = term.map(e => aiddl2pb(e)).toSeq
        val colType: CollectionType = {
          if x.isInstanceOf[SetTerm] then CollectionType.SET
          else CollectionType.LIST
        }
        PbTerm()
          .withCollection(PbCol()
            .withColType(colType)
            .withData(data))
      case Sym(name) => PbTerm().withSym(name)
      case Var(name) => {
        if (name.startsWith("_"))
          PbTerm().withVar("_")
        else
          PbTerm().withVar(name)
      }
      case Str(value) => PbTerm().withStr(value)
      case KeyVal(key, value) =>
        PbTerm().withKvp(
          PbKvp()
            .withKey(aiddl2pb(key))
            .withValue(aiddl2pb(value)))
      case EntRef(mod, name, alias) =>
        PbTerm().withEntRef(PbEntRef()
          .withModule(aiddl2pb(mod))
          .withName(aiddl2pb(name))
          .withAlias(aiddl2pb(alias)))
      case Tuple(x@_*) =>
        val data = x.map(e => aiddl2pb(e))
        PbTerm().withCollection(PbCol()
          .withColType(CollectionType.TUPLE)
          .withData(data))
      case Bool(v) => PbTerm().withBoolean(v)
      case num: Num => num match {
        case Real(x) => PbTerm().withReal(x)
        case Integer(x) =>  PbTerm().withInt(x)
        case Rational(n, d) => PbTerm().withRational(
          PbRat()
            .withNominator(n)
            .withDenominator(d))
        case InfPos() => PbTerm().withOtherNumerical(OtherNumerical.INF_POS)
        case InfNeg() => PbTerm().withOtherNumerical(OtherNumerical.INF_NEG)
        case NaN() => PbTerm().withOtherNumerical(OtherNumerical.NAN)
      }
      case f: FunRef => PbTerm().withFunRef(f.uri.toString)
    }
  }

}
