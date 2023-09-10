package org.aiddl.core.scala.function.`type`

import org.aiddl.core.scala.function.{Evaluator, Function, Verbose}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, FunRef, InfNeg, InfPos, Integer, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple}
import org.aiddl.core.scala.representation.conversion.{given_Conversion_Term_KeyVal, given_Conversion_Term_Sym}
import org.aiddl.core.scala.util.logger.Logger

import java.util.logging.Level
import scala.language.implicitConversions

protected[scala] class  TypeFunction(typeTerm: Term, eval: Evaluator) extends Function with Verbose {
  override def apply(x: Term): Term = Bool(check(typeTerm, x, 0))

  private def check(t: Term, x: Term, depth: Int): Boolean = {
    this.logger.message(Level.INFO, 1, s"Checking if $x has type $t")
    var r = t match {
      case Tuple(Sym("org.aiddl.type.set-of"), subType, args@_*) => Bool(x.isInstanceOf[SetTerm] && x.asSet.forall(e => this.check(subType, e, depth + 1)))
      case Tuple(Sym("org.aiddl.type.list-of"), subType, args@_*) => Bool(x.isInstanceOf[ListTerm] && x.asList.forall(e => this.check(subType, e, depth + 1)))
      case Tuple(Sym("org.aiddl.type.collection-of"), subType, args@_*) => Bool(x.isInstanceOf[CollectionTerm] && x.asCol.forall(e => this.check(subType, e, depth + 1)))
      case Tuple(Sym("org.aiddl.type.tuple.signed"), signature, args@_*) => if (x.isInstanceOf[Tuple]) {
        val rest = Tuple(args: _*)
        val min = rest.getOrElse(Sym("min"), Integer(signature.length)).asNum
        val max = rest.getOrElse(Sym("max"), Integer(signature.length)).asNum
        val repeat: Int = rest.getOrElse(Sym("repeat"), Integer(1)).asInt.asInt.x.toInt

        val repeat_start_idx: Int = signature.length - repeat
        val sigLen = signature.asList.length
        val tSize = Num(x.length)

        if (min <= tSize && tSize <= max.asNum) {
          Bool(x.asTup.zipWithIndex forall ({
            case (e, i) =>
              val sigIdx = if (i < sigLen) i
              else repeat_start_idx + ((i - sigLen) % repeat)
              this.check(signature(sigIdx), e, depth + 1)
          }))
        } else Bool(false)
      } else Bool(false)
      case Tuple(Sym("org.aiddl.type.matrix"), args@_*) =>
        if ((x.isInstanceOf[Tuple] || x.isInstanceOf[ListTerm]) && x.length > 0) {
          val rest = Tuple(args: _*)
          val colTypes = rest.get(Sym("col-types"))
          val rowTypes = rest.get(Sym("row-types"))
          val cellType = rest.get(Sym("cell-type"))
          val m: Int = rest.getOrElse(Sym("m"), Num(x.length)).intoInt
          val n: Int = rest.getOrElse(Sym("n"), {Num(x(0).length)}).intoInt

          val numRowMatches = rowTypes match {
            case Some(rts) => m == rts.length
            case None => true
          }
          val numColsMatches = colTypes match {
            case Some(cts) => n == cts.length
            case None => true
          }

          if ( !numRowMatches || !numColsMatches )
            throw new IllegalArgumentException(s"Matrix type has mismatch between specified number of row/column types and" +
              s" matrix size: $typeTerm")

          if (!numRowMatches || !numColsMatches || x.length != m) Bool(false)
          else {
            val mSat = (0 until m).forall(i => {
              x(i).length == n
                && (0 until n).forall(j => {
                val c1: Boolean = cellType match {
                  case None => true
                  case Some(t) => t(x(i)(j)).asBool.boolVal
                }
                val c2: Boolean = rowTypes match {
                  case None => true
                  case Some(t) => t(i)(x(i)(j)).asBool.boolVal
                }
                  val c3: Boolean = colTypes match {
                  case None => true
                  case Some(t) => t(j)(x(i)(j)).asBool.boolVal
                }
                c1 && c2 && c3
              })
            })
            Bool(mSat)
          }
        } else {
          Bool(false)
        }
      case Tuple(Sym("org.aiddl.type.dictionary"), keyTypeCol, args@_*) => {
        Bool(keyTypeCol.asCol.forall(
          {
            case KeyVal(key, subType) => x.get(key) match {
              case Some(e) => this.check(subType, e, depth + 1)
              case None => false
            }
            case _ => throw new IllegalArgumentException(s"Type $t must provide collection of key-value pairs.")
          }) && {
          t.get(Sym("optional")) match {
            case None => true
            case Some(o) =>
              o.asCol.forall( opt => {
                x.get(opt.key) match {
                  case None => true
                  case Some(e) => this.check(opt.value, e, depth + 1)
                }
              })
          }
        })
      }
      case Tuple(Sym("org.aiddl.type.enum"), domain, _*) => Bool(domain.asCol.contains((x)))
      case Tuple(Sym("org.aiddl.type.range"), args@_*) =>
        if (x.isInstanceOf[Num]) {
          val rest = Tuple(args: _*)
          val min: Num = rest.getOrElse(Sym("min"), InfNeg()).asNum
          val max: Num = rest.getOrElse(Sym("max"), InfPos()).asNum
          Bool(min <= x.asNum && x.asNum <= max)
        } else {
          Bool(false)
        }
      case Tuple(Sym("org.aiddl.type.typed-key-value"), KeyVal(keyType, valueType), _*) =>
        if (x.isInstanceOf[KeyVal])
          Bool(this.check(keyType, x.key, depth + 1) && this.check(valueType, x.value, depth + 1))
        else
          Bool(false)
      case Tuple(Sym("org.aiddl.type.union"), choices) => Bool(choices.asCol.exists(st => this.check(st, x, depth + 1)))
      case Tuple(Sym("org.aiddl.type.intersection"), choices) => Bool(choices.asCol.forall(st => this.check(st, x, depth + 1)))
      case uri: Sym => eval(Tuple(uri, x))
      case fType: FunRef => fType(x)
      case _ => {
        val resolved = eval(t)
        this.logger.message(Level.INFO, depth, s"$t resolved to: $resolved")
        if (resolved != t)
          Bool(this.check(resolved, x, depth + 1))
        else throw new IllegalArgumentException(s"Type definition $t did not match any type pattern")
      }
    }

    r = if (r.boolVal) t.get(Sym("constraint")) match {
      case Some(c) => eval(c)(x)
      case None => Bool(true)
    } else r
    this.logger.message(Level.INFO, depth, s"Answer: $r ($x is $t)")
    r match {
      case Bool(v) => v
      case _ => throw new IllegalArgumentException(s"#type expected to return Boolean value. Got: $r of type ${r.getClass.getSimpleName}")
    }
  }
}
