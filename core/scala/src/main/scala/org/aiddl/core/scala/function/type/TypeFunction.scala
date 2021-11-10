package org.aiddl.core.scala.function.`type`

import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, FunRef, InfNeg, InfPos, Integer, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple}
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermUnpackImplicits.term2int
import org.aiddl.core.scala.representation.BoolImplicits.term2Boolean

class  TypeFunction(typeTerm: Term, eval: Evaluator) extends Function with Verbose {
  override def apply(x: Term): Term = Bool(check(typeTerm, x))

  def check(t: Term, x: Term): Boolean = {
    logInc(1, s"Checking if $x has type $t")
    var r = t match {
      case Tuple(Sym("org.aiddl.type.set-of"), subType, args@_*) if (x.isInstanceOf[SetTerm]) => Bool(x.asSet.forall(e => this.check(subType, e)))
      case Tuple(Sym("org.aiddl.type.list-of"), subType, args@_*) if (x.isInstanceOf[ListTerm]) => Bool(x.asList.forall(e => this.check(subType, e)))
      case Tuple(Sym("org.aiddl.type.collection-of"), subType, args@_*) if (x.isInstanceOf[CollectionTerm]) => Bool(x.asCol.forall(e => this.check(subType, e)))
      case Tuple(Sym("org.aiddl.type.tuple.signed"), signature, args@_*) => if (x.isInstanceOf[Tuple]) {
        val rest = Tuple(args: _*)
        val min = rest.getOrElse(Sym("min"), Integer(signature.length))
        val max = rest.getOrElse(Sym("max"), Integer(signature.length))
        val repeat: Int = rest.getOrElse(Sym("repeat"), Integer(1)).asInt.asInt.x.toInt

        val repeat_start_idx: Int = signature.length - repeat
        val sigLen = signature.asList.length
        val tSize = Num(x.length)

        if (min <= tSize && tSize <= max) {
          Bool(x.asTup.zipWithIndex forall ({
            case (e, i) => {
              val sigIdx = if (i < sigLen) i
              else repeat_start_idx + ((i - sigLen) % repeat)
              this.check(signature(sigIdx), e)
            }
          }))
        } else Bool(false)
      } else Bool(false)
      case Tuple(Sym("org.aiddl.type.matrix"), args@_*) => if ((x.isInstanceOf[Tuple] || x.isInstanceOf[ListTerm]) && x.length > 0) {
        val rest = Tuple(args: _*)
        val colTypes = rest.get(Sym("col-types"))
        val rowTypes = rest.get(Sym("row-types"))
        val cellType = rest.get(Sym("cell-type"))
        val m: Int = rest.getOrElse(Sym("m"), x.length)
        val n: Int = rest.getOrElse(Sym("n"), x(0).length)

        if (x.length != m) Bool(false)
        else {
          val mSat = (0 until m).forall(i => {
            x(i).length == n
              && (0 until n).forall(j => {
              val c1: Boolean = cellType match {
                case None => true
                case Some(t) => t(x(i)(j))
              }
              val c2: Boolean = rowTypes match {
                case None => true
                case Some(t) => t(i)(x(i)(j))
              }
              val c3: Boolean = colTypes match {
                case None => true
                case Some(t) => t(j)(x(i)(j))
              }
              c1 && c2 && c3
            })
          })
          Bool(mSat)
        }
      } else Bool(false)
      case Tuple(Sym("org.aiddl.type.map"), keyTypeCol, args@_*) => {
        Bool(keyTypeCol.asCol.forall(
          {
            case KeyVal(key, subType) => x.get(key) match {
              case Some(e) => this.check(subType, e)
              case None => false
            }
            case _ => throw new IllegalArgumentException("Type " + t + " must provide collection of key-value pairs.")
          }))
      }
      case Tuple(Sym("org.aiddl.type.enum"), domain, _*) => Bool(domain.asCol.contains((x)))
      case Tuple(Sym("org.aiddl.type.range"), args@_*) =>
        if (x.isInstanceOf[Num]) {
          val rest = Tuple(args: _*)
          val min: Num = rest.getOrElse(Sym("min"), InfNeg())
          val max: Num = rest.getOrElse(Sym("max"), InfPos())
          Bool(min <= x && x <= max)
        } else {
          Bool(false)
        }
      case Tuple(Sym("org.aiddl.type.typed-key-value"), KeyVal(keyType, valueType), _*) =>
        if (x.isInstanceOf[KeyVal])
          Bool(this.check(keyType, x.key) && this.check(valueType, x.value))
        else
          Bool(false)
      case Tuple(Sym("org.aiddl.type.union"), choices) => Bool(choices.asCol.exists(st => this.check(st, x)))
      case uri: Sym => eval(Tuple(uri, x))
      case fType: FunRef => fType(x)
      case _ => {
        val resolved = eval(t)
        log(1, s"$t resolved to: $resolved")
        if (resolved != t)
          Bool(this.check(resolved, x))
        else throw new IllegalArgumentException(s"Type definition $t did not match any type pattern")
      }
    }

    r = if (r.boolVal) t.get(Sym("constraint")) match {
      case Some(c) => eval(c)(x)
      case None => Bool(true)
    } else
      r

    logDec(1, s"Answer: $r ($x is $t)")
    r match {
      case Bool(v) => v
      case _ => throw new IllegalArgumentException(s"#type expected to return Boolean value. Got: $r of type ${r.getClass.getSimpleName}")
    }
  }

}
