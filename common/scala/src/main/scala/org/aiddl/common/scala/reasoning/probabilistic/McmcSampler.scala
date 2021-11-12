package org.aiddl.common.scala.reasoning.probabilistic

import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.mutable.LinkedHashSet
import scala.util.Random
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm

import scala.collection.mutable

object McmcSampler {
  def apply( r: Random ): McmcSampler = {
    val s = new McmcSampler
    s.r = r
    s
  }
}

class McmcSampler extends InferenceFunction with Verbose {
  var nSamples = 100
  var r = new Random()

  private val parents = new HashMap[Term, ListTerm]()
  private val probIndex = new HashMap[Term, Int]().withDefaultValue(0)
  private val parentImpact = new HashMap[(Term, Term), Int]()
  private val children = new HashMap[Term, List[Term]]().withDefaultValue(Nil)
  private val pCond = new HashMap[Term, CollectionTerm]()
  private val values = new HashMap[Term, ListTerm]()
  private val valueIndex = new mutable.HashMap[(Term, Term), Int]()

  var variables: List[Term] = Nil

  override def init( bn: Term ) = {
    parents.clear(); children.clear(); pCond.clear(); values.clear();
    valueIndex.clear(); probIndex.clear(); parentImpact.clear();
    variables = Nil
    bn.asCol.foreach( cpt => {
      val x = cpt(0)
      variables = x :: variables
      parents.put(x, cpt(1))
      //val impactList = calcParentMult(cpt(1).asList.list.toList, values)
      //cpt(1).zip(impactList).foreach( (p, i) => parentImpact.put((x, p), i  ))
      values.put(x, cpt(2))
      //cpt(2).zipWithIndex.foreach( (v, i) => valueIndex.put( (x, v), i ) )
      pCond.put(x, cpt(3))
      cpt(1).foreach( p => children.put(p, x :: children(p)) )
    })
  }

  private def calcParentMult( ps: Seq[Term], values: HashMap[Term, ListTerm] ): List[Int] =
    ps match {
      case x :: Nil => List(1)
      case x :: y :: tail => {
        val r = calcParentMult(ps.tail, values)
        (r.head * values(y).length) :: r
      }
      case _ => Nil
    }

  private def updateSample( s: HashMap[Term, Term], k: Term, v: Term ) = {
    //val p = s(k)
    s.put(k, v)

    //variables.view.filter( parents(_).contains(k) ).foreach( x => {
    //  val impact = parentImpact((x, k))
    //  probIndex.put( x, probIndex(x) - impact * valueIndex((k, p)) + impact * valueIndex((k, v)))
    //})
  }


  override def apply( x: Term, es: CollectionTerm ): ListTerm = {
    val n = new HashMap[Term, Num]().withDefaultValue(0)
    val zs = variables.filter( v =>  !es.exists( e => e.key == v ) )
    val sample = new HashMap[Term, Term]()
    es.foreach( e => sample.put(e.key, e.value) )
    zs.foreach( z => sample.put(z, values(z)(r.nextInt(values(z).length))) )

    log(1, s"Evidence: $es")
    log(1, s"Flipping: $zs")

    //sample.foreach( (k, v) =>
    //  variables.view.filter( parents(_).contains(k) ).foreach( x =>
    //    probIndex.put( x, probIndex(x) + parentImpact((x, k)) * valueIndex((k, v)))  ) )

    for ( i <- 1 to nSamples ) {
      logInc(1, s"Sample: $i")
      n.put(sample(x), n(sample(x)) + 1)
      zs.foreach( z => {
        log(1, s"flipping: $z")
        val mb = children(z) match {
          //case Nil => pCond(z)(probIndex(z)).value
          case Nil => probVector(sample, parents(z), pCond(z))
          case cs => {
            val prods = values(z).map( v => {
              updateSample(sample, z, v)
              cs.foldLeft(Num(1))((c, y) => {
                c * probVector(sample, parents(y), pCond(y))(values(y).indexOf(sample(y)))
                //c *  pCond(y)(probIndex(y)).value(valueIndex((y, sample(y))))
              })
            })
            val p_z = probVector(sample, parents(z), pCond(z))
            //val p_z = pCond(z)(probIndex(z)).value
            val mb = (0 until p_z.length).map( i => p_z(i) * prods(i) )
            val sum = mb.foldLeft(Num(0))( (c, p) => c + p )
            ListTerm(if ( sum == Num(0) ) mb else mb.map( p => p/sum ))
          }
        }
        log(1, s"Markov Bed: $mb")
        val roll = Num(r.nextDouble())
        var sum = Num(0)
        val pick = (0 until mb.length).find( j => {
          sum += mb(j)
          roll < sum
        } ).get
        log(1, s"Roll: $roll $sum $pick")
        updateSample(sample, z, values(z)(pick))
      })
      logDec(1, "Done")
    }
    ListTerm(values(x).map( v => KeyVal(v, n(v) / nSamples )))
  }

  private def probVector( sample: Map[Term,Term], parents: ListTerm, p: CollectionTerm ): ListTerm = {
    p(ListTerm(parents.map(p => sample(p)).toSeq))
    /*parents match
      case None => p(ListTerm.empty)
      case Some(ps) => */
  }
}
