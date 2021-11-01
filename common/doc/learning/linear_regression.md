# Linear Regression

Linear Regression [1] fits a linear function to a numerical data set.  This
problem can be solved by the Least Mean Squares method. One interesting feature
of this approach is that it can handle more complex functions by applying them
as expansions to the data set before regression.

## Type Definition

A regression problem is a key-value tuple. The key `attributes` contains its
attributes, the key `label` contains the name of the target, and the key `data`
contains a matrix with numerical cells. The constraint imposes that the label is
one of the attributes, the matrix has the correct size, and that all data points
are of the type specified in the corresponing attributes.

    (#type RegressionProblem
      (type.kv-tuple [attributes:^$Attributes
                      label:^term
                      data:(type.matrix cell-type:^numerical)]
                      constraint:(lambda ?X
                                    (let [ ?Atts:(get-key attributes ?X)
                                           ?Label:(get-key label ?X)
                                           ?Data:(get-key data ?X) ]
                                         (and
                                           (exists (?Label _) ?Atts true)
                                           (= (size ?Atts) (size (first ?Data)))
                                           (forall ?Datapoint ?Data
                                             (forall ((?Var ?Domain) ?Val) (zip [?Atts ?Datapoint])
                                               (has-type ?Val ?Domain) )))))))


The full AIDDL type definition can be found [here](../../aiddl/).

## Example Input

The following problem consists of two real values attributes `x` and `y` where
`y` is the label. The `data` matrix consists of tuples of size two. The first element is the value of `y` and the second element is the value of `x`.

    (^RegressionProblem@SL problem
      (
        attributes:[(y ^real) (x ^real)]
        label:y
        data:[
          (-35.18741599864562 -9.306855831832804)
          (-23.013023507219703 -6.262284982640911)
          (-36.97623661135576 -9.754119431990931)
          (-26.200848920504658 -7.06223820206926)
          (-27.29009260520309 -7.321602260127804)
          (-22.520781513737223 -6.127446866247694)
          (-37.01530578144983 -9.743654178730269)
          (-37.88593998612442 -9.974419711817736)
          (-32.87058918952381 -8.724850115912917)
          (-31.147867685485537 -8.296529860498087)
          (-33.50167991161018 -8.870416038130601)
          (-31.206635871554624 -8.299596553462376)
          (-19.74201109788898 -5.44328668459868)
          (-34.21576261094285 -9.042612700027032)
          (-32.35456278629413 -8.586804083413488)
          (-23.289996651357466 -6.310808337019586)
          (-35.055076235614116 -9.26436723750092)
          (-34.79501522398709 -9.207420667090357)
          (-36.4487672134167 -9.623402305980594)
          (-28.565962660040817 -7.653472742671912)
          (-29.19146566877446 -7.7908231690744305)
          (-19.366489148843844 -5.331539126816407)
          (-35.01179981333075 -9.250708682920516)
          (-19.161581633368183 -5.299909466645133)
          (-33.80360769001688 -8.942229244111358)
          (-20.13597277105702 -5.537740024860733)
          (-18.217179328540674 -5.046906575076909)
          (-34.24527746220852 -9.059711064875046)
          (-26.444250143231468 -7.109097564907286)
          (-29.83926324461677 -7.960390312541163)
          (-28.604851743143282 -7.652148179783091)
          (-29.245775144600152 -7.8139307305622365)
          (-29.304176986125423 -7.814966147794719)
          (-31.04983044883943 -8.263910202822805)
          (-32.93455526947976 -8.739817210544503)
          (-27.545282009089416 -7.391053446807946)
          (-23.951439788736895 -6.491953705107582)
          (-24.176171853917364 -6.543250408494831)
          (-34.75299832335447 -9.185240490521304)
          (-33.02154469973546 -8.752721779775113)
          (-20.94044719409686 -5.725245280973864)
          (-28.610968332792112 -7.649357271736967)
          (-22.630709955038856 -6.162317233967295)
          (-37.58838248087462 -9.907182865326424)
          (-34.334289173139666 -9.071981864369057)
          (-28.969152317107838 -7.73259504817624)
          (-24.599420653185682 -6.648534398625862)
          (-32.85931168126686 -8.718426699743443)
          (-31.25864158334901 -8.310019261126875)
          (-32.226989833041024 -8.547092131216457)
        ]
      ))

The aiddl file containing this example can be found
[here](../../test/learning/regression/problem-01.aiddl).

## Implementation Overview

We solve this problem with the Ordinary Least Squares [1] method.  The trait
[Learner](../../scala/src/main/scala/org/aiddl/common/scala/learning/supervised/Learner.scala)
captures some common methods of most machine learning algorithms, and wraps them
up in the `apply` method. We rely on some matrix algorihms available in AIDDL
common to solve the regression problem. 

    class OrdinaryLeastSquaresRegression extends Learner {
        val inv = new LupInverter
        var w: Matrix = _
    
        def fit(x: ListTerm, y: ListTerm): Term = {
            val X = AiddlMatrix(x)
            val Y = AiddlMatrix(y)
     
            w = inv(X.t * X) * X.t * Y
            w.term
        }
    
        def predict(x: ListTerm): ListTerm = {
            val y = AiddlMatrix(x)(w)
            ListTerm((0 until y.m).map( i => y(i) ): _*)
        }
    }

As a result, writing the `fit` comes pretty close to how it would be described
in a textbook. The `predict` method simply multiplies a data set with the weight
vector `w` and packs the result into an AIDDL list.  The full implementation can
be found
[here](../../scala/src/main/scala/org/aiddl/common/scala/learning/supervised/least_squares/OrdinaryLeastSquaresRegression.scala)

## Try It Yourself

- Link to test case
- Commented test case code


The test case containing this code can be found [here](../../scala/src/test/scala/).

# References

[1] Murphy, Kevin P. (2012). "Machine Learning: A Probabilistic Perspective". The MIT Press.

# TODO

- Try it Yourself section
- Link to Linear Algebra article for matrix methods
