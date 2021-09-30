package org.aiddl.util.function;

import org.aiddl.core.container.Container;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.representation.Term;
import org.aiddl.util.function.math.ExptFunction;
import org.aiddl.util.function.math.SineFunction;
import org.aiddl.util.function.random.NormalDistributionSampler;
import org.aiddl.util.function.random.UniformElementSampler;
import org.aiddl.util.function.random.UniformIntegerSampler;
import org.aiddl.util.function.random.UniformRealSampler;
import org.aiddl.util.request.RequestCaller;
import org.aiddl.util.request.RequestHandler;

public class RegistryLoader {
    private static double log10_2 = Math.log10(2.0);

    public static void register( FunctionRegistry F, Container C ) {
        F.addFunction( Uri.CALL_REQUEST, new RequestCaller(C, new RequestHandler(F)));

        F.addFunction( Uri.MATH_SIN, (x) -> Term.real(Math.sin(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_COS, (x) -> Term.real(Math.cos(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_ASIN, (x) -> Term.real(Math.asin(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_ACOS, (x) -> Term.real(Math.acos(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_TAN, (x) -> Term.real(Math.tan(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_ATAN, (x) -> Term.real(Math.atan(x.getDoubleValue())) );

        F.addFunction( Uri.MATH_SQRT, (x) -> Term.real(Math.sqrt(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_EXPT, new ExptFunction());

        F.addFunction( Uri.MATH_LOG, (x) -> Term.real(Math.log(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_LN, (x) -> Term.real(Math.log10(x.getDoubleValue())) );
        F.addFunction( Uri.MATH_LOG2, (x) -> Term.real(Math.log10(x.getDoubleValue()) / log10_2) );

        F.addFunction( Uri.MATH_ABS, (x) -> x.asNum().isNegative() ? x.asNum().mult(Term.integer(-1)) : x );
        F.addFunction( Uri.MATH_RAD_2_DEG, (x) -> Term.real(x.asNum().getDoubleValue() * (180.0/Math.PI)) );
        F.addFunction( Uri.MATH_DEG_2_RAD, (x) -> Term.real(x.asNum().getDoubleValue() * (Math.PI/180.0)) );

        F.addFunction( Uri.RANDOM, new UniformRealSampler() );
        F.addFunction( Uri.RANDOM_RAND_INT, new UniformIntegerSampler() );
        F.addFunction( Uri.RANDOM_ELEM, new UniformElementSampler() );
        F.addFunction( Uri.RANDOM_NORMAL, new NormalDistributionSampler() );
    }
}
