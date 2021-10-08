import math
import random as r

from aiddl_core.representation.real import Real

from src.aiddl_util.function import uri
from src.aiddl_util.function.math import ExptFunction
from src.aiddl_util.function.random import UniformRealSampler, UniformIntegerSampler, UniformElementSampler, \
    NormalDistributionSampler
from src.aiddl_util.request.request_caller import CallRequestFunction
from src.aiddl_util.request.request_handler import RequestHandler


def load_functions(fun_reg, container):
    fun_reg.add_function(uri.CALL_REQUEST, CallRequestFunction(container, RequestHandler(container, fun_reg)))

    fun_reg.add_function(uri.MATH_COS, lambda x: Real(math.cos(x.real_value())))
    fun_reg.add_function(uri.MATH_SIN, lambda x: Real(math.sin(x.real_value())))
    fun_reg.add_function(uri.MATH_ACOS, lambda x: Real(math.acos(x.real_value())))
    fun_reg.add_function(uri.MATH_ASIN, lambda x: Real(math.asin(x.real_value())))
    fun_reg.add_function(uri.MATH_TAN, lambda x: Real(math.tan(x.real_value())))
    fun_reg.add_function(uri.MATH_ATAN, lambda x: Real(math.atan(x.real_value())))
    fun_reg.add_function(uri.MATH_SQRT, lambda x: Real(math.sqrt(x.real_value())))
    fun_reg.add_function(uri.MATH_EXPT, ExptFunction())
    fun_reg.add_function(uri.MATH_LOG, lambda x: Real(math.log(x.real_value())))
    fun_reg.add_function(uri.MATH_LN, lambda x: Real(math.log10(x.real_value())))
    fun_reg.add_function(uri.MATH_LOG2, lambda x: Real(math.log2(x.real_value())))
    fun_reg.add_function(uri.RANDOM, lambda x: UniformRealSampler())
    fun_reg.add_function(uri.RANDOM_RAND_INT, lambda x: UniformIntegerSampler())
    fun_reg.add_function(uri.RANDOM_ELEM, lambda x: UniformElementSampler())
    fun_reg.add_function(uri.RANDOM_NORMAL, lambda x: NormalDistributionSampler())

