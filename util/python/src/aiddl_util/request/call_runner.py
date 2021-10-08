from aiddl_core.tools.logger import Logger
from aiddl_core.container.container import Entry
from aiddl_core.function.uri import TYPE_TERM
from aiddl_core.representation.collection import Collection

from aiddl_util.request.output_ref_conv import out2ref


def run_call(rhandler, request, exec_module):
    name = rhandler.eval.apply(request[1].resolve(rhandler.C))
    input = rhandler.eval.apply(request[2].resolve(rhandler.C))
    output = request[3].resolve(rhandler.C)
    if rhandler.verbose:
        Logger.msg(rhandler.name, "call " + str(name)
                   + " with " + str(input))
        Logger.inc_depth()
    f = rhandler.F.get_function(name)
    if f is None:
        raise ValueError("Function not registered: " + str(name)
                         + " request: " + str(request))

    if rhandler.enforce_type_checking \
       and callable(getattr(f, "get_interface_uri", None)):
        interface_uri = f.get_interface_uri()
        f_type_check = rhandler.F.get_input_checker(interface_uri)
        if not f_type_check.apply(input).bool_value():
            rhandler.eval.set_verbose(2)
            f_type_check.apply(input)
            raise "Input type check failed: " + str(request)
    result = f.apply(input)

    if rhandler.enforce_type_checking \
       and callable(getattr(f, "get_interface_uri", None)):
        interface_uri = f.get_interface_uri()
        f_type_check = rhandler.F.get_ouput_checker(interface_uri)
        if not f_type_check.apply(result).bool_value():
            rhandler.eval.set_verbose(2)
            f_type_check.apply(result)
            raise ValueError("Input type check failed: " + str(request))
    if isinstance(output, Collection):
        for out in output:
            key = out.get_key()
            ref = out2ref(out.get_value(),
                          rhandler.C,
                          exec_module)
            value = result.get(key, None)
            if value is None:
                raise ValueError("Request: " + str(request)
                                 + "returned: " + str(result)
                                 + "missing key: " + str(output)
                                 + "from output specification: " + str(output)
                                 + "Make sure the keys in the output "
                                 + "specification are the same as used in the"
                                 + " result returned by the requested function.")
            prev_entry = rhandler.C.get_entry(ref.get_ref_module(),
                                              ref.get_ref_target())
            if prev_entry is None:
                prev_type = TYPE_TERM
            else:
                prev_type = prev_entry.get_type()

            out_entry = Entry(prev_type,
                              ref.get_ref_target(),
                              value)

            if rhandler.verbose:
                Logger.msg(rhandler.name,
                           "Writing: %s" % (str(out_entry)))
            rhandler.C.set_entry(out_entry, module=ref.get_ref_module())
        if rhandler.verbose:
            Logger.dec_depth()            
    else:
        ref = out2ref(output,
                      rhandler.C,
                      exec_module)
        prev_entry = rhandler.C.get_entry(ref.get_ref_target(),
                                          module=ref.get_ref_module())
        if prev_entry is None:
            prev_type = TYPE_TERM
        else:
            prev_type = prev_entry.get_type()

        out_entry = Entry(prev_type,
                          ref.get_ref_target(),
                          result)
        rhandler.C.set_entry(out_entry, ref.get_ref_module())
        if rhandler.verbose:
            Logger.msg(rhandler.name,
                       "Writing: %s" % (str(out_entry)))
            Logger.dec_depth()
