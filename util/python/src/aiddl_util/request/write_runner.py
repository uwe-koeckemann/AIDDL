from aiddl_core.container.container import Entry
from aiddl_core.function.uri import TYPE_TERM

from aiddl_core.tools.logger import Logger


from aiddl_util.request.output_ref_conv import out2ref


def run_write(rhandler, request, exec_module):
    value = rhandler.eval.apply(request.get(1).resolve(rhandler.C))
    ref = out2ref(request.get(2).resolve(rhandler.C),
                  rhandler.C,
                  exec_module)
    if rhandler.verbose:
        Logger.msg(rhandler.name, "write %s to %s" % (str(value),
                                                      str(ref)))
    prev_entry = rhandler.C.get_entry(ref.get_ref_target(),
                                      module=ref.get_ref_module())
    if prev_entry is None:
        prev_type = TYPE_TERM
    else:
        prev_type = prev_entry.get_type()

    out_entry = Entry(prev_type, ref.get_ref_target(), value)

    if rhandler.verbose:
        Logger.msg(rhandler.name, "Writing: %s" % (str(out_entry)))
    rhandler.C.set_entry(out_entry, module=ref.get_ref_module())
