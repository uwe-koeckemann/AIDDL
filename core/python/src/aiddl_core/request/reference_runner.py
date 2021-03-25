from aiddl_core.representation.reference import Reference
from aiddl_core.tools.logger import Logger


def run_reference_request(rhandler, request, exec_module):
    ref_target = request.get_ref_target()
    if isinstance(ref_target, Reference):
        ref_target = rhandler.C.resolve_reference(ref_target)
    resolved_request = rhandler.C.get_entry(request.get_ref_module(),
                                            ref_target)
    if resolved_request is None:
        if rhandler.verbose:
            Logger.msg(rhandler.name, "Entry not found in working module:"
                       + str(request.get_ref_target))
            Logger.inc_depth()
        entries = rhandler.C.get_matching_entries(None, None, request)
        for e in entries:
            if rhandler.verbose:
                Logger.msg(rhandler.name, "Match: " + str(e.get_value()))
            rhandler.satisfy_request(e.get_value(), exec_module)
        if rhandler.verbose:
            if len(entries) == 0:
                Logger.msg(rhandler.name,
                           "Could not find entries matching: "
                           + str(request))
            Logger.dec_depth()
    else:
        if rhandler.verbose:
            Logger.msg(rhandler.name, "Reference %s (%s) resolved to %s."
                       % (str(request),
                          str(request.get_ref_target().resolve(rhandler.C)),
                          str(resolved_request)))
            Logger.inc_depth()
        rhandler.satisfy_request(resolved_request.get_value(), exec_module)
        if rhandler.verbose:
            Logger.dec_depth()

            
