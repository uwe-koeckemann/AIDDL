import time


class StopWatch:
    m = {}
    start_time = {}

    def start(key):
        if key not in StopWatch.m.keys():
            StopWatch.m[key] = []
        StopWatch.start_time[key] = time.time()

    def stop(key):
        StopWatch.m[key] = time.time() - StopWatch.start_time[key]

    def get_sums_str():
        key_list = []
        for k in StopWatch.m.keys():
            key_list.append(k)
        key_list.sort()
        s = ""
        for k in key_list:
            s += "%s: %f" % (k, sum(StopWatch.m[k]))
        return s
