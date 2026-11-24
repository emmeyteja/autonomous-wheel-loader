"""Optional device-side overflow union after every physics substep."""

import warp as wp


@wp.kernel
def accumulate_overflow(current: wp.array[int], cumulative: wp.array[int]):
    i = wp.tid()
    cumulative[i] = cumulative[i] | current[i]
