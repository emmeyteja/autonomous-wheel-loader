"""Bounded trial range noise with independent per-environment reset state."""

import numpy as np


class RangeNoise:
    def __init__(self, count, seed, std_m, dropout, min_m, max_m):
        if count < 1 or std_m < 0 or not 0 <= dropout < 1 or not 0 < min_m < max_m:
            raise ValueError("Invalid range-noise settings")
        self.seed, self.std, self.dropout, self.lo, self.hi = (
            seed,
            std_m,
            dropout,
            min_m,
            max_m,
        )
        self.generators = [
            np.random.default_rng(np.random.SeedSequence([seed, i]))
            for i in range(count)
        ]

    def reset(self, mask):
        mask = np.asarray(mask, bool)
        if mask.shape != (len(self.generators),):
            raise ValueError("Reset mask shape differs")
        for i in np.flatnonzero(mask):
            self.generators[i] = np.random.default_rng(
                np.random.SeedSequence([self.seed, int(i)])
            )

    def apply(self, ranges_m):
        values = np.asarray(ranges_m, float)
        if values.ndim != 2 or len(values) != len(self.generators):
            raise ValueError("Invalid range batch")
        result = values.copy()
        for i, g in enumerate(self.generators):
            result[i] += g.normal(0, self.std, values.shape[1])
            keep = g.random(values.shape[1]) >= self.dropout
            valid = (
                np.isfinite(values[i])
                & keep
                & (result[i] >= self.lo)
                & (result[i] <= self.hi)
            )
            result[i, ~valid] = np.nan
        return result, np.isfinite(result)
