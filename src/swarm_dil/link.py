"""Message-delivery model for disconnected, intermittent, limited (DIL) links."""
import numpy as np


class Channel:
    """Independent channel per directed link, advanced once per message slot.

    loss:  long-run fraction of messages dropped.
    burst: mean length of a loss burst in slots. None gives independent
           (Bernoulli) losses; a number gives a Gilbert-Elliott channel whose
           Bad state drops everything and whose Good state delivers everything.
    """

    def __init__(self, shape, loss, burst, rng):
        if not 0.0 <= loss < 1.0:
            raise ValueError(f"loss must be in [0, 1), got {loss}")
        self.shape, self.loss, self.burst, self.rng = shape, loss, burst, rng
        if burst is not None:
            if burst < 1:
                raise ValueError(f"burst must be >= 1 slot, got {burst}")
            self.p_bg = 1.0 / burst
            self.p_gb = loss * self.p_bg / (1.0 - loss)
            if self.p_gb > 1.0:
                raise ValueError(f"loss={loss} unreachable with burst={burst}: need loss <= burst/(burst+1)")
            self.bad = rng.random(shape) < loss  # start in the stationary distribution

    def step(self):
        """Advance one slot; True where the message is delivered."""
        u = self.rng.random(self.shape)
        if self.burst is None:
            return u >= self.loss
        self.bad = np.where(self.bad, u >= self.p_bg, u < self.p_gb)
        return ~self.bad
