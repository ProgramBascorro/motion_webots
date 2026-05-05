"""Abstract base class for all role FSMs."""
from abc import ABC, abstractmethod

from ..intent_types import Intent
from ..world_model import WorldModel


class BaseRole(ABC):
    """Each role FSM holds its own state and produces an Intent each tick.
    No ROS I/O here — pure state machine logic.
    """

    @abstractmethod
    def update(self, world: WorldModel) -> Intent:
        """Advance the FSM one step given the current world snapshot.
        Returns the intent for this tick.
        """

    @property
    @abstractmethod
    def state_name(self) -> str:
        """Human-readable current FSM state for logging/monitoring."""
