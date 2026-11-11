from __future__ import annotations

from abc import ABC, abstractmethod

from ..models import TrackerFrame


class VRInputProvider(ABC):
    """Hardware-specific boundary. All returned values use SI units."""

    @abstractmethod
    def read(self) -> TrackerFrame:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __enter__(self) -> "VRInputProvider":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
