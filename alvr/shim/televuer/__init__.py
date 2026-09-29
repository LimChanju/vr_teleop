"""Select the ALVR-backed TeleVuer compatibility adapter.

The task launcher prepends this ``shim`` directory to ``PYTHONPATH`` before it
starts Unitree's unmodified ``teleop_hand_and_arm.py``.  Its regular
``from televuer import TeleVuerWrapper`` import therefore resolves here.
"""

from alvr_teleop_adapter import TeleData, TeleVuerWrapper

__all__ = ["TeleData", "TeleVuerWrapper"]
