"""Current Unitree Isaac Lab G1 task catalog and launch metadata.

Catalog source revisions used when this file was written:

* unitreerobotics/xr_teleoperate: ``817fb00``
* unitreerobotics/unitree_sim_isaaclab: ``e30c25b``
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskSpec:
    alias: str
    task_id: str
    end_effector: str
    wholebody: bool = False
    experimental_synergy: bool = False

    @property
    def sim_dds_flag(self) -> str:
        return {
            "dex1": "--enable_dex1_dds",
            "dex3": "--enable_dex3_dds",
            "inspire": "--enable_inspire_dds",
        }[self.end_effector]

    @property
    def teleop_end_effector(self) -> str:
        return "inspire_dfx" if self.end_effector == "inspire" else self.end_effector

    @property
    def teleop_input_mode(self) -> str:
        # Unitree rejects controller mode for Dex3/Inspire.  The ALVR shim uses
        # hand mode and supplies an explicit controller-driven 25-joint synergy.
        return "controller" if self.end_effector == "dex1" else "hand"

    @property
    def support_label(self) -> str:
        return "experimental-synergy" if self.experimental_synergy else "native-controller"

    @property
    def required_sim_assets(self) -> tuple[str, ...]:
        if self.wholebody:
            robot = {
                "dex1": "assets/robots/g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd",
                "dex3": "assets/robots/g1-29dof_wholebody_dex3/g1_29dof_with_dex3_rev_1_0.usd",
                "inspire": "assets/robots/g1-29dof_wholebody_inspire/g1_29dof_with_inspire_rev_1_0.usd",
            }[self.end_effector]
            return (
                robot,
                "assets/objects/small_warehouse/small_warehouse_digital_twin.usd",
                "assets/objects/PackingTable_2/PackingTable.usd",
                "assets/objects/PackingTable/PackingTable.usd",
                "assets/model/policy.onnx",
            )

        robot = {
            "dex1": "assets/robots/g1-29dof-dex1-base-fix-usd/g1_29dof_with_dex1_base_fix1.usd",
            "dex3": "assets/robots/g1-29dof-dex3-base-fix-usd/g1_29dof_with_dex3_base_fix.usd",
            "inspire": "assets/robots/g1-29dof-inspire-base-fix-usd/g1_29dof_with_inspire_rev_1_0.usd",
        }[self.end_effector]
        if self.alias.startswith("pick-cylinder"):
            scene = ("assets/objects/PackingTable/PackingTable.usd",)
        elif self.alias.startswith("drawer"):
            scene = (
                "assets/objects/small_warehouse/small_warehouse_digital_twin.usd",
                "assets/objects/drawers/cabinet_collider.usd",
            )
        else:
            scene = (
                "assets/objects/small_warehouse_digital_twin/small_warehouse_digital_twin.usd",
                "assets/objects/table_with_yellowbox.usd",
            )
        return (robot, *scene)


def _task(
    alias: str,
    task_id: str,
    end_effector: str,
    *,
    wholebody: bool = False,
) -> TaskSpec:
    return TaskSpec(
        alias=alias,
        task_id=task_id,
        end_effector=end_effector,
        wholebody=wholebody,
        experimental_synergy=end_effector in {"dex3", "inspire"},
    )


TASKS: tuple[TaskSpec, ...] = (
    _task("pick-cylinder-dex1", "Isaac-PickPlace-Cylinder-G129-Dex1-Joint", "dex1"),
    _task("pick-cylinder-dex3", "Isaac-PickPlace-Cylinder-G129-Dex3-Joint", "dex3"),
    _task("pick-cylinder-inspire", "Isaac-PickPlace-Cylinder-G129-Inspire-Joint", "inspire"),
    _task("pick-redblock-dex1", "Isaac-PickPlace-RedBlock-G129-Dex1-Joint", "dex1"),
    _task("pick-redblock-dex3", "Isaac-PickPlace-RedBlock-G129-Dex3-Joint", "dex3"),
    _task("pick-redblock-inspire", "Isaac-PickPlace-RedBlock-G129-Inspire-Joint", "inspire"),
    _task("stack-rgy-dex1", "Isaac-Stack-RgyBlock-G129-Dex1-Joint", "dex1"),
    _task("stack-rgy-dex3", "Isaac-Stack-RgyBlock-G129-Dex3-Joint", "dex3"),
    _task("stack-rgy-inspire", "Isaac-Stack-RgyBlock-G129-Inspire-Joint", "inspire"),
    _task("drawer-dex1", "Isaac-Pick-Redblock-Into-Drawer-G129-Dex1-Joint", "dex1"),
    _task("drawer-dex3", "Isaac-Pick-Redblock-Into-Drawer-G129-Dex3-Joint", "dex3"),
    _task(
        "move-cylinder-dex1",
        "Isaac-Move-Cylinder-G129-Dex1-Wholebody",
        "dex1",
        wholebody=True,
    ),
    _task(
        "move-cylinder-dex3",
        "Isaac-Move-Cylinder-G129-Dex3-Wholebody",
        "dex3",
        wholebody=True,
    ),
    _task(
        "move-cylinder-inspire",
        "Isaac-Move-Cylinder-G129-Inspire-Wholebody",
        "inspire",
        wholebody=True,
    ),
)

TASK_BY_NAME = {task.alias: task for task in TASKS}
TASK_BY_NAME.update({task.task_id: task for task in TASKS})


def resolve_task(name: str) -> TaskSpec:
    try:
        return TASK_BY_NAME[name]
    except KeyError as error:
        aliases = ", ".join(task.alias for task in TASKS)
        raise ValueError(f"unknown G1 task {name!r}; choose one of: {aliases}") from error


def task_rows() -> list[tuple[str, str, str, str]]:
    return [
        (
            task.alias,
            "wholebody" if task.wholebody else "fixed",
            task.end_effector,
            task.support_label,
        )
        for task in TASKS
    ]


__all__ = ["TASKS", "TaskSpec", "resolve_task", "task_rows"]
