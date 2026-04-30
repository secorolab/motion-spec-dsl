# SPDX-License-Identifier: MPL-2.0
"""Solver and solver-reference validation."""

from __future__ import annotations

from motion_spec_dsl.domain import (
    ConstraintHandler,
    ControllerAlias,
    ControllerEntry,
    Model,
    QuantityType,
    SolverEntry,
    SolverRef,
    SubSpace,
    _resolved_controller,
    _resolved_solver,
)
from motion_spec_dsl.semantics import controller_command_record
from motion_spec_dsl.validation.common import constraint_handlers, semantic_error
from motion_spec_dsl.validation.robots import robot_component, robot_component_anchor


def _solver_component(solver: SolverEntry):
    if solver.robot.component_name is None:
        return solver.robot.robot_spec.chain
    return robot_component(solver.robot.robot_spec, solver.robot.component_name)


def _solver_component_anchor(solver: SolverEntry, anchor: str) -> str:
    if solver.robot.component_name is None:
        return getattr(solver.robot.robot_spec.chain, anchor, "")
    return robot_component_anchor(
        solver.robot.robot_spec,
        solver.robot.component_name,
        anchor,
    )


def _anchor_matches_solver(anchor, solver: SolverEntry, expected_anchor: str) -> bool:
    return (
        anchor.robot_spec is solver.robot.robot_spec
        and anchor.component_name == solver.robot.component_name
        and anchor.anchor == expected_anchor
    )


def validate_solver_refs(model: Model) -> None:
    for handler in constraint_handlers(model):
        for solver in handler.solvers:
            component = _solver_component(solver)
            if component is None:
                raise semantic_error(
                    f"Solver '{solver.name}' references unknown robot or component '{solver.robot}'.",
                    solver,
                )

            expected_root = _solver_component_anchor(solver, "root")
            if not _anchor_matches_solver(solver.root, solver, "root"):
                expected = (
                    f"{solver.robot}.chain.root"
                    if solver.robot.component_name is None
                    else f"{solver.robot}.root"
                )
                raise semantic_error(
                    f"Solver '{solver.name}' root must reference '{expected}'.",
                    solver,
                )
            if expected_root and expected_root != robot_component_anchor(
                solver.root.robot_spec,
                solver.root.component_name or solver.root.robot_spec.name,
                solver.root.anchor,
            ):
                raise semantic_error(
                    f"Solver '{solver.name}' root does not match robot '{solver.robot}'.",
                    solver,
                )

            requires_end = getattr(component, "end", "") != ""
            if requires_end and solver.end is None:
                raise semantic_error(
                    f"Solver '{solver.name}' for robot '{solver.robot}' requires end.",
                    solver,
                )
            if not requires_end and solver.end is not None:
                raise semantic_error(
                    f"Solver '{solver.name}' for robot '{solver.robot}' must not define end.",
                    solver,
                )
            if solver.end is not None and not _anchor_matches_solver(solver.end, solver, "end"):
                expected = (
                    f"{solver.robot}.chain.end"
                    if solver.robot.component_name is None
                    else f"{solver.robot}.end"
                )
                raise semantic_error(
                    f"Solver '{solver.name}' end must reference '{expected}'.",
                    solver,
                )


def validate_controller_solver_refs(model: Model) -> None:
    for handler in constraint_handlers(model):
        resolved_handler_solvers = [_resolved_solver(solver) for solver in handler.solvers]
        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            explicit_solver = getattr(resolved_controller.solver, "solver", None)
            if explicit_solver is None:
                if len(resolved_handler_solvers) == 1:
                    continue
                raise semantic_error(
                    f"Controller '{controller.name}' must specify solver because handler "
                    f"'{handler.name}' assembles {len(resolved_handler_solvers)} solvers.",
                    controller,
                )
            if all(id(explicit_solver) != id(solver) for solver in resolved_handler_solvers):
                raise semantic_error(
                    f"Controller '{controller.name}' references solver "
                    f"'{explicit_solver.name}', but handler "
                    f"'{handler.name}' does not assemble it.",
                    controller,
                )


def controller_solver(controller: ControllerEntry | ControllerAlias) -> SolverEntry | None:
    solver_ref = getattr(controller, "solver", None)
    if isinstance(solver_ref, SolverRef):
        return solver_ref.solver
    return None


def handler_controller_solver(
    handler: ConstraintHandler,
    controller: ControllerEntry | ControllerAlias,
) -> SolverEntry | None:
    solver = controller_solver(controller)
    if solver is not None:
        return solver
    solvers = [_resolved_solver(item) for item in handler.solvers]
    return solvers[0] if len(solvers) == 1 else None


def _controller_domain(controller: ControllerEntry | ControllerAlias) -> str:
    resolved_controller = _resolved_controller(controller)
    subspace = resolved_controller.params.constraint.constraint.view.subspace
    command = controller_command_record(controller)
    if command.command_type in {QuantityType.Force, QuantityType.Torque} or subspace in {
        SubSpace.Force,
        SubSpace.Torque,
    }:
        return "force"
    return "pose"


def validate_supported_solver_algorithms(model: Model) -> None:
    for handler in constraint_handlers(model):
        for solver in handler.solvers:
            resolved_solver = _resolved_solver(solver)
            if str(resolved_solver.algorithm) == "RNE":
                raise semantic_error(
                    f"Solver '{resolved_solver.name}' in handler '{handler.name}' uses RNE, "
                    "but RNE is not modeled in the DSL generator yet.",
                    solver,
                )


def validate_mixed_solver_domains(model: Model) -> None:
    for handler in constraint_handlers(model):
        domains_by_algorithm: dict[str, set[str]] = {}
        for controller in handler.controllers:
            solver = controller_solver(controller)
            if solver is None:
                continue
            domains_by_algorithm.setdefault(str(solver.algorithm), set()).add(
                _controller_domain(controller)
            )

        if "ACHD" not in domains_by_algorithm or "RNE" not in domains_by_algorithm:
            continue

        overlapping_domains = domains_by_algorithm["ACHD"] & domains_by_algorithm["RNE"]
        if overlapping_domains:
            overlap = ", ".join(sorted(overlapping_domains))
            raise semantic_error(
                f"Handler '{handler.name}' mixes ACHD and RNE on the same domain(s): "
                f"{overlap}. They may only coexist when ACHD and RNE controllers are "
                "split across pose and force domains.",
                handler,
            )
