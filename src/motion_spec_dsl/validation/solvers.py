# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Solver and solver-reference validation."""

from __future__ import annotations

from motion_spec_dsl.controller_semantics import controller_command_record
from motion_spec_dsl.classes import (
    ConstraintHandler,
    ControllerAlias,
    ControllerEntry,
    ControllerType,
    ExecutionContext,
    HandlerControlMode,
    Model,
    QuantityType,
    SolverEntry,
    SolverRef,
    SubSpace,
    _resolved_controller,
    _resolved_solver,
)
from motion_spec_dsl.validation.common import constraint_handlers, semantic_error
from motion_spec_dsl.validation.constraints import validate_saturation_spec


SUPPORTED_CONTROL_MODES_BY_SOLVER_ALGORITHM: dict[str, set[HandlerControlMode]] = {
    "ACHD": {HandlerControlMode.JointTorque},
    "RNE": {HandlerControlMode.JointTorque},
    "CommandForwarding": {HandlerControlMode.JointTorque},
}


def _implicit_solver_for_controller(
    handler: ConstraintHandler,
    controller: ControllerEntry | ControllerAlias,
) -> SolverEntry | None:
    """The sole solver matching a controller's command-forwarding vs dynamics role, or None
    if ambiguous.
    """
    solvers = [_resolved_solver(item) for item in handler.solvers]
    resolved_controller = _resolved_controller(controller)
    if resolved_controller.type == ControllerType.FeedForward:
        candidates = [solver for solver in solvers if str(solver.algorithm) == "CommandForwarding"]
    else:
        candidates = [solver for solver in solvers if str(solver.algorithm) != "CommandForwarding"]
    return candidates[0] if len(candidates) == 1 else None


def validate_controller_solver_refs(model: Model) -> None:
    """Raise if a controller omits a solver where the handler has several, or references a
    solver the handler does not assemble.
    """
    for handler in constraint_handlers(model):
        resolved_handler_solvers = [_resolved_solver(solver) for solver in handler.solvers]
        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            explicit_solver = getattr(resolved_controller.solver, "solver", None)
            if explicit_solver is None:
                if _implicit_solver_for_controller(handler, controller) is not None:
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
    """The solver a controller explicitly references, or None."""
    solver_ref = getattr(controller, "solver", None)
    if isinstance(solver_ref, SolverRef):
        return solver_ref.solver
    return None


def handler_controller_solver(
    handler: ConstraintHandler,
    controller: ControllerEntry | ControllerAlias,
) -> SolverEntry | None:
    """The solver that runs `controller`: explicit, the handler's sole solver, or the
    implicit role match.
    """
    solver = controller_solver(controller)
    if solver is not None:
        return solver
    solvers = [_resolved_solver(item) for item in handler.solvers]
    if len(solvers) == 1:
        return solvers[0]
    return _implicit_solver_for_controller(handler, controller)


def _controller_domain(controller: ControllerEntry | ControllerAlias) -> str:
    """A controller's control domain: `force` or `pose`."""
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
    """Raise on unsupported solver algorithms (e.g. standalone RNE off the MuJoCo backend)."""
    for handler in constraint_handlers(model):
        for solver in handler.solvers:
            resolved_solver = _resolved_solver(solver)
            if str(resolved_solver.algorithm) == "CommandForwarding":
                continue
            if str(resolved_solver.algorithm) == "RNE":
                is_mujoco = any(
                    getattr(context.platform, "kind", None) == "simulation"
                    and getattr(context.platform, "name", None) == "MuJoCo"
                    for context in model.specs
                    if isinstance(context, ExecutionContext)
                )
                if not is_mujoco:
                    raise semantic_error(
                        f"Solver '{resolved_solver.name}' in handler '{handler.name}' uses RNE, "
                        "but standalone RNE is only supported on the MuJoCo backend.",
                        solver,
                    )


def validate_solver_limits(model: Model) -> None:
    """Raise on repeated, unsupported, or wrong-typed solver saturation limits."""
    expected_by_target = {
        "torque": QuantityType.Torque,
        "linear-acceleration": QuantityType.LinearAcceleration,
        "angular-acceleration": QuantityType.AngularAcceleration,
    }
    for handler in constraint_handlers(model):
        for solver in handler.solvers:
            resolved_solver = _resolved_solver(solver)
            limits = getattr(resolved_solver, "limits", None)
            if limits is None:
                continue
            seen: set[str] = set()
            for entry in limits.entries:
                target = str(entry.target)
                if target in seen:
                    raise semantic_error(
                        f"Solver '{resolved_solver.name}' repeats saturation target '{target}'.",
                        entry,
                    )
                seen.add(target)
                expected = expected_by_target.get(target)
                if expected is None:
                    raise semantic_error(
                        f"Solver '{resolved_solver.name}' has unsupported saturation target '{target}'.",
                        entry,
                    )
                validate_saturation_spec(
                    entry.saturation,
                    expected=expected,
                    owner=entry,
                    label=f"Solver '{resolved_solver.name}' {target}",
                )


def validate_handler_control_mode_solver_compatibility(model: Model) -> None:
    """Raise if a handler's control mode is unsupported by its solver's algorithm."""
    for handler in constraint_handlers(model):
        for solver in handler.solvers:
            resolved_solver = _resolved_solver(solver)
            supported_modes = SUPPORTED_CONTROL_MODES_BY_SOLVER_ALGORITHM.get(
                str(resolved_solver.algorithm), set()
            )
            if handler.control_mode not in supported_modes:
                raise semantic_error(
                    f"Handler '{handler.name}' control mode {handler.control_mode.value} "
                    f"is not supported by solver '{resolved_solver.name}' with algorithm "
                    f"{resolved_solver.algorithm}.",
                    solver,
                )


def validate_mixed_solver_domains(model: Model) -> None:
    """Raise on incompatible mixing of ACHD and RNE controller domains within one handler."""
    for handler in constraint_handlers(model):
        domains_by_algorithm: dict[str, set[str]] = {}
        for controller in handler.controllers:
            solver = controller_solver(controller)
            if solver is None or str(solver.algorithm) == "CommandForwarding":
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
