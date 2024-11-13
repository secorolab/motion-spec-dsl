from textx import metamodel_from_file
import json
import math
from pyld import jsonld
from pprint import pprint


def var_type(v):
    type_map = {
        "Pose": {
            "angular": "Orientation",
            "linear": "Position"
        },
        "VelocityTwist": {
            "angular": "AngularVelocity",
            "linear": "LinearVelocity"
        },
        "AccelerationTwist": {
            "angular": "AngularAcceleration",
            "linear": "LinearAcceleration"
        },
        "Wrench": {
            "angular": "Torque",
            "linear": "Force",
        }
    }

    if v.entity.type in ["Pose", "VelocityTwist", "AccelerationTwist", "Wrench" ]:
        return type_map[v.entity.type][v.subspace]

    return v.entity.type

def subspace(v):
    type_map = {
        "Pose": {
            "angular": "orientation",
            "linear": "position"
        },
        "VelocityTwist": {
            "angular": "angular-velocity",
            "linear": "linear-velocity"
        },
        "AccelerationTwist": {
            "angular": "angular-acceleration",
            "linear": "linear-acceleration"
        },
        "Wrench": {
            "angular": "torque",
            "linear": "force",
        }
    }

    return type_map[v.entity.type][v.subspace]

def parse_quantity(q):
    if q.unit == "deg/s":
        return math.radians(q.value), "RAD-PER-S"
    elif q.unit == "cm/s":
        return q.value / 100.0, "M-PER-S"
    elif q.unit == "deg":
        return math.radians(q.value), "RAD"
    elif q.unit == "cm":
        return q.value / 100.0, "M"

    unit_map = {
        "deg/s": "DEG-PER-S",
        "rad/s": "RAD-PER-S",
        "m/s": "M-PER-S",
        "cm/s": "CentiM-PER-SEC",
        "deg": "DEG",
        "rad": "RAD",
        "m": "M",
        "cm": "CentiM",
        "N": "N",
        "Nm": "N-M"
    }

    return q.value, unit_map[q.unit]

def parse_context(c):
    quantity = {}
    quantity["@id"] = c.variable.variable.name
    quantity["@type"] = [ "QuantityKind", "Quantity", c.variable.variable.type ]
    quantity["quantity-kind"] = c.variable.variable.type

    value, unit = parse_quantity(c.variable.variable.default)
    quantity["value"] = value
    quantity["unit"] = unit

    return quantity

def parse_view(v, scope):
    view = {}

    if v.subspace and v.axis:
        view["@id"] = scope + "/view-" + v.entity.name + "-" + v.subspace + "-" + v.axis
        view["@type"] = [ "View", v.entity.type + "CoordinateView" ]
        view["into"] = v.entity.name
        view["subspace"] = subspace(v)
        view["axis"] = v.axis
    elif v.axis:
        view["@id"] = scope + "/view-" + v.entity.name + "-" + v.axis
        view["@type"] = [ "View", v.entity.type + "CoordinateView" ]
        view["into"] = v.entity.name
        view["axis"] = v.axis
    else:
        view["@id"] = v.entity.name

    return view

def parse_constraint(c, scope):
    id_ = scope + "/" + c.name

    cstr = {}
    cstr["@id"] = id_
    cstr["@type"] = [ "Constraint" ]
    cstr["quantity"] = parse_view(c.var, id_)

    cls = c.expr.__class__.__name__
    if cls in ["GreaterThanConstraint", "LessThanConstraint"]:
        cstr["@type"].extend(["UnilateralConstraint", cls, var_type(c.var) + "Constraint"])
        cstr["threshold"] = parse_context(c.expr.threshold)
    elif cls == "BilateralConstraint":
        cstr["@type"].append(cls)
        cstr["lower-threshold"] = parse_context(c.expr.lower)
        cstr["upper-threshold"] = parse_context(c.expr.upper)
    elif cls == "EqualityConstraint":
        cstr["@type"].append(cls)
        cstr["reference-value"] = parse_context(c.expr.reference)

    return cstr

def parse_spec(model):
    id_ = model.name

    spec = {}
    spec["@id"] = id_
    spec["@type"] = [ "GuardedMotion" ]
    spec["when"] = [parse_constraint(cstr, id_) for cstr in model.when]
    spec["while"] = [parse_constraint(cstr, id_) for cstr in model.while_]
    spec["until"] = [parse_constraint(cstr, id_) for cstr in model.until]

    return spec

def load(file):
    data = ""
    with open(file) as f:
        data = f.read()
    return json.loads(data)

if __name__ == "__main__":
    meta = metamodel_from_file("comp-rob2b/metamodels/grammars/rob_mot.tx")
    model = meta.model_from_file("comp-rob2b/models/dsl/demo.rob_mot")
    #model = meta.model_from_file("comp-rob2b/models/dsl/test.rob_mot")

    spec = parse_spec(model)

    ctx = [
        load("comp-rob2b/metamodels/task/motion-specification.json")["@context"],
        load("comp-rob2b/metamodels/qudt.json")["@context"]
    ]
    mod = {
        "@context": ctx,
        "@graph": spec
    }

    options = {
        "base": "https://comp-rob2b.github.io/models/motion-specification/"
    }

    pprint(jsonld.flatten(mod, ctx=ctx, options=options))
