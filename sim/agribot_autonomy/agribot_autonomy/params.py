"""Parameter helpers: declare with a documented range so bad values are refused.

A parameter declared with a range is rejected by ROS itself, at launch from a
YAML file (the node fails to start with the reason) and at run time by
`ros2 param set` (the call returns unsuccessful and the old value stays).
"""
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor


def number(node, name, default, low, high, description, step=0.0):
    """Declare a float parameter limited to [low, high]; returns its value."""
    descriptor = ParameterDescriptor(
        description=description, floating_point_range=[FloatingPointRange(
            from_value=float(low), to_value=float(high), step=float(step))])
    return float(node.declare_parameter(name, float(default), descriptor).value)


def integer(node, name, default, low, high, description):
    descriptor = ParameterDescriptor(
        description=description, integer_range=[IntegerRange(from_value=int(low), to_value=int(high), step=1)])
    return int(node.declare_parameter(name, int(default), descriptor).value)


def flag(node, name, default, description):
    return bool(node.declare_parameter(name, bool(default), ParameterDescriptor(description=description)).value)


def text(node, name, default, description):
    return str(node.declare_parameter(name, str(default), ParameterDescriptor(description=description)).value)


def vector(node, name, default, count, low, high, description):
    """Declare a float-array parameter of exactly `count` values within [low, high]."""
    node.declare_parameter(name, [float(v) for v in default], ParameterDescriptor(description=description))
    return checked_vector(node, name, count, low, high)


def checked_vector(node, name, count, low, high):
    values = [float(v) for v in node.get_parameter(name).value]
    if len(values) != count or any(not (low <= v <= high) for v in values):
        raise ValueError(f"parameter '{name}' must have {count} values in [{low}, {high}], got {values}")
    return values
