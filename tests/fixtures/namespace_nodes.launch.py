"""Multiple nodes in namespaces for testing."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Node without namespace
    l.node("root_package", "root_node")

    # Node in namespace
    with l.namespace("ns1"):
        l.node("ns1_package", "ns1_node", name="node_in_ns1")

    # Node in different namespace
    with l.namespace("ns2"):
        l.node("ns2_package", "ns2_node", name="node_in_ns2")

    return l
