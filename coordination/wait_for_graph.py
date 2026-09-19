"""
coordination/wait_for_graph.py
=================================
Builds a directed "wait-for" graph (R1 -> R2 means "R1 is waiting for
R2") and detects cycles in it, per project spec section 18.

DECENTRALIZATION NOTE: every robot builds this graph independently, from
its own LocalWorldModel - specifically, from each peer's self-reported
`waiting_for` field (broadcast as part of its ROBOT_STATE message; see
communication/message.py and robots/robot_state.py) plus its own current
wait target. There is no central deadlock detector: because every robot
receives (eventually) the same broadcast information, every robot that
has heard from all cycle members independently reconstructs the same
graph and reaches the same conclusion - a form of "gossip consensus"
common in decentralized systems, not a central authority.

NetworkX is used for the actual cycle-finding (as suggested in the
project spec), but the logic is intentionally kept simple enough to
also reason about by hand: a cycle in this directed graph IS a deadlock,
full stop - there's no fuzziness in the definition.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import networkx as nx


def build_wait_for_graph(waiting_for: Dict[str, Optional[str]]) -> "nx.DiGraph":
    """Build a directed graph from a mapping robot_id -> robot_id it is
    currently waiting for (or None/absent if it isn't waiting on anyone).
    """
    graph = nx.DiGraph()
    for robot_id in waiting_for:
        graph.add_node(robot_id)
    for robot_id, target in waiting_for.items():
        if target is not None:
            graph.add_node(target)
            graph.add_edge(robot_id, target)
    return graph


def find_cycles(graph: "nx.DiGraph") -> List[List[str]]:
    """Return all elementary cycles in the wait-for graph. A non-empty
    result means at least one deadlock exists among those robots."""
    return list(nx.simple_cycles(graph))


def cycle_containing(cycles: List[List[str]], robot_id: str) -> Optional[List[str]]:
    """Return the first detected cycle this robot is part of, or None."""
    for cycle in cycles:
        if robot_id in cycle:
            return cycle
    return None
