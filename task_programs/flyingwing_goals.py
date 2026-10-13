"""Example closed-loop translation, extension, and dwell program.

Targets use metres in the mocap world frame. The formation config selects
this program explicitly; the timed demonstration remains its default.
"""

from swarm.domain.tasks import (
    Delay, Hold, NodePositionGoal, Parallel, ProjectedDistanceGoal, Sequence,
)


ANCHOR_TARGET = (0.0, 0.0)
WING_SPAN_M = 0.6
WING_HEIGHT_M = 0.6


PROGRAM = Sequence((
    Parallel((
        NodePositionGoal(
            id="anchor", node_id="car3", target=ANCHOR_TARGET,
            weight=2.0, hold_after_completion=True,
        ),
        Sequence((
            Delay(seconds=1.0),
            Parallel((
                ProjectedDistanceGoal(
                    id="left_span", source_id="car6", target_id="car1",
                    axis=(1.0, 0.0), target=WING_SPAN_M,
                    hold_after_completion=True,
                ),
                ProjectedDistanceGoal(
                    id="right_span", source_id="car2", target_id="car8",
                    axis=(1.0, 0.0), target=WING_SPAN_M,
                    hold_after_completion=True,
                ),
            )),
        )),
    )),
    Hold(seconds=2.0),
    Parallel((
        ProjectedDistanceGoal(
            id="left_height", source_id="car5", target_id="car6",
            axis=(0.0, 1.0), target=WING_HEIGHT_M,
            hold_after_completion=True,
        ),
        ProjectedDistanceGoal(
            id="right_height", source_id="car7", target_id="car8",
            axis=(0.0, 1.0), target=WING_HEIGHT_M,
            hold_after_completion=True,
        ),
    )),
    Hold(seconds=2.0),
))
