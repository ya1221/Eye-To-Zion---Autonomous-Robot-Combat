# tactical_brain

## Purpose
Act as the central decision-making engine for the autonomous robot.

## Logic
Uses Behavior Trees (py_trees) to evaluate the current state (vision, health, location) and decides whether to attack, flee, or search.

## Data Flow
- **Input:** Vision detections, Health stats, Navigation status.
- **Output:** Action goals to Nav2, `/shooting_cmd`, or direct `/cmd_vel` overrides.
