#!/usr/bin/env python3
"""Small API integration smoke check; uses recorded *current* positions only."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pednav.core import RobotState
from pednav.sophia import NavigationObservation, PlannerAdapter


def main():
    adapter = PlannerAdapter(mode='predictive')
    for timestamp, ped_y in ((0., 1.0), (.2, .92), (.4, .84)):
        observation = NavigationObservation(
            time_s=timestamp,
            robot=RobotState(x=-2, y=0, theta=0),
            pedestrian_ids=('PED1',),
            pedestrian_xy=np.array([[0.0, ped_y]]),
            goal_xy=(2.0, 0.0),
            walkway_halfwidth_m=1.15,
        )
        command = adapter.step(observation)
        left, right = adapter.wheel_velocities(command)
        print(f't={timestamp:.1f} estimated_vy={command.pedestrian_velocity_m_s[0,1]:+.3f} '
              f'predicted_y_at_horizon={command.predicted_xy_at_horizon[0,1]:+.3f} '
              f'wheel_L={left:+.3f} wheel_R={right:+.3f}')


if __name__ == '__main__':
    main()
