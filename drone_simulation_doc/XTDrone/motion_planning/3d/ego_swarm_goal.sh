#!/bin/bash
while(true)
do
    python3 ego_swarm_goal.py typhoon_h480 0 9 5 3 &
    python3 ego_swarm_goal.py typhoon_h480 1 9 1.2 3 &
    python3 ego_swarm_goal.py typhoon_h480 2 9 -1.2 3 &
    python3 ego_swarm_goal.py typhoon_h480 3 9 -4 3 &
    python3 ego_swarm_goal.py typhoon_h480 4 9 -4 3 &
    python3 ego_swarm_goal.py typhoon_h480 5 9 -4 3

done