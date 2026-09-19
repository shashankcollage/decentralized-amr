# Dashboard API

All endpoints are read-only monitoring/control-of-the-experiment; **none
of them influence a robot's own movement decisions**. See
`dashboard/app.py`'s module docstring for the enforced separation.

Base URL when running `python main.py --dashboard`: `http://127.0.0.1:8080`

| Method | Path | Description |
|---|---|---|
| GET | `/` | Live HTML dashboard page (polls the endpoints below every `DASHBOARD_POLL_INTERVAL_MS`). |
| GET | `/api/status` | `{sim_time, tick_count, running, paused, num_robots, active_robots, completed_robots, average_battery, total_distance, total_collisions}` |
| GET | `/api/robots` | `{robot_id: {position, status, battery, distance_travelled, current_task, ...}}` for every robot. |
| GET | `/api/warehouse` | Static layout snapshot (`width`, `height`, `cells`, `pickup_points`, `dropoff_points`, `charging_stations`, `intersections`) plus currently `blocked_aisles`. |
| GET | `/api/tasks` | Full task pool state (if a `TaskManager` is attached to the run). |
| GET | `/api/conflicts` | Robots currently in `NEGOTIATING` status (best-effort snapshot of active conflicts). |
| GET | `/api/deadlocks` | Robots currently inside an enforced deadlock-backoff cooldown, with their consecutive-backoff count. |
| GET | `/api/metrics` | Fleet-wide derived metrics (total distance, completed tasks, collisions, messages sent, average battery). |
| POST | `/api/simulation/pause` | Pauses the simulator's own clock. Verified to freeze `sim_time` exactly (see README section 6) - never touches robot state. |
| POST | `/api/simulation/resume` | Resumes the simulator's clock from where it was paused. |

## Example

```bash
curl http://127.0.0.1:8080/api/status
# {"sim_time": 4.9, "tick_count": 49, "running": true, "paused": false,
#  "num_robots": 3, "active_robots": 2, "completed_robots": 1, ...}

curl -X POST http://127.0.0.1:8080/api/simulation/pause
# {"paused": true}
```
