# Collecting ROS2 logs into the stack

This stack is wired to receive ROS2 node logs through the `rosout_otel_bridge`
package (`pkgs/rosout_otel_bridge`). The bridge subscribes to `/rosout` and exports
each record as an OpenTelemetry log to the collector, which fans it into VictoriaLogs.

```text
ROS2 nodes ──/rosout──> rosout_otel_bridge ──OTLP/HTTP :4318──> otel-collector ──> VictoriaLogs
```

> [!IMPORTANT]
> `/rosout` carries only logs emitted through the ROS logging API (`get_logger()`,
> `RCLCPP_INFO`, `self.get_logger().info(...)`, …). Raw `print`/stdout, Gazebo
> internals, and launch-system chatter do NOT flow through `/rosout` and are not
> captured by this bridge.

## Prerequisites

- The stack runs on a host with Docker (your dev/remote PC, not the Raspberry Pi).
- The bridge runs on a host with ROS2 Humble and these Python packages:

    ```bash
    pip install opentelemetry-sdk opentelemetry-exporter-otlp-proto-http
    ```

- The bridge and the ROS2 nodes share the same `ROS_DOMAIN_ID` so the bridge can
  see `/rosout`. The bridge may run on the remote PC and observe the Burger's nodes
  over the DDS network.

## Run it

```bash
# 1. start the shared infra (collector + 3 stores)
cd observability && make up

# 2. confirm the collector and stores are healthy
docker compose ps

# 3. build and source the bridge package (from the colcon workspace)
colcon build --packages-select rosout_otel_bridge
source install/setup.bash

# 4. run the bridge (point it at the collector host if not localhost)
ros2 run rosout_otel_bridge bridge
#   or, with overrides:
ros2 launch rosout_otel_bridge bridge.launch.py \
    otlp_endpoint:=http://<collector-host>:4318 service_name:=ros2
```

## Verify end to end

```bash
# generate a ROS log from any node, e.g. a quick talker, then query:
cd observability
./obs/logs.sh '_time:5m service.name:ros2' 20
./obs/logs.sh '_time:15m service.name:ros2 severity_text:error' 50
./obs/logs.sh '_time:15m service.name:ros2 ros.node:rosout_otel_bridge' 20
```

If you see records, the pipeline works. Each record carries `severity_text`
(`info`/`warn`/`error`), `ros.node`, and source location (`code.filepath`,
`code.function`, `code.lineno`).

## Troubleshooting

| Symptom | First thing to check |
| --- | --- |
| No logs appear at all | QoS mismatch on `/rosout`. The bridge uses the rcl rosout QoS (reliable, volatile, keep-last 1000). Confirm `ros2 topic echo /rosout` shows traffic and that `ROS_DOMAIN_ID` matches. |
| Bridge starts but exports nothing | Wrong `otlp_endpoint`. It must be the collector's `:4318` reachable from the bridge host; the bridge appends `/v1/logs`. |
| Connection refused on export | The stack is not up, or Docker daemon is down. Run `make up` and `docker compose ps`. |
| Logs land but with no severity filter working | Filter on `severity_text`, not `level`. |

## Configuration

| Parameter / env | Default | Purpose |
| --- | --- | --- |
| `otlp_endpoint` / `OTEL_EXPORTER_OTLP_ENDPOINT` | `http://localhost:4318` | Collector OTLP/HTTP base endpoint |
| `service_name` / `OTEL_SERVICE_NAME` | `ros2` | `service.name` attached to every exported log |
| `include_self` | `false` | When `true`, also export the bridge node's own logs |
