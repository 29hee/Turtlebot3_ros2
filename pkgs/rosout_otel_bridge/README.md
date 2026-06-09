# rosout_otel_bridge

Bridges ROS2 node logs (`/rosout`) into the local
[ObservabilityStack](../../observability/README.md) by exporting each record as an
OpenTelemetry log over OTLP/HTTP to the collector, which fans it into VictoriaLogs.

```text
ROS2 nodes ──/rosout──> rosout_otel_bridge ──OTLP/HTTP :4318──> otel-collector ──> VictoriaLogs
```

For the full run/verify/troubleshoot guide see
[observability/docs/ROS2.md](../../observability/docs/ROS2.md).

## Quick start

```bash
pip install opentelemetry-sdk opentelemetry-exporter-otlp-proto-http
colcon build --packages-select rosout_otel_bridge
source install/setup.bash

ros2 run rosout_otel_bridge bridge
# or
ros2 launch rosout_otel_bridge bridge.launch.py \
    otlp_endpoint:=http://<collector-host>:4318 service_name:=ros2
```

Then query from `observability/`:

```bash
./obs/logs.sh '_time:5m service.name:ros2' 20
```

## Notes

- `/rosout` carries only ROS-logging-API output. `print`/stdout, Gazebo internals,
  and launch chatter are not captured.
- The subscriber uses the rcl `/rosout` QoS (reliable, volatile, keep-last 1000). If
  no logs appear, a QoS or `ROS_DOMAIN_ID` mismatch is the first thing to check.
- The pure translation logic lives in `translate.py` and is unit-tested
  (`test/test_translate.py`) without a ROS2 host.

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `otlp_endpoint` | `http://localhost:4318` | Collector OTLP/HTTP base endpoint (the node appends `/v1/logs`) |
| `service_name` | `ros2` | `service.name` attached to every exported log |
| `include_self` | `false` | When `true`, also export the bridge node's own logs |
