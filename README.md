# esphome-sma-net

External ESPHome component for reading SMA NET data via UART.

## Status

This repository is structured as an ESPHome external component:

- `components/sma_net/__init__.py`
- `components/sma_net/sensor.py`
- `components/sma_net/text_sensor.py`
- `components/sma_net/sma_net.cpp`
- `components/sma_net/sma_net.h`

## Requirements

- ESPHome (recommended: current stable)
- ESP32 board (this repo currently targets ESP32-C3 in examples)
- UART wiring to your SMA NET source

## Install / Use as external component

### Option A: Local (development)

```yaml
external_components:
  - source:
      type: local
      path: ./components
    components: [sma_net]
```

### Option B: From GitHub

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/robert-budde/esphome-sma-net.git
      ref: main
    components: [sma_net]
```

For production, prefer a release tag (e.g. `ref: v0.1.0`) instead of `main`.

## Minimal configuration

See `examples/sma-net-minimal.yaml`.

Core component:

```yaml
sma_net:
  id: sma
  uart_id: uart_sma
  update_interval: 3s
  slow_factor: 20
  debug: false
```

## Sensor platforms

### Numeric sensors

```yaml
sensor:
  - platform: sma_net
    sma_net_id: sma
    channel: Pac
    interval: fast   # fast|slow
    name: "Pac"
```

### Text sensors

```yaml
text_sensor:
  - platform: sma_net
    sma_net_id: sma
    channel: "Mode"
    name: "Mode"
```

## Component options

`sma_net:`

- `id` (required)
- `uart_id` (required)
- `update_interval` (optional, default from polling schema)
- `slow_factor` (optional, default `2`, minimum `2`)
- `debug` (optional, default `false`)

`sensor:` (`platform: sma_net`)

- `sma_net_id` (required)
- `channel` (required)
- `interval` (optional: `slow` or `fast`, default `slow`)

`text_sensor:` (`platform: sma_net`)

- `sma_net_id` (required)
- `channel` (required)

## Testing workflow

Recommended repo layout for testing:

- `examples/sma-net-minimal.yaml` (quick smoke test)
- `examples/sma-net-full.yaml` (larger validation setup)

Use `!secret` values in examples and keep a local `secrets.yaml` outside git-tracked sensitive data.

## Troubleshooting

- Verify UART pins and baud rate first.
- Enable debug logging:

```yaml
logger:
  level: DEBUG
  baud_rate: 0

sma_net:
  debug: true
```

- If entities do not update, reduce config complexity and start with the minimal example.
