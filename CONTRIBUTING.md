# Contributing

Contributions that improve documentation clarity, harden safety paths, or fix control bugs are welcome.

## Workflow

1. Fork the repository and create a feature branch from `main`.
2. Keep changes focused — one concern per PR (firmware vs ROS vs docs).
3. Match existing module-header style (abstract docstring / file banner).
4. Open a pull request with:
   - **What** changed
   - **Why** (match condition, hardware constraint, or bug)
   - **How tested** (bench, sim, or on-robot)

## Code guidelines

### ROS nodes (Python)

- Keep a module docstring summarizing topics, rates, and behaviour.
- Prefer explicit named constants over magic numbers.
- On any node that drives hardware, register a shutdown hook that zeros outputs.

### Firmware (Arduino / ESP32)

- Keep the file banner up to date (pins, structs, timing).
- Document USART struct layouts next to the `struct` definition.
- Prefer fixed-rate loops (`vTaskDelayUntil` / timed loops) over busy-waits for sampling.

### Documentation

- Prefer Mermaid diagrams in Markdown so GitHub renders them without binary assets.
- Put deep math and pin tables in `docs/`; keep `README.md` as the abstract + overview.

## Safety

Do not open PRs that remove fail-safe shutdown behaviour, disable PWM clamping, or hard-code Wi-Fi / Bluetooth credentials into tracked source. Use local untracked config or environment variables instead.

## Questions

Open an issue describing the robot (R1/R2), software version, and observed vs expected behaviour.
