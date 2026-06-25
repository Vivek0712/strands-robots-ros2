# Security Policy

## Reporting a vulnerability

If you discover a security issue in **strands-robots-ros2**, please report it
**privately** — do not open a public issue.

- Preferred: open a [GitHub private security advisory](https://github.com/Vivek0712/strands-robots-ros2/security/advisories/new).
- Or email: **Vivek Raja** <vivekraja98@gmail.com>

Please include reproduction steps and the affected version/commit. You'll get an
acknowledgement as soon as possible, and credit (if you wish) once a fix ships.

## Scope & safety note

This project commands **physical and simulated robots**. Treat any agent-driven
deployment as safety-critical:

- The `RuntimeAdapter` enforces `safety_limits` (max velocity / duration /
  watchdog) at the boundary — keep them set conservatively for real hardware.
- Never expose a `rosbridge` / control endpoint to an untrusted network.
- Validate prompts and model output before driving real hardware.

## Supported versions

Pre-1.0 — only the latest `main` is supported.
