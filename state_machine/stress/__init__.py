"""Memory and stress harness for the Phase 0 mission loop.

The target computer (Jetson Nano) has 4 GB of RAM shared between the OS, the
GPU and our code, so every module we add to the 10 Hz loop has to fit in what
is left. This package measures what the loop costs today and how that grows
with longer runs, bigger observations, more cameras and kept history.

    python3 -m state_machine.stress                  # quick suite (~1 min)
    python3 -m state_machine.stress --suite full     # long runs (several min)
    python3 -m state_machine.stress --list           # show every case

Each case runs in its own Python process so peak memory is not polluted by
the cases before it. See state_machine/stress/README.md for how to read the
report and how to add a workload when a new module lands.
"""
