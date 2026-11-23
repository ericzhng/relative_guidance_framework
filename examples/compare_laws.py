"""Master runner: executes both missile homing and orbital rendezvous comparisons."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from examples import compare_missile_laws, compare_rendezvous_laws


def main() -> None:
    """Run all application benchmarks."""
    compare_missile_laws.main()
    print("\n" + "=" * 60 + "\n")
    compare_rendezvous_laws.main()


if __name__ == "__main__":
    main()
