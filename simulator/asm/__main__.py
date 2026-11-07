"""``python -m asm file.pio`` -- the same CLI as ``python -m asm.pioasm file.pio``."""

import sys

from .pioasm import main

if __name__ == "__main__":
    sys.exit(main())
