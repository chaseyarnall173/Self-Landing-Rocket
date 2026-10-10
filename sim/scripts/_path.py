"""Make ``import rocketsim`` work when running scripts from any directory."""
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parent.parent
ROOT = SIM.parent
IMG = ROOT / "docs" / "img"
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))
