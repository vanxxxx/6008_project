from pathlib import Path
import sys


PYTHON_SOURCE = Path(__file__).parents[1] / "app" / "src" / "main" / "python"
sys.path.insert(0, str(PYTHON_SOURCE))
