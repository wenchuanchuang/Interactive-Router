from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication

from router_app.gui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    initial_file = sys.argv[1] if len(sys.argv) > 1 else None
    window = MainWindow(initial_file)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
