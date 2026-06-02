from __future__ import annotations

import argparse
import sys

from PyQt5.QtWidgets import QApplication

from router_app.gui.main_window import MainWindow


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", nargs="?")
    parser.add_argument("--freerouting-full", action="store_true")
    parser.add_argument("--pcb-router", action="store_true")
    parser.add_argument("--router-profile-manifest")
    args = parser.parse_args(sys.argv[1:])

    app = QApplication([sys.argv[0]])
    window = MainWindow(
        args.board,
        freerouting_full=args.freerouting_full,
        pcb_router=args.pcb_router,
        router_profile_manifest=args.router_profile_manifest,
    )
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
