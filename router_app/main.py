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
    parser.add_argument("--drc-feedback-pairwise", action="store_true")
    parser.add_argument("--drc-feedback-max-iterations", type=int, default=0)
    parser.add_argument("--drc-feedback-kicad-cli")
    args = parser.parse_args(sys.argv[1:])

    app = QApplication([sys.argv[0]])
    window = MainWindow(
        args.board,
        freerouting_full=args.freerouting_full,
        pcb_router=args.pcb_router,
        router_profile_manifest=args.router_profile_manifest,
        drc_feedback_pairwise=args.drc_feedback_pairwise,
        drc_feedback_max_iterations=args.drc_feedback_max_iterations,
        drc_feedback_kicad_cli=args.drc_feedback_kicad_cli,
    )
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
