import argparse
import os
from multiprocessing import freeze_support

import numpy as np
import wx

from mlib.core.logger import LoggingMode, MLogger

APP_NAME = "VmdSizing"
VERSION_NAME = "6.00.00_β18"

# 指数表記なし、有効小数点桁数6、30を超えると省略あり、一行の文字数200
np.set_printoptions(suppress=True, precision=6, threshold=30, linewidth=200)

if __name__ == "__main__":
    try:
        # Windowsマルチプロセス対策
        freeze_support()
    except Exception:
        pass

    # 引数の取得
    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", default=20, type=int)
    parser.add_argument("--log_mode", default=0, type=int)
    parser.add_argument("--out_log", default=0, type=int)
    parser.add_argument("--is_saving", default=1, type=int)
    parser.add_argument("--lang", default="ja", type=str)

    args, argv = parser.parse_known_args()

    # ロガーの初期化
    MLogger.initialize(
        lang=args.lang,
        root_dir=os.path.dirname(os.path.abspath(__file__)),
        version_name=f"{APP_NAME} {VERSION_NAME}",
        mode=LoggingMode(args.log_mode),
        level=args.verbose,
        is_out_log=args.out_log,
    )

    from service.form.main_frame import MainFrame

    from mlib.utils.file_utils import get_path

    # アプリの起動
    app = wx.App(False)
    icon = wx.Icon(get_path("resources/logo.ico"), wx.BITMAP_TYPE_ICO)
    frame = MainFrame(
        app, f"{APP_NAME} {VERSION_NAME}", wx.Size(900, 800), args.is_saving
    )
    frame.SetIcon(icon)
    frame.Show(True)
    app.MainLoop()
