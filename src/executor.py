import multiprocessing
import os
import sys

import numpy as np
import wx

sys.path.append(os.path.join(os.path.dirname(__file__), "mbase"))

# 指数表記なし、有効小数点桁数6、30を超えると省略あり、一行の文字数200
np.set_printoptions(suppress=True, precision=6, threshold=30, linewidth=200)

# Windowsマルチプロセス対策
multiprocessing.freeze_support()

if __name__ == "__main__":

    from mbase.mlib.pmx.collection import PmxModel

    model = PmxModel()

    # 引数指定がない場合、通常起動
    app = wx.App(False)
    frame = wx.Frame(parent=None, id=-1, title="wxPython", size=(400, 400))
    frame.Show(True)
    app.MainLoop()
