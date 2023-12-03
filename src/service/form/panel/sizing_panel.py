import os
import webbrowser

import wx
from service.form.widgets.sizing_bone_set import SizingBoneSet

from mlib.core.logger import ConsoleHandler, MLogger
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.widgets.console_ctrl import ConsoleCtrl
from mlib.service.form.widgets.exec_btn_ctrl import ExecButton
from mlib.service.form.widgets.image_btn_ctrl import ImageButton
from mlib.utils.file_utils import save_histories

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class SizingPanel(NotebookPanel):
    def __init__(self, frame: NotebookFrame, tab_idx: int, *args, **kw) -> None:
        super().__init__(frame, tab_idx, *args, **kw)
        self.sizing_sets: list[SizingBoneSet] = []

        self.sizing_worker = None

        self._initialize_ui()

        self.on_add_set(wx.EVT_BUTTON)

        self.EnableExec(False)

    def _initialize_ui(self) -> None:
        # ヘッダー -----------------------------
        self.description_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.description_ctrl = wx.StaticText(
            self,
            wx.ID_ANY,
            __("モーション作成元モデルにモーションを読み込んだ時の動きを、任意のモデルで再現できるようモーションを調整します\n"),
        )
        self.description_sizer.Add(self.description_ctrl, 0, wx.ALL, 2)
        self.root_sizer.Add(self.description_sizer, 0, wx.ALL, 3)

        self.header_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.add_set_btn_ctrl = wx.Button(
            self,
            wx.ID_ANY,
            __("サイジングセット追加"),
            wx.DefaultPosition,
            wx.Size(120, -1),
        )
        self.add_set_btn_ctrl.SetToolTip(__("サイジングセットを追加できます"))
        self.add_set_btn_ctrl.Bind(wx.EVT_BUTTON, self.on_add_set)
        self.header_sizer.Add(self.add_set_btn_ctrl, 0, wx.ALL, 3)

        self.clear_set_btn_ctrl = wx.Button(
            self,
            wx.ID_ANY,
            __("サイジングセット全削除"),
            wx.DefaultPosition,
            wx.Size(120, -1),
        )
        self.clear_set_btn_ctrl.SetToolTip(__("全てのサイジングセットを削除できます"))
        self.clear_set_btn_ctrl.Bind(wx.EVT_BUTTON, self.on_clear_set)
        self.header_sizer.Add(self.clear_set_btn_ctrl, 0, wx.ALL, 3)

        self.root_sizer.Add(self.header_sizer, 0, wx.ALL | wx.ALIGN_RIGHT, 3)

        self.box_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # 設定 ------------------------------

        self.config_box = wx.StaticBox(self, wx.ID_ANY, __("追加補正"))
        self.config_sizer = wx.StaticBoxSizer(self.config_box, orient=wx.VERTICAL)

        self.is_full_config = False
        self.full_config_btn_ctrl = wx.Button(
            self, wx.ID_ANY, __("全追加補正ON"), wx.DefaultPosition, wx.Size(120, -1)
        )
        self.full_config_btn_ctrl.SetToolTip(__("全ての追加補正を有効にします"))
        self.full_config_btn_ctrl.Bind(wx.EVT_BUTTON, self.on_full_config)
        self.config_sizer.Add(self.full_config_btn_ctrl, 0, wx.ALL | wx.ALIGN_RIGHT, 3)

        # 全親統合 ------------------------
        self.integrate_parent_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.integrate_parent_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("全親統合"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.integrate_parent_check_ctrl.SetToolTip(
            __("全ての親の移動や回転を子ボーンに振り分けて、全ての親のキーフレームを削除します")
        )
        self.integrate_parent_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_add_config)
        self.integrate_parent_sizer.Add(self.integrate_parent_check_ctrl, 0, wx.ALL, 3)
        self.integrate_parent_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(event, "https://bowlroll.net/file/index"),
            __("詳しい解説ページをデフォルトブラウザで開きます"),
        )
        self.integrate_parent_sizer.Add(self.integrate_parent_help_ctrl, 0, wx.ALL, 0)
        self.config_sizer.Add(self.integrate_parent_sizer, 0, wx.ALL, 1)

        # 捩り分散 ------------------------
        self.twist_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.twist_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("捩り分散"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.twist_check_ctrl.SetToolTip(__("腕を腕捩りなど、捩りボーンに捩り回転を分散させます"))
        self.twist_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_add_config)
        self.twist_sizer.Add(self.twist_check_ctrl, 0, wx.ALL, 3)
        self.twist_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(event, "https://bowlroll.net/file/index"),
            __("詳しい解説ページをデフォルトブラウザで開きます"),
        )
        self.twist_sizer.Add(self.twist_help_ctrl, 0, wx.ALL, 0)
        self.config_sizer.Add(self.twist_sizer, 0, wx.ALL, 1)

        # 位置合わせ ------------------------
        self.align_group_sizer = wx.BoxSizer(wx.VERTICAL)

        self.align_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.align_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("手首位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.align_check_ctrl.SetToolTip(__("手首の位置を元モーションと大体同じ位置になるよう合わせます"))
        self.align_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_align_ctrl)
        self.align_sizer.Add(self.align_check_ctrl, 0, wx.ALL, 3)

        self.align_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(event, "https://bowlroll.net/file/index"),
            __("詳しい解説ページをデフォルトブラウザで開きます"),
        )
        self.align_sizer.Add(self.align_help_ctrl, 0, wx.ALL, 0)
        self.align_group_sizer.Add(self.align_sizer, 0, wx.ALL, 0)

        self.align_finger_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.align_finger_blank = wx.StaticText(self, wx.ID_ANY, "     ")
        self.align_finger_sizer.Add(self.align_finger_blank)

        self.align_finger_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("指位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.align_finger_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_align_sub_ctrl)
        self.align_finger_check_ctrl.SetToolTip(
            __("鎖骨あたりに対する指の位置を元モーションと大体同じ位置になるよう合わせます")
        )

        self.align_finger_sizer.Add(self.align_finger_check_ctrl, 0, wx.ALL, 0)
        self.align_group_sizer.Add(self.align_finger_sizer, 0, wx.ALL, 3)

        self.align_finger_tail_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.align_finger_tail_blank = wx.StaticText(self, wx.ID_ANY, "     ")
        self.align_finger_tail_sizer.Add(self.align_finger_tail_blank)

        self.align_finger_tail_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("指先位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.align_finger_tail_check_ctrl.Bind(
            wx.EVT_CHECKBOX, self.on_check_align_sub_ctrl
        )
        self.align_finger_tail_check_ctrl.SetToolTip(
            __("手のひらに対する指先の位置を元モーションと大体同じ位置になるよう合わせます")
        )
        self.align_finger_tail_sizer.Add(
            self.align_finger_tail_check_ctrl, 0, wx.ALL, 0
        )
        self.align_group_sizer.Add(self.align_finger_tail_sizer, 0, wx.ALL, 3)

        self.config_sizer.Add(self.align_group_sizer, 0, wx.ALL, 1)

        # # スタンス追加補正 ----------------------
        # self.stance_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # self.stance_correct_check_ctrl = wx.CheckBox(self.window, wx.ID_ANY, __("スタンス追加補正"), wx.DefaultPosition, wx.DefaultSize, 0)
        # self.stance_correct_check_ctrl.SetToolTip(__("元モーションの動きにより近くなるよう追加で補正を行います\n補正内容は任意で選択できます"))
        # self.stance_correct_check_ctrl.SetBackgroundColour(self.background_color)
        # self.stance_correct_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_stance_correct)
        # self.stance_sizer.Add(self.stance_correct_check_ctrl, 0, wx.ALL, 2)

        # self.stance_correct_btn_ctrl = wx.Button(self.window, wx.ID_ANY, __("補正内訳"), size=wx.Size(120, 20))
        # self.stance_correct_btn_ctrl.Bind(wx.EVT_BUTTON, self.on_select_stance_correct)
        # self.stance_correct_btn_ctrl.SetToolTip(__("スタンス追加補正で行う補正を取捨選択することができます"))
        # self.stance_sizer.Add(self.stance_correct_btn_ctrl, 0, wx.ALL, 2)

        # self.separate1 = wx.StaticText(self.window, wx.ID_ANY, "     |     ")
        # self.separate1.SetBackgroundColour(self.background_color)
        # self.stance_sizer.Add(self.separate1)

        # # 捩り分散
        # self.twist_check_ctrl = wx.CheckBox(self.window, wx.ID_ANY, __("捩り分散"), wx.DefaultPosition, wx.DefaultSize, 0)
        # self.twist_check_ctrl.SetToolTip(__("腕を腕捩りなど、捩りボーンに捩り回転を分散させます"))
        # self.twist_check_ctrl.SetBackgroundColour(self.background_color)
        # self.stance_sizer.Add(self.twist_check_ctrl, 0, wx.ALL, 2)

        # self.config_sizer.Add(self.stance_sizer, 0, wx.ALL, 2)

        # self.config_sizer.Add(wx.StaticLine(self.window, wx.ID_ANY), 1, wx.EXPAND | wx.ALL, 10)

        # self.config_sizer.Add(self.align_sizer, 0, wx.ALL, 2)

        # self.config_sizer.Add(wx.StaticLine(self.window, wx.ID_ANY), 1, wx.EXPAND | wx.ALL, 10)

        # # 足IKオフセット ------------------------
        # self.offset_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # self.leg_ik_offset_label = wx.StaticText(self.window, wx.ID_ANY, __("足IKオフセット"))
        # self.leg_ik_offset_label.SetBackgroundColour(self.background_color)
        # self.offset_sizer.Add(self.leg_ik_offset_label, 0, wx.ALL, 2)

        # self.leg_ik_offset_slider = FloatSliderCtrl(
        #     parent=self.window,
        #     value=0,
        #     min_value=-5,
        #     max_value=5,
        #     increment=0.01,
        #     spin_increment=0.01,
        #     border=3,
        #     size=wx.Size(270, -1),
        #     change_event=None,
        #     tooltip=__("モーションで足が重なってしまう場合などで、足を少し開き気味にするなどのオフセットを追加できます"),
        # )
        # self.leg_ik_offset_slider._slider.SetBackgroundColour(self.background_color)
        # self.offset_sizer.Add(self.leg_ik_offset_slider.sizer, 0, wx.ALL, 2)

        # self.config_sizer.Add(self.offset_sizer, 0, wx.ALL, 2)

        self.box_sizer.Add(self.config_sizer, 4, wx.ALL, 1)

        # ウィンドウ ------------------------------
        self.file_window = wx.ScrolledWindow(
            self,
            wx.ID_ANY,
            wx.DefaultPosition,
            wx.Size(550, 600),
            wx.FULL_REPAINT_ON_RESIZE | wx.VSCROLL | wx.HSCROLL,
        )
        self.file_window.SetScrollRate(5, 5)

        self.window_sizer = wx.BoxSizer(wx.VERTICAL)
        self.set_sizer = wx.BoxSizer(wx.VERTICAL)

        self.window_sizer.Add(self.set_sizer, 0, wx.ALL | wx.EXPAND, 3)
        self.file_window.SetSizer(self.window_sizer)
        self.box_sizer.Add(self.file_window, 6, wx.ALL, 1)

        self.root_sizer.Add(self.box_sizer, 1, wx.GROW, 0)

        # ボタン -------------------------
        self.btn_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.exec_btn_ctrl = ExecButton(
            self,
            self,
            __("サイジング実行"),
            __("サイジング実行停止"),
            self.exec,
            250,
            __("サイジングを実行します\nサイジングセットを1件以上設定後、クリックできるようになります"),
        )
        self.exec_btn_ctrl.exec_worker = self.sizing_worker
        self.btn_sizer.Add(self.exec_btn_ctrl, 0, wx.ALL, 3)

        self.root_sizer.Add(self.btn_sizer, 0, wx.ALIGN_CENTER | wx.SHAPED, 3)

        # コンソール -----------------
        self.console_ctrl = ConsoleCtrl(self, self.frame, self, rows=150)
        self.console_ctrl.set_parent_sizer(self.root_sizer)

    def on_help(self, event: wx.Event, url: str) -> None:
        webbrowser.open(url)

    def on_full_config(self, event: wx.Event) -> None:
        self.is_full_config = not self.is_full_config

        self.full_config_btn_ctrl.SetLabelText(
            __("全追加補正OFF") if self.is_full_config else __("全追加補正ON")
        )
        self.full_config_btn_ctrl.SetToolTip(
            __("全ての追加補正を無効にします") if self.is_full_config else __("全ての追加補正を有効にします")
        )
        self.full_config_btn_ctrl.SetBackgroundColour(
            self.active_background_color
            if self.is_full_config
            else wx.SystemSettings.GetColour(wx.SYS_COLOUR_BTNFACE)
        )

        self.integrate_parent_check_ctrl.SetValue(self.is_full_config)
        self.twist_check_ctrl.SetValue(self.is_full_config)
        self.align_check_ctrl.SetValue(self.is_full_config)
        self.align_finger_check_ctrl.SetValue(self.is_full_config)
        self.align_finger_tail_check_ctrl.SetValue(self.is_full_config)

        self.on_change_dest_model_pmx(event)

    def on_check_add_config(self, event: wx.Event) -> None:
        for sizing_set in self.sizing_sets:
            sizing_set.create_output_path()

    def on_check_align_ctrl(self, event: wx.Event) -> None:
        if not self.align_check_ctrl.GetValue():
            self.align_finger_check_ctrl.SetValue(0)
            self.align_finger_tail_check_ctrl.SetValue(0)
        self.on_change_dest_model_pmx(event)

    def on_check_align_sub_ctrl(self, event: wx.Event) -> None:
        if (
            self.align_finger_check_ctrl.GetValue()
            or self.align_finger_tail_check_ctrl.GetValue()
        ):
            self.align_check_ctrl.SetValue(1)
        self.on_change_dest_model_pmx(event)

    def on_change_dest_model_pmx(self, event: wx.Event) -> None:
        for sizing_set in self.sizing_sets:
            sizing_set.on_change_dest_model_pmx(event)

    def on_add_set(self, event: wx.Event) -> None:
        sizing_idx = len(self.sizing_sets)
        sizing_set = SizingBoneSet(self.file_window, self.frame, self, sizing_idx)

        self.sizing_sets.append(sizing_set)
        self.set_sizer.Add(sizing_set.sizer, 1, wx.GROW, 0)

        self.Enable(True)
        self.fit_window()

    def on_clear_set(self, event: wx.Event) -> None:
        self.window_sizer.Hide(self.set_sizer, recursive=True)
        del self.sizing_sets
        self.sizing_sets = []

    def fit_window(self) -> None:
        self.file_window.Layout()
        self.file_window.Fit()
        self.Layout()

    def save_histories(self) -> None:
        for sizing_set in self.sizing_sets:
            sizing_set.motion_ctrl.save_path()
            sizing_set.src_model_ctrl.save_path()
            sizing_set.dest_model_ctrl.save_path()

        save_histories(self.frame.histories)

    def exec(self, event: wx.Event) -> None:
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.frame.running_worker = True
        self.save_histories()

        self.Enable(False)
        self.exec_btn_ctrl.Enable(True)
        self.sizing_worker.start()

    def on_exec_result(self, result: bool, data: list, elapsed_time: str):
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.console_ctrl.write(f"\n----------------\n{elapsed_time}")

        self.Enable(True)
        self.frame.running_worker = False
        self.frame.on_sound()

        logger.info("サイジング完了", decoration=MLogger.Decoration.BOX)

    def Enable(self, enable: bool) -> None:
        for sizing_set in self.sizing_sets:
            sizing_set.Enable(enable)
        self.add_set_btn_ctrl.Enable(enable)
        self.clear_set_btn_ctrl.Enable(enable)

        self.full_config_btn_ctrl.Enable(enable)
        self.integrate_parent_check_ctrl.Enable(enable)
        self.integrate_parent_help_ctrl.Enable(enable)
        self.twist_check_ctrl.Enable(enable)
        self.twist_help_ctrl.Enable(enable)
        self.align_check_ctrl.Enable(enable)
        self.align_finger_check_ctrl.Enable(enable)
        self.align_finger_tail_check_ctrl.Enable(enable)
        self.align_help_ctrl.Enable(enable)

        self.EnableExec(enable)

    def EnableExec(self, enable: bool) -> None:
        self.exec_btn_ctrl.Enable(enable)
