import os

import wx
from mlib.core.logger import ConsoleHandler, MLogger
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.widgets.console_ctrl import ConsoleCtrl
from mlib.service.form.widgets.exec_btn_ctrl import ExecButton
from mlib.service.form.widgets.float_slider_ctrl import FloatSliderCtrl
from mlib.service.form.widgets.image_btn_ctrl import ImageButton
from mlib.utils.file_utils import save_histories
from service.form.widgets.sizing_bone_set import SizingBoneSet

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
            self, wx.ID_ANY, __("全追加補正ON"), wx.DefaultPosition, wx.Size(200, -1)
        )
        self.full_config_btn_ctrl.SetToolTip(__("全ての追加補正を有効にします"))
        self.full_config_btn_ctrl.Bind(wx.EVT_BUTTON, self.on_full_config)
        self.config_sizer.Add(self.full_config_btn_ctrl, 0, wx.ALL | wx.ALIGN_RIGHT, 3)

        # 全ての親統合 ------------------------
        self.integrate_root_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.integrate_root_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("全ての親統合"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.integrate_root_check_ctrl.SetToolTip(
            __("全ての親の移動や回転を子ボーンに振り分けて、全ての親のキーフレームを削除します")
        )
        self.integrate_root_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_add_config)
        self.integrate_root_sizer.Add(self.integrate_root_check_ctrl, 0, wx.ALL, 3)
        self.integrate_root_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(
                event,
                "全ての親統合",
                [
                    "サイジング先モデルを任意の位置に置きやすくできるよう、全ての親の値を子ボーンに移し替えます",
                    "　・全ての親の移動や回転を、センターや足IKなどの子ボーンに割り当てます",
                    "　・全ての親のキーフレを削除するので、モデルを全ての親で移動させた後にモーションを読み込んでも、原点に戻ったりしなくなります",
                    "　・チェックをONにした場合、サイジングモーションに「I」を追加します",
                ],
            ),
            __("解説をメッセージ欄に表示します"),
        )
        self.integrate_root_sizer.Add(self.integrate_root_help_ctrl, 0, wx.ALL, 0)
        self.config_sizer.Add(self.integrate_root_sizer, 0, wx.ALL, 1)

        # 腰統合 ------------------------
        self.integrate_waist_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.integrate_waist_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("腰統合"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.integrate_waist_check_ctrl.SetToolTip(
            __("腰の移動や回転を上半身・下半身に振り分けて、腰のキーフレームを削除します")
        )
        self.integrate_waist_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_add_config)
        self.integrate_waist_sizer.Add(self.integrate_waist_check_ctrl, 0, wx.ALL, 3)
        self.integrate_waist_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(
                event,
                "腰統合",
                [
                    "サイジング先モデルが元モデルと同じになるように、腰の値を子ボーンに移し替えます",
                    "　・腰の移動や回転を、上半身や下半身に割り当てます",
                    "　・チェックをONにした場合、サイジングモーションに「W」を追加します",
                ],
            ),
            __("解説をメッセージ欄に表示します"),
        )
        self.integrate_waist_sizer.Add(self.integrate_waist_help_ctrl, 0, wx.ALL, 0)
        self.config_sizer.Add(self.integrate_waist_sizer, 0, wx.ALL, 1)

        # # 下半身補正 ------------------------
        # self.stance_lower_sizer = wx.BoxSizer(wx.HORIZONTAL)
        # self.stance_lower_check_ctrl = wx.CheckBox(
        #     self, wx.ID_ANY, __("下半身補正"), wx.DefaultPosition, wx.DefaultSize, 0
        # )
        # self.stance_lower_check_ctrl.SetToolTip(__("足の傾きが同じになるよう、下半身の傾きを調整します"))
        # self.stance_lower_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_add_config)
        # self.stance_lower_sizer.Add(self.stance_lower_check_ctrl, 0, wx.ALL, 3)
        # self.stance_lower_help_ctrl = ImageButton(
        #     self,
        #     "resources/icon/help.png",
        #     wx.Size(12, 12),
        #     lambda event: self.on_help(
        #         event,
        #         "下半身補正",
        #         [
        #             "サイジング先モデルが元モデルと同じになるように、下半身の傾きを調整します",
        #             "　・特にしゃがんでいる時の足の向きが違っている時に有効です",
        #             "　・チェックをONにした場合、サイジングモーションに「L」を追加します",
        #         ],
        #     ),
        #     __("解説をメッセージ欄に表示します"),
        # )
        # self.stance_lower_sizer.Add(self.stance_lower_help_ctrl, 0, wx.ALL, 0)
        # self.config_sizer.Add(self.stance_lower_sizer, 0, wx.ALL, 1)

        # 腕位置合わせ ------------------------
        self.align_arm_group_sizer = wx.BoxSizer(wx.VERTICAL)

        self.align_arm_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.align_arm_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("腕位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.align_arm_check_ctrl.SetToolTip(__("腕周りのを元モーションと大体同じ位置になるよう合わせます"))
        self.align_arm_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_align_arm_ctrl)
        self.align_arm_sizer.Add(self.align_arm_check_ctrl, 0, wx.ALL, 3)

        self.align_arm_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(
                event,
                "腕位置合わせ",
                [
                    "サイジング先モデルが元モデルと同じポーズになるように、腕系ボーンの位置を調整します",
                    "　・肩の高さ、手首の位置など、腕周りのボーンを一括で調整します",
                    "　・特に手を合わせたり、ハートを作るポーズなどが崩れにくくなります",
                    "　・指位置合わせをONにした場合、親指0の差異も含めて調整します",
                    "　・中間点追加にチェックを入れると、全フレームで腕の軌跡がズレてないか確認し、ズレてたら中間キーフレームを追加してズレないようにします。",
                    "　・チェックをONにした場合、サイジングモーションに「P」を追加します",
                ],
            ),
            __("解説をメッセージ欄に表示します"),
        )
        self.align_arm_sizer.Add(self.align_arm_help_ctrl, 0, wx.ALL, 0)
        self.align_arm_group_sizer.Add(self.align_arm_sizer, 0, wx.ALL, 0)

        self.align_arm_finger_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.align_arm_finger_blank = wx.StaticText(self, wx.ID_ANY, "     ")
        self.align_arm_finger_sizer.Add(self.align_arm_finger_blank)

        self.align_arm_finger_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("指位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.align_arm_finger_check_ctrl.Bind(
            wx.EVT_CHECKBOX, self.on_check_align_arm_sub_ctrl
        )
        self.align_arm_finger_check_ctrl.SetToolTip(__("指の位置を元モーションと大体同じ位置になるよう合わせます"))

        self.align_arm_finger_sizer.Add(self.align_arm_finger_check_ctrl, 0, wx.ALL, 3)
        self.align_arm_group_sizer.Add(self.align_arm_finger_sizer, 0, wx.ALL, 0)

        self.align_arm_middle_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.align_arm_middle_blank = wx.StaticText(self, wx.ID_ANY, "     ")
        self.align_arm_middle_sizer.Add(self.align_arm_middle_blank)

        self.align_arm_middle_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("中間点追加"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.align_arm_middle_check_ctrl.Bind(
            wx.EVT_CHECKBOX, self.on_check_align_arm_sub_ctrl
        )
        self.align_arm_middle_check_ctrl.SetToolTip(
            __("腕系キーの間で腕の軌跡がズレないよう、腕系キーフレームの中間でも捩り分散処理を行います")
        )

        self.align_arm_middle_sizer.Add(self.align_arm_middle_check_ctrl, 0, wx.ALL, 3)

        self.align_arm_middle_threshold_text = wx.StaticText(
            self, wx.ID_ANY, __("  閾値")
        )
        self.align_arm_middle_sizer.Add(
            self.align_arm_middle_threshold_text, 0, wx.ALL, 3
        )

        self.align_arm_middle_threshold_slider = FloatSliderCtrl(
            parent=self,
            value=4,
            min_value=0,
            max_value=10,
            increment=0.01,
            spin_increment=0.01,
            border=3,
            size=wx.Size(100, -1),
            tooltip=__("腕系キーフレームの中間で、手首が元からどの程度ズレていたらキーフレームを追加するか指定できます"),
        )
        self.align_arm_middle_sizer.Add(
            self.align_arm_middle_threshold_slider.sizer, 0, wx.ALL, 3
        )
        self.align_arm_group_sizer.Add(self.align_arm_middle_sizer, 0, wx.ALL, 0)

        self.config_sizer.Add(self.align_arm_group_sizer, 0, wx.ALL, 1)

        # 捩り分散 ------------------------
        self.twist_group_sizer = wx.BoxSizer(wx.VERTICAL)

        self.twist_check_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.twist_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("捩り分散"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.twist_check_ctrl.SetToolTip(__("腕を腕捩りなど、捩りボーンに捩り回転を分散させます"))
        self.twist_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_twist_ctrl)
        self.twist_check_sizer.Add(self.twist_check_ctrl, 0, wx.ALL, 3)
        self.twist_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(
                event,
                "捩り分散",
                [
                    "サイジング先モデルの腕の変形が綺麗になるよう、腕捩と手捩を再計算します",
                    "　・腕の軸回転、腕捩の回転、をまとめて腕捩ボーンに割り当てます",
                    "　・ひじの軸回転、手捩の回転、手首の軸回転、をまとめて手捩ボーンに割り当てます",
                    "　・ひじの軸方向以外の回転を、人間のひじ構造と同じようにひじのY方向を曲げるよう、回転軸を調整します",
                    "　・腕の軌跡がズレにくくなるよう、腕・腕捩・ひじ・手捩・手首のいずれかのキーが打たれているキーフレームに対して処理を行います",
                    "　・中間点追加にチェックを入れると、全フレームで腕の軌跡がズレてないか確認し、ズレてたら中間キーフレームを追加してズレないようにします。",
                    "　・チェックをONにした場合、サイジングモーションに「T」を追加します",
                ],
            ),
            __("解説をメッセージ欄に表示します"),
        )
        self.twist_check_sizer.Add(self.twist_help_ctrl, 0, wx.ALL, 0)
        self.twist_group_sizer.Add(self.twist_check_sizer, 0, wx.ALL, 0)

        self.twist_middle_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.twist_middle_blank = wx.StaticText(self, wx.ID_ANY, "     ")
        self.twist_middle_sizer.Add(self.twist_middle_blank)

        self.twist_middle_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("中間点追加"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.twist_middle_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_twist_sub_ctrl)
        self.twist_middle_check_ctrl.SetToolTip(
            __("腕系キーの間で腕の軌跡がズレないよう、腕系キーフレームの中間でも捩り分散処理を行います")
        )

        self.twist_middle_sizer.Add(self.twist_middle_check_ctrl, 0, wx.ALL, 3)

        self.twist_middle_threshold_text = wx.StaticText(self, wx.ID_ANY, __("  閾値"))
        self.twist_middle_sizer.Add(self.twist_middle_threshold_text, 0, wx.ALL, 3)

        self.twist_middle_threshold_slider = FloatSliderCtrl(
            parent=self,
            value=4,
            min_value=0,
            max_value=10,
            increment=0.01,
            spin_increment=0.01,
            border=3,
            size=wx.Size(100, -1),
            tooltip=__("腕系キーフレームの中間で、手首が元からどの程度ズレていたらキーフレームを追加するか指定できます"),
        )
        self.twist_middle_sizer.Add(
            self.twist_middle_threshold_slider.sizer, 0, wx.ALL, 3
        )
        self.twist_group_sizer.Add(self.twist_middle_sizer, 0, wx.ALL, 0)

        self.config_sizer.Add(self.twist_group_sizer, 0, wx.ALL, 1)

        # つま先IK統合 ------------------------
        self.integrate_toe_ik_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.integrate_toe_ik_check_ctrl = wx.CheckBox(
            self, wx.ID_ANY, __("つま先IK統合"), wx.DefaultPosition, wx.DefaultSize, 0
        )
        self.integrate_toe_ik_check_ctrl.SetToolTip(
            __("つま先IKの移動や回転を足IKに振り分けて、つま先IKのキーフレームを削除します")
        )
        self.integrate_toe_ik_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_add_config)
        self.integrate_toe_ik_sizer.Add(self.integrate_toe_ik_check_ctrl, 0, wx.ALL, 3)
        self.integrate_toe_ik_help_ctrl = ImageButton(
            self,
            "resources/icon/help.png",
            wx.Size(12, 12),
            lambda event: self.on_help(
                event,
                "つま先IK統合",
                [
                    "サイジング先モデルが元モデルと同じになるように、つま先IKの値を足IKに移し替えます",
                    "　・チェックをONにした場合、サイジングモーションに「O」を追加します",
                ],
            ),
            __("解説をメッセージ欄に表示します"),
        )
        self.integrate_toe_ik_sizer.Add(self.integrate_toe_ik_help_ctrl, 0, wx.ALL, 0)
        self.config_sizer.Add(self.integrate_toe_ik_sizer, 0, wx.ALL, 1)

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

    def on_help(self, event: wx.Event, title: str, messages: list[str]) -> None:
        self.console_ctrl.write("-------------------------------")
        self.console_ctrl.write(__(title))
        self.console_ctrl.write("-------------------------------")
        for message in messages:
            self.console_ctrl.write(__(message))

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

        self.integrate_root_check_ctrl.SetValue(self.is_full_config)
        self.integrate_waist_check_ctrl.SetValue(self.is_full_config)
        # self.stance_lower_check_ctrl.SetValue(self.is_full_config)
        self.twist_check_ctrl.SetValue(self.is_full_config)
        self.twist_middle_check_ctrl.SetValue(self.is_full_config)
        self.align_arm_check_ctrl.SetValue(self.is_full_config)
        self.align_arm_finger_check_ctrl.SetValue(self.is_full_config)
        self.align_arm_middle_check_ctrl.SetValue(self.is_full_config)
        self.integrate_toe_ik_check_ctrl.SetValue(self.is_full_config)

        self.on_change_dest_model_pmx(event)

    def on_check_add_config(self, event: wx.Event) -> None:
        for sizing_set in self.sizing_sets:
            sizing_set.create_output_path()

    def on_check_twist_ctrl(self, event: wx.Event) -> None:
        if not self.twist_check_ctrl.GetValue():
            self.twist_middle_check_ctrl.SetValue(0)
        self.on_change_dest_model_pmx(event)

    def on_check_twist_sub_ctrl(self, event: wx.Event) -> None:
        if self.twist_middle_check_ctrl.GetValue():
            self.twist_check_ctrl.SetValue(1)
        self.on_change_dest_model_pmx(event)

    def on_check_align_arm_ctrl(self, event: wx.Event) -> None:
        if not self.align_arm_check_ctrl.GetValue():
            self.align_arm_finger_check_ctrl.SetValue(0)
            self.align_arm_middle_check_ctrl.SetValue(0)
        self.on_change_dest_model_pmx(event)

    def on_check_align_arm_sub_ctrl(self, event: wx.Event) -> None:
        if (
            self.align_arm_finger_check_ctrl.GetValue()
            or self.align_arm_middle_check_ctrl.GetValue()
        ):
            self.align_arm_check_ctrl.SetValue(1)
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
        self.integrate_root_check_ctrl.Enable(enable)
        self.integrate_root_help_ctrl.Enable(enable)
        self.integrate_waist_check_ctrl.Enable(enable)
        self.integrate_waist_help_ctrl.Enable(enable)
        # self.stance_lower_check_ctrl.Enable(enable)
        # self.stance_lower_help_ctrl.Enable(enable)
        self.twist_check_ctrl.Enable(enable)
        self.twist_middle_check_ctrl.Enable(enable)
        self.twist_middle_threshold_slider.Enable(enable)
        self.twist_help_ctrl.Enable(enable)
        self.align_arm_check_ctrl.Enable(enable)
        self.align_arm_finger_check_ctrl.Enable(enable)
        self.align_arm_middle_check_ctrl.Enable(enable)
        self.align_arm_middle_threshold_slider.Enable(enable)
        self.align_arm_help_ctrl.Enable(enable)
        self.integrate_toe_ik_check_ctrl.Enable(enable)
        self.integrate_toe_ik_help_ctrl.Enable(enable)

        self.EnableExec(enable)

    def EnableExec(self, enable: bool) -> None:
        self.exec_btn_ctrl.Enable(enable)
