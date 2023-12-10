class BoneNames:
    @staticmethod
    def sizing_display_slot():
        return "SIZING"

    @staticmethod
    def root():
        return "全ての親"

    @staticmethod
    def shoulder_root(direction: str) -> str:
        return f"{direction}肩根元"

    @staticmethod
    def shoulder_p(direction: str) -> str:
        return f"{direction}肩P"

    @staticmethod
    def shoulder(direction: str) -> str:
        return f"{direction}肩"

    @staticmethod
    def shoulder_c(direction: str) -> str:
        return f"{direction}肩C"

    @staticmethod
    def shoulder_center(direction: str) -> str:
        return f"{direction}肩中点"

    @staticmethod
    def shoulder_ik_parent(direction: str) -> str:
        return f"{direction}肩IK親"

    @staticmethod
    def shoulder_ik(direction: str) -> str:
        return f"{direction}肩IK"

    @staticmethod
    def arm(direction: str) -> str:
        return f"{direction}腕"

    @staticmethod
    def arm_twist(direction: str, index: int = 0):
        return f"{direction}腕捩" if index == 0 else f"{direction}腕捩{index}"

    @staticmethod
    def arm_twist_vertical(direction: str):
        return f"{direction}腕捩垂線"

    @staticmethod
    def arm_ik_parent(direction: str) -> str:
        return f"{direction}腕IK親"

    @staticmethod
    def arm_ik(direction: str) -> str:
        return f"{direction}腕IK"

    @staticmethod
    def arm_direction(direction: str) -> str:
        return f"{direction}腕方向"

    @staticmethod
    def arm_direction_tail(direction: str) -> str:
        return f"{direction}腕方向先"

    @staticmethod
    def arm_direction_ik(direction: str) -> str:
        return f"{direction}腕方向IK"

    @staticmethod
    def arm_rotate(direction: str) -> str:
        return f"{direction}腕回転"

    @staticmethod
    def arm_rotate_tail(direction: str) -> str:
        return f"{direction}腕回転先"

    @staticmethod
    def arm_rotate_ik(direction: str) -> str:
        return f"{direction}腕回転IK"

    @staticmethod
    def elbow(direction: str) -> str:
        return f"{direction}ひじ"

    @staticmethod
    def elbow_vertical(direction: str) -> str:
        return f"{direction}ひじ垂線"

    @staticmethod
    def elbow_ik_parent(direction: str) -> str:
        return f"{direction}ひじIK親"

    @staticmethod
    def elbow_ik(direction: str) -> str:
        return f"{direction}ひじIK"

    @staticmethod
    def elbow_direction(direction: str) -> str:
        return f"{direction}ひじ方向"

    @staticmethod
    def elbow_direction_tail(direction: str) -> str:
        return f"{direction}ひじ方向先"

    @staticmethod
    def elbow_direction_ik(direction: str) -> str:
        return f"{direction}ひじ方向IK"

    @staticmethod
    def elbow_rotate(direction: str) -> str:
        return f"{direction}ひじ回転"

    @staticmethod
    def elbow_rotate_tail(direction: str) -> str:
        return f"{direction}ひじ回転先"

    @staticmethod
    def elbow_rotate_ik(direction: str) -> str:
        return f"{direction}ひじ回転IK"

    @staticmethod
    def wrist_twist(direction: str, index: int = 0):
        return f"{direction}手捩" if index == 0 else f"{direction}手捩{index}"

    @staticmethod
    def wrist_twist_vertical(direction: str):
        return f"{direction}手捩垂線"

    @staticmethod
    def wrist(direction: str) -> str:
        return f"{direction}手首"

    @staticmethod
    def wrist_tail(direction: str) -> str:
        return f"{direction}手首先"

    @staticmethod
    def wrist_vertical(direction: str) -> str:
        return f"{direction}手首垂線"

    @staticmethod
    def wrist_ik_parent(direction: str) -> str:
        return f"{direction}手首IK親"

    @staticmethod
    def wrist_ik(direction: str) -> str:
        return f"{direction}手首IK"

    @staticmethod
    def wrist_rotate(direction: str) -> str:
        return f"{direction}手首回転"

    @staticmethod
    def wrist_rotate_tail(direction: str) -> str:
        return f"{direction}手首回転先"

    @staticmethod
    def wrist_rotate_ik(direction: str) -> str:
        return f"{direction}手首回転IK"

    @staticmethod
    def wrist_direction(direction: str) -> str:
        return f"{direction}手首方向"

    @staticmethod
    def wrist_direction_tail(direction: str) -> str:
        return f"{direction}手首方向先"

    @staticmethod
    def wrist_direction_ik(direction: str) -> str:
        return f"{direction}手首方向IK"

    @staticmethod
    def thumb0(direction: str) -> str:
        return f"{direction}親指０"

    @staticmethod
    def thumb_ik_parent(direction: str) -> str:
        return f"{direction}親IK親"

    @staticmethod
    def thumb_ik(direction: str) -> str:
        return f"{direction}親IK"

    @staticmethod
    def thumb1(direction: str) -> str:
        return f"{direction}親指１"

    @staticmethod
    def thumb2(direction: str) -> str:
        return f"{direction}親指２"

    @staticmethod
    def thumb_tail(direction: str) -> str:
        return f"{direction}親指先"

    @staticmethod
    def index1(direction: str) -> str:
        return f"{direction}人指１"

    @staticmethod
    def index2(direction: str) -> str:
        return f"{direction}人指２"

    @staticmethod
    def index3(direction: str) -> str:
        return f"{direction}人指３"

    @staticmethod
    def index_tail(direction: str) -> str:
        return f"{direction}人指先"

    @staticmethod
    def middle1(direction: str) -> str:
        return f"{direction}中指１"

    @staticmethod
    def middle2(direction: str) -> str:
        return f"{direction}中指２"

    @staticmethod
    def middle3(direction: str) -> str:
        return f"{direction}中指３"

    @staticmethod
    def middle_tail(direction: str) -> str:
        return f"{direction}中指先"

    @staticmethod
    def ring1(direction: str) -> str:
        return f"{direction}薬指１"

    @staticmethod
    def ring2(direction: str) -> str:
        return f"{direction}薬指２"

    @staticmethod
    def ring3(direction: str) -> str:
        return f"{direction}薬指３"

    @staticmethod
    def ring_tail(direction: str) -> str:
        return f"{direction}薬指先"

    @staticmethod
    def pinky1(direction: str) -> str:
        return f"{direction}小指１"

    @staticmethod
    def pinky2(direction: str) -> str:
        return f"{direction}小指２"

    @staticmethod
    def pinky3(direction: str) -> str:
        return f"{direction}小指３"

    @staticmethod
    def pinky_tail(direction: str) -> str:
        return f"{direction}小指先"

    @staticmethod
    def fingers(direction: str) -> list[str]:
        return [
            BoneNames.thumb0(direction),
            BoneNames.thumb1(direction),
            BoneNames.thumb2(direction),
            BoneNames.thumb_tail(direction),
            BoneNames.index1(direction),
            BoneNames.index2(direction),
            BoneNames.index3(direction),
            BoneNames.index_tail(direction),
            BoneNames.middle1(direction),
            BoneNames.middle2(direction),
            BoneNames.middle3(direction),
            BoneNames.middle_tail(direction),
            BoneNames.ring1(direction),
            BoneNames.ring2(direction),
            BoneNames.ring3(direction),
            BoneNames.ring_tail(direction),
            BoneNames.pinky1(direction),
            BoneNames.pinky2(direction),
            BoneNames.pinky3(direction),
            BoneNames.pinky_tail(direction),
        ]

    # @staticmethod
    # def parent() -> str:
    #     return "全ての親"

    @staticmethod
    def center() -> str:
        return "センター"

    @staticmethod
    def groove() -> str:
        return "グルーブ"

    @staticmethod
    def waist() -> str:
        return "腰"

    @staticmethod
    def upper() -> str:
        return "上半身"

    @staticmethod
    def upper2() -> str:
        return "上半身2"

    @staticmethod
    def upper3() -> str:
        return "上半身3"

    @staticmethod
    def lower() -> str:
        return "下半身"

    @staticmethod
    def neck() -> str:
        return "首"

    @staticmethod
    def head() -> str:
        return "頭"

    @staticmethod
    def waist_cancel(direction: str) -> str:
        return f"腰キャンセル{direction}"

    @staticmethod
    def leg(direction: str) -> str:
        return f"{direction}足"

    @staticmethod
    def knee(direction: str) -> str:
        return f"{direction}ひざ"

    @staticmethod
    def ankle(direction: str) -> str:
        return f"{direction}足首"

    @staticmethod
    def leg_d(direction: str) -> str:
        return f"{direction}足D"

    @staticmethod
    def knee_d(direction: str) -> str:
        return f"{direction}ひざD"

    @staticmethod
    def ankle_d(direction: str) -> str:
        return f"{direction}足首D"

    @staticmethod
    def toe(direction: str) -> str:
        return f"{direction}つま先"

    @staticmethod
    def toe_ex(direction: str) -> str:
        return f"{direction}足先EX"

    @staticmethod
    def leg_ik_parent(direction: str) -> str:
        return f"{direction}足IK親"

    @staticmethod
    def leg_ik(direction: str) -> str:
        return f"{direction}足ＩＫ"

    @staticmethod
    def toe_ik(direction: str) -> str:
        return f"{direction}つま先ＩＫ"
