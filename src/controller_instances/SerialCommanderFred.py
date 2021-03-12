from clamp_controller.ClampModel import ClampModel
from clamp_controller.SerialCommander import SerialCommander
from clamp_controller.RosClampCommandListener import RosClampCommandListener

# SerialCommanderTokyo is a SErialCommander that contains the initialization of the clamps
# used in the Tokyo project.


class SerialCommanderFred(SerialCommander):

    def __init__(self):
        SerialCommander.__init__(self)

        # 918 step/mm is derived from
        # - 17 steps per rev encoder x 4 phase
        # - 1:54 gearbox
        # - 4mm lead screw

        # Soft Limit Min Max is calibrated to the CL3 clamp that was constructed.

        # Batt Min Max Value 860 to 1004 is calibrated according to the LiPo Charger's percentage reference
        # It is safe to use the battery to 0% as indicated here.
        self.clamp1: ClampModel = ClampModel('1', 918, 0, 94.0, 225.0, 860.0, 1004.0)
        self.clamp2: ClampModel = ClampModel('2', 918, 0, 94.0, 225.0, 860.0, 1004.0)
        self.clamp3: ClampModel = ClampModel('3', 918, 0, 94.0, 225.0, 860.0, 1004.0)
        self.clamp4: ClampModel = ClampModel('4', 918, 0, 94.0, 225.0, 860.0, 1004.0)
        self.add_clamp(self.clamp1)
        self.add_clamp(self.clamp2)
        self.add_clamp(self.clamp3)
        self.add_clamp(self.clamp4)

        self.ros_client: RosClampCommandListener = None
        self.status_update_interval_low_ms = 950