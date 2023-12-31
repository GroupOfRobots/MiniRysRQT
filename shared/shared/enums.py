from enum import Enum


class ControlKeyEnum(str, Enum):
    FORWARD = 'forward'
    RIGHT = 'right'
    BACKWARD = 'backward'
    LEFT = 'left'
    STABLE = 'stable'  # position is preserved if stable key is pressed


class MotorControlPositionEnum(int, Enum):
    FORWARD = 0
    FORWARD_LEFT = 1
    FORWARD_RIGHT = 2
    RIGHT = 3
    BACKWARD = 4
    BACKWARD_LEFT = 5
    BACKWARD_RIGHT = 6
    LEFT = 7


motorControlPositionToDataKeyMap = {
    MotorControlPositionEnum.FORWARD: 'forward',
    MotorControlPositionEnum.FORWARD_LEFT: 'forwardLeft',
    MotorControlPositionEnum.FORWARD_RIGHT: 'forwardRight',
    MotorControlPositionEnum.RIGHT: 'right',
    MotorControlPositionEnum.BACKWARD: 'backward',
    MotorControlPositionEnum.BACKWARD_LEFT: 'backwardLeft',
    MotorControlPositionEnum.BACKWARD_RIGHT: 'backwardRight',
    MotorControlPositionEnum.LEFT: 'left'
}


class PackageNameEnum(str, Enum):
    ControlPanel = 'control_panel',
    Joystick = 'joystick',
    CommandsPanel = 'commands_panel',
    SetupPanel = 'setup_panel',
    FanPanel = 'fan_panel',
    Dashboard = 'dashboard'
    DashboardDistanceSensors = 'dashboard_distance_sensors',
    DeletedRobotScreenWidget = 'shared',
    NoRobotConfigurationScreenWidget = 'shared1', # TODO jakoś ominąć shared
    CameraPanel = 'camera_panel',
    CameraVideoRecorderPanel = 'camera_video_recorder_panel',
    ProcessPanel = 'process_panel',


packageNameToDisplayNameMap = {
    PackageNameEnum.ControlPanel: 'Control Panel',
    PackageNameEnum.Joystick: 'Joystick',
    PackageNameEnum.CommandsPanel: 'Commands Panel',
    PackageNameEnum.SetupPanel: 'Setup Panel',
    PackageNameEnum.FanPanel: 'Fan Panel',
    PackageNameEnum.Dashboard: 'Dashboard',
    PackageNameEnum.DashboardDistanceSensors: 'Dashboard Distance Sensors',
    PackageNameEnum.DeletedRobotScreenWidget: 'Deleted Robot',
    PackageNameEnum.NoRobotConfigurationScreenWidget: 'No Robot Configuration',
    PackageNameEnum.CameraPanel: 'Camera Panel',
    PackageNameEnum.CameraVideoRecorderPanel: 'Camera Video Recorder Panel',
    PackageNameEnum.ProcessPanel: 'Process Panel'
}

packageNameToUIFileMap = {
    PackageNameEnum.ControlPanel: 'control_panel.ui',
    PackageNameEnum.Joystick: 'joystick.ui',
    PackageNameEnum.CommandsPanel: 'commands_panel.ui',
    PackageNameEnum.SetupPanel: 'setup_dashboard.ui',
    PackageNameEnum.FanPanel: 'fan_panel.ui',
    PackageNameEnum.Dashboard: 'dashboard.ui',
    PackageNameEnum.DashboardDistanceSensors: 'dashboard_distance_sensors.ui',
    PackageNameEnum.DeletedRobotScreenWidget: 'deleted_robot.ui',
    PackageNameEnum.NoRobotConfigurationScreenWidget: 'no_robot_configuration.ui',
    PackageNameEnum.CameraPanel: 'camera_panel.ui',
    PackageNameEnum.CameraVideoRecorderPanel: 'camera_video_recorder_panel.ui',
    PackageNameEnum.ProcessPanel: 'process_panel.ui'
}
