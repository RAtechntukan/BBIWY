function o_busCommand = UpdateRobotController(p_busRobot)
myRobot = RobotController.getInstance;
o_busCommand = myRobot.updateStep(p_busRobot);

end