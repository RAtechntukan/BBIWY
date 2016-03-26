function o_busCommand = updateOutput(obj)
o_busCommand = [];
o_busCommand.Mode = EMode.SPEED;
o_busCommand.Angle = single(0.1);
o_busCommand.Position = single(1);
end