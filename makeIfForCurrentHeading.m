function [output] = makeIfForCurrentHeading(input)

HeadingConstant = input(1);
HeadingRamp = input(2);
HeadingType = input(3);

if HeadingType==0
    output = HeadingConstant;
elseif HeadingType==1
    output = HeadingRamp;
else
end
    
    
end

