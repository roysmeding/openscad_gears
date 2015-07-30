module gearTooth(faceWidth, baseRadius, innerRadius, outerRadius, baseToothThickness, helixTwist=0, numPoints=20, numSlices=10) {
	// if the base radius is smaller than the inner radius, start the involute curve at the inner radius, otherwise at t=0.
	tStart = (baseRadius > innerRadius) ? 0 : acos(baseRadius / innerRadius)*PI/180;
	tEnd = sqrt(outerRadius*outerRadius - baseRadius*baseRadius) / baseRadius;
	tStep = (tEnd-tStart)/(numPoints-1);

	epsilon = 0.01;

	function polar(r,a) = [r*cos(a), r*sin(a)];
	function r2d(a) = a*180/PI;

	function involute(t, a)    = [baseRadius*(cos(r2d(t)+a) + t*sin(r2d(t)+a)),  baseRadius*(sin(r2d(t)+a) - t*cos(r2d(t)+a))];
	function involute_r(t, a)  = [baseRadius*(cos(r2d(t)-a) + t*sin(r2d(t)-a)), -baseRadius*(sin(r2d(t)-a) - t*cos(r2d(t)-a))];

	linear_extrude(height=faceWidth, center=true, twist=helixTwist, slices=numSlices)
		polygon(concat(
				(baseRadius<innerRadius) ? [] : [
					polar(innerRadius,         -baseToothThickness/2),
					polar( baseRadius+epsilon, -baseToothThickness/2),
				],

				[for(t=[tStart :  tStep : tEnd  ]) involute(t,   -baseToothThickness/2) ],
				[for(t=[tEnd   : -tStep : tStart]) involute_r(t,  baseToothThickness/2) ],

				(baseRadius<innerRadius) ? [] : [
					polar( baseRadius+epsilon, baseToothThickness/2),
					polar(innerRadius,         baseToothThickness/2),
				]
			)
		);
}

module spurTeeth(nTeeth, faceWidth, pitchCircleRadius, addendum, dedendum, backlash=0.01, pressureAngle=20, helixAngle=0) {
	// derived variables
	outerRadius = pitchCircleRadius + addendum;
	innerRadius = pitchCircleRadius - dedendum;
	baseRadius  = pitchCircleRadius * cos(pressureAngle);

	// Pitch between successive teeth in degrees on the pitch circle.
	angularPitch       = 360 / nTeeth;

	// Thickness of the gear tooth in degrees on the base circle.
	// The first term is the width on the pitch circle, the second term is a correction
	// to account for the involute profile starting on the base circle.
	baseToothThickness = angularPitch/2 + 2*(tan(pressureAngle)*180/PI - pressureAngle) - backlash*360/(2*PI*pitchCircleRadius);

	// Degrees of twist across the whole gear face width
	helixTwist = tan(helixAngle)*faceWidth*180 / (PI*pitchCircleRadius);

	union() {
		// The teeth
		for(a=[0:angularPitch:360]) {
			rotate(v=[0,0,1], a=a)
				render() gearTooth(faceWidth, baseRadius, innerRadius, outerRadius, baseToothThickness, helixTwist);
		}
	}
} ;

module helicalGear(nTeeth, faceWidth, pitchCircleRadius, hubWidth, ringRadius, centerRadius, holeRadius, helixAngle=30) {
	modul    = pitchCircleRadius*2 / nTeeth;
	addendum = modul;
	dedendum = 1.25*modul;

	innerRadius = pitchCircleRadius - dedendum;

	difference() {
		union() {
			spurTeeth(nTeeth, faceWidth, pitchCircleRadius, addendum, dedendum, helixAngle=helixAngle);
			difference() {
				cylinder(r=innerRadius,              h=faceWidth,     center=true);
				cylinder(r=innerRadius - ringRadius, h=faceWidth+0.1, center=true);
			}
			cylinder(r=innerRadius - ringRadius, h=hubWidth, center=true);
			cylinder(r=centerRadius, h=faceWidth, center=true);
		}
		cylinder(r=holeRadius, h=faceWidth+0.1, center=true);
	}
}

module herringboneGear(nTeeth, faceWidth, pitchCircleRadius, hubWidth, ringRadius, centerRadius, holeRadius, helixAngle=30, offset=0) {
	modul    = pitchCircleRadius*2 / nTeeth;
	addendum = modul;
	dedendum = 1.25*modul;

	innerRadius = pitchCircleRadius - dedendum;

	difference() {
		union() {
			translate([0,0,-faceWidth/4])
				spurTeeth(nTeeth, faceWidth/2, pitchCircleRadius, addendum, dedendum, helixAngle=helixAngle);

			mirror([0,0,1])
				translate([0,0,-faceWidth/4])
					rotate(v=[0,0,1], a=offset*360/nTeeth)
						spurTeeth(nTeeth, faceWidth/2, pitchCircleRadius, addendum, dedendum, helixAngle=helixAngle);

			difference() {
				cylinder(r=innerRadius,              h=faceWidth,     center=true);
				cylinder(r=innerRadius - ringRadius, h=faceWidth+0.1, center=true);
			}

			cylinder(r=innerRadius - ringRadius, h=hubWidth, center=true);
			cylinder(r=centerRadius, h=faceWidth, center=true);
		}
		cylinder(r=holeRadius, h=faceWidth+0.1, center=true);
	}
}

module spurGear(nTeeth, faceWidth, pitchCircleRadius, hubWidth, ringRadius, centerRadius, holeRadius) {
	helicalGear(
		nTeeth            = nTeeth,
		faceWidth         = faceWidth,
		pitchCircleRadius = pitchCircleRadius,
		hubWidth          = hubWidth,
		ringRadius        = ringRadius,
		centerRadius      = centerRadius,
		holeRadius        = holeRadius,
		helixAngle        = 0
	);
}
