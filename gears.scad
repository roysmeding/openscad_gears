module gearTooth(faceWidth, baseRadius, innerRadius, outerRadius, baseToothThickness, helixTwist=0, numPoints=20, numSlices=10) {
	// A single involute profile gear tooth.
	// Parameters:
	//  - faceWidth:   Length of the tooth along the gear's axis of rotation.
	//  - baseRadius:  Radius (from the gear center) where the involute curve originates.
	//  - innerRadius: Radius where the tooth as a whole starts. Can be larger or smaller than the baseRadius.
	//  - outerRadius: Radius where the tooth ends.
	//  - baseToothThickness: Tooth thickness, measured in degrees at the baseRadius.
	//  - helixTwist:  Angle the tooth twists over the entire faceWidth.
	//  - numPoints:   Number of points to use for each involute curve.
	//  - numSlices:   Number of slices for the tooth along the axis of rotation.

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

module spurTeeth(nTeeth, faceWidth, pitchCircleRadius, addendum, dedendum, backlash=0.01, helixAngle=0, pressureAngle=20) {
	// A complete set of spur or helical gear teeth.
	// Parameters:
	//  - nTeeth:    Number of teeth.
	//  - faceWidth: Length of the teeth along the gear's axis of rotation.
	//  - pitchCircleRadius: Radius of the pitch circle, a fundamental parameter of a gear.
	//  - addendum:  Amount the gear extends beyond the pitch circle radius.
	//  - dedendum:  Offset between base of gear teeth and pitch circle radius.
	//  - backlash:  Amount of backlash (in length units) as measured along the pitch circle.
	//  - pressureAngle: Pressure angle, another fundamental gear parameter. Affects shape of teeth.
	//  - helixAngle: Angle between the gear tooth and the axis of rotation of the gear.

	// derived variables
	outerRadius = pitchCircleRadius + addendum;
	innerRadius = pitchCircleRadius - dedendum;
	baseRadius  = pitchCircleRadius * cos(pressureAngle);

	// Pitch between successive teeth in degrees on the pitch circle.
	angularPitch       = 360 / nTeeth;

	// Thickness of the gear tooth in degrees on the base circle.
	// The first term is the angle on the pitch circle, the second term is a correction
	// to convert this to an angle on the base circle.
	baseToothThickness = angularPitch/2 + 2*(tan(pressureAngle)*180/PI - pressureAngle) - backlash*360/(2*PI*pitchCircleRadius);

	// Compute degrees of twist across the whole gear face width, instead of the weird helix angle.
	helixTwist = tan(helixAngle)*faceWidth*180 / (PI*pitchCircleRadius);

	for(a=[0:angularPitch:360]) {
		rotate(v=[0,0,1], a=a)
			render() gearTooth(faceWidth, baseRadius, innerRadius, outerRadius, baseToothThickness, helixTwist);
	}
} ;

module helicalGear(nTeeth, faceWidth, pitchCircleRadius, diskWidth, ringRadius, centerRadius, holeRadius, helixAngle=30) {
	// A helical gear at a standard pressure angle, backlash, addendum and dedendum.
	// Parameters:
	//  - nTeeth:       Number of teeth.
	//  - faceWidth:    Length of the teeth along the gear's axis of rotation.
	//  - pitchCircleRadius: Radius of the pitch circle, a fundamental parameter of a gear.
	//  - diskWidth:    Width of the 'disk' (main inner body of the gear) along the gear axis of rotation.
	//  - ringRadius:   Radius of the outer 'ring' that the teeth attach to.
	//  - centerRadius: Radius of the center cylinder with the shaft hole in it.
	//  - holeRadius:   Radius of the hole
	//  - helixAngle:   Angle between the gear tooth and the axis of rotation of the gear.

	modul    = pitchCircleRadius*2 / nTeeth;

	// These are standard addendum/dedendum values from the AGMA for general use.
	addendum = modul;
	dedendum = 1.25*modul;

	innerRadius = pitchCircleRadius - dedendum;

	difference() {
		union() {
			// Teeth
			spurTeeth(nTeeth, faceWidth, pitchCircleRadius, addendum, dedendum, pressureAngle=pressureAngle, helixAngle=helixAngle);

			// Ring that the teeth attach to
			difference() {
				cylinder(r=innerRadius,              h=faceWidth,     center=true);
				cylinder(r=innerRadius - ringRadius, h=faceWidth+0.1, center=true);
			}

			// 
			cylinder(r=innerRadius - ringRadius, h=diskWidth, center=true);
			cylinder(r=centerRadius, h=faceWidth, center=true);
		}
		cylinder(r=holeRadius, h=faceWidth+0.1, center=true);
	}
}

module herringboneGear(nTeeth, faceWidth, pitchCircleRadius, diskWidth, ringRadius, centerRadius, holeRadius, helixAngle=30, offset=0) {
	// A standard herringbone gear, consisting of two helical parts to cancel out axial forces.
	// Parameters:
	//  - nTeeth:       Number of teeth.
	//  - faceWidth:    Length of the teeth along the gear's axis of rotation.
	//  - pitchCircleRadius: Radius of the pitch circle, a fundamental parameter of a gear.
	//  - diskWidth:    Width of the 'disk' (main inner body of the gear) along the gear axis of rotation.
	//  - ringRadius:   Radius of the outer 'ring' that the teeth attach to.
	//  - centerRadius: Radius of the center cylinder with the shaft hole in it.
	//  - holeRadius:   Radius of the hole
	//  - helixAngle:   Angle between the gear tooth and the axis of rotation of the gear.
	//  - offset:       Rotational offset between the two helical parts, as a fraction of the  tooth pitch.

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

			cylinder(r=innerRadius - ringRadius, h=diskWidth, center=true);
			cylinder(r=centerRadius, h=faceWidth, center=true);
		}
		cylinder(r=holeRadius, h=faceWidth+0.1, center=true);
	}
}

module spurGear(nTeeth, faceWidth, pitchCircleRadius, diskWidth, ringRadius, centerRadius, holeRadius) {
	// A boring spur gear, without any twist or anything.
	// Parameters:
	//  - nTeeth:       Number of teeth.
	//  - faceWidth:    Length of the teeth along the gear's axis of rotation.
	//  - pitchCircleRadius: Radius of the pitch circle, a fundamental parameter of a gear.
	//  - diskWidth:    Width of the 'disk' (main inner body of the gear) along the gear axis of rotation.
	//  - ringRadius:   Radius of the outer 'ring' that the teeth attach to.
	//  - centerRadius: Radius of the center cylinder with the shaft hole in it.
	//  - holeRadius:   Radius of the hole

	helicalGear(
		nTeeth            = nTeeth,
		faceWidth         = faceWidth,
		pitchCircleRadius = pitchCircleRadius,
		diskWidth         = diskWidth,
		ringRadius        = ringRadius,
		centerRadius      = centerRadius,
		holeRadius        = holeRadius,
		helixAngle        = 0
	);
}

module rackTooth(faceWidth, pitch, helixAngle=0, pressureAngle=20) {
	baseToothThickness = pitch/2;
	modul = pitch / PI;

	add = modul;
	ded = 1.25 * modul;

	// (half of) inner and outer tooth widths
	iw = baseToothThickness/2 - add*tan(pressureAngle);
	ow = baseToothThickness/2 + ded*tan(pressureAngle);

	dx = tan(helixAngle) * faceWidth/2;

	polyhedron(
			points=[
				[-iw-dx, -ded, -faceWidth/2],
				[-ow-dx,  add, -faceWidth/2],
				[ ow-dx,  add, -faceWidth/2],
				[ iw-dx, -ded, -faceWidth/2],
				[-iw+dx, -ded,  faceWidth/2],
				[-ow+dx,  add,  faceWidth/2],
				[ ow+dx,  add,  faceWidth/2],
				[ iw+dx, -ded,  faceWidth/2]
			],

			faces=[
				[0,1,2,3],
				[0,1,5,4],
				[1,2,6,5],
				[2,3,7,6],
				[0,3,7,4],
				[4,5,6,7]
			],

			convexity=2
		);
}

module rack(length, depth, faceWidth, pitch, angle, offset=0, pressureAngle=20) {
	// A rack that meshes with a helical or spur gear.
	// Parameters:
	//  - length: length of the rack
	//  - depth: length of the rack along the direction the teeth point in
	//  - faceWidth: length of the teeth along the long axis of the rack (the axis along which it can move)
	//  - pitch: number of teeth per unit length
	//  - angle: angle of teeth relative to the long axis of the rack. compatible with helicalGear's helixAngle.
	modul = pitch / PI;

	add = modul;
	ded = 1.25 * modul;

	union() {
		cube_depth = depth-(add+ded);
		translate([0,cube_depth/2+add,0])
			cube([length, cube_depth, faceWidth], center=true);

		for(x=[-length/2 + offset*pitch:pitch:length/2]) {
			translate([x,0,0]) {
				rackTooth(
						faceWidth     = faceWidth,
						pitch         = pitch,
						helixAngle    = angle,
						pressureAngle = pressureAngle
					);
			}
		}
	}
}
