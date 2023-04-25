// Functions to build Totemmaker
//
// Number is number of centimetres not actual
// value in milimetres
//

// Laika

translate([-70, -60, -70]) {

// Lower Frame
totem_x(16);
translate([0, 110, 0]) totem_x(16);
translate([0, 10, 0]) totem_y(10);
translate([150, 10, 0]) totem_y(10);
translate([50, 10, 0]) totem_y(10);
translate([100, 10, 0]) totem_y(10);

// Motors
translate([-20, 27, 12])  motor(true);
translate([-20, 92, 12])  motor(false);
translate([180, 27, 12])  motor(true);
translate([180, 92, 12])  motor(false);
translate([-25, 0, 20]) cube([10, 120, 10]);
translate([175, 0, 20]) cube([10, 120, 10]);

// Lower Uprights
translate([0, 0, 10]) totem_z(3);
translate([0, 110, 10]) totem_z(3);
translate([150, 0, 10]) totem_z(3);
translate([150, 110, 10]) totem_z(3);
translate([80, 0, 10]) totem_z(3);
translate([80, 110, 10]) totem_z(3);

// Upper Frame
translate([-70, 0, 40]) totem_x(28);
translate([-70, 110, 40]) totem_x(28);
translate([0, 10, 40]) totem_y(10);
translate([150, 10, 40]) totem_y(10);
translate([175, 0, 30]) totem_y(12);
translate([200, 10, 40]) totem_y(10);

// Tower Uprights
translate([0, 0, 50]) totem_z(4);
translate([0, 110, 50]) totem_z(4);
translate([110, 0, 50]) totem_z(4);
translate([110, 110, 50]) totem_z(4);
translate([0, 0, 90]) cube([120, 120, 10]);

// Front Bonnet
translate([150, 0, 50]) cube([60, 120, 2]);

// Camera Mount
translate([210, 45, 25]) cube([5, 30, 30]);

// Laika Body

$fn=50;
minkowski()
{
  translate([15, 15, 10]) cube([130,90,80]);
  cylinder(r=5,h=1);
}
translate([135,65,90]) cylinder(r=10, h=3);

// Raspberry Pi

minkowski()
{
  translate([-65, 15, 40]) cube([60,90,30]);

}

translate([-25, 0, 30]) totem_y(12);
translate([-60, 0, 30]) totem_y(12);

// Lidar Base
translate([10, 60, 100])
hull() {
    cylinder(r=5, h=40);
    translate([20,25,0]) cylinder(r=5, h=40);
    translate([20,-25,0]) cylinder(r=5, h=40);
    translate([40,30,0]) cylinder(r=5, h=40);
    translate([40,-30,0]) cylinder(r=5, h=40);
    translate([60,40,0]) cylinder(r=5, h=40);
    translate([60,-40,0]) cylinder(r=5, h=40);
    translate([80,40,0]) cylinder(r=5, h=40);
    translate([80,-40,0]) cylinder(r=5, h=40);
    translate([95,35, 0]) cylinder(r=5, h=40);
    translate([95,-35, 0]) cylinder(r=5, h=40);
    translate([100, 0, 0]) cylinder(r=5, h=40);
}

}

// Motor Module

module motor(left)
{
    if(left==true)
        {
            rotate([-90, 0, 0])
            {
                cylinder(h=55, r=12, center=true);
                translate([0, 0, 97]) cylinder(h=10, r=5, center=true);
            }
        }
  else
       {
            rotate([-90, 0, 0])
            {
                cylinder(h=55, r=12, center=true);
                translate([0, 0, -97]) cylinder(h=10, r=5, center=true);
            }
        }
    
}

// Totemmaker modules

module totem_x(length)
{
    for(i = [0 : 10 : (length-1)*10])
    {
        translate([i , 0, 0]) ihole_x();
    }
}

module totem_y(length)
{
    rotate([0, 0, 90]) 
    {
        translate([0, -10, 0]) totem_x(length);
    }
}

module totem_z(length)
{
    rotate([0, -90, 0]) 
    {
        translate([0, 0, -10]) totem_x(length);
    }
}

module ihole_x()
{
    difference()
    {
        ibeam_x();
        rotate([90, 0, 0])
        {
            translate([5 ,5, -5]) cylinder(h=2.1, d=4, center = true);
        }
    }  
}

module ibeam_x(length)
{
    cube([10, 10, 2]);
    translate([0, 0, 8]) cube([10, 10, 2]);
    translate([0, 4, 2]) cube([10, 2, 6]);
}