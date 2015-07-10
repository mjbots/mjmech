@0xde8badf1cd5e4323;

struct Position3D {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
}

struct Quaternion {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
  w @3 :Float64;
}

struct Frame {
  translation @0 :Position3D;
  rotation @1 :Quaternion;
}

struct GaitData {
  timestamp @0 :Float64;
  bodyRobot @1 :Frame;
  cogRobot @2 :Frame;
  leg1Robot @3 :Position3D;
  leg2Robot @4 :Position3D;
  leg3Robot @5 :Position3D;
  leg4Robot @6 :Position3D;
  phase @7 :Float64;
}
