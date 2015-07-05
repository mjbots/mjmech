@0xd99bf62841b32cf5;

struct ServoData {
  timestamp @0 :Float64;
  values @1 :List(Value);
  struct Value {
    id @0 :Int32;
    angleDeg @1 :Float64;
  }
}
