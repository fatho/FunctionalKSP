namespace ANewDawn.Math

[<Struct>]
type vec3<[<Measure>] 'u> = 
    { x: float<'u>
      y: float<'u>
      z: float<'u> }

    static member (+)(v1: vec3<'u>, v2: vec3<'u>) =
        { x = v1.x + v2.x; y = v1.y + v2.y; z = v1.z + v2.z }
    static member (-)(v1: vec3<'u>, v2: vec3<'u>) =
        { x = v1.x - v2.x; y = v1.y - v2.y; z = v1.z - v2.z }
    static member (~-)(v2: vec3<'u>) =
        { x = - v2.x; y = - v2.y; z = - v2.z }
    static member (*)(v1: vec3<'u>, s: float<'v>) =
        { x = v1.x * s; y = v1.y * s; z = v1.z * s}
    static member (/)(v1: vec3<'u>, s: float<'v>) =
        { x = v1.x / s; y = v1.y / s; z = v1.z / s}
    static member (*)(s: float<'v>, v1: vec3<'u>) =
        { x = v1.x * s; y = v1.y * s; z = v1.z * s}
    
    override this.ToString() = sprintf "V(%.3f, %.3f, %.3f)" this.x this.y this.z

module Vec3 =
    open ANewDawn.Units

    let pack<[<Measure>] 'u> (x: float, y: float, z: float): vec3<'u> =
        let asU v : float<'u> = LanguagePrimitives.FloatWithMeasure v
        { x = asU x; y = asU y; z = asU z }
        
    let unpack<[<Measure>] 'u> (v: vec3<'u>): float * float * float =
        float v.x, float v.y, float v.z

    let dot<[<Measure>] 'u, [<Measure>] 'v> (v1: vec3<'u>) (v2: vec3<'v>): float<'u * 'v> =
        v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    let magSq<[<Measure>] 'u> (v: vec3<'u>): float<'u^2> = dot v v

    let mag<[<Measure>] 'u> (v: vec3<'u>): float<'u> = sqrt (magSq v)

    let norm<[<Measure>] 'u> (v: vec3<'u>): vec3<1> = v / mag v

    let cosAngle<[<Measure>] 'u, [<Measure>] 'v> (v1: vec3<'u>) (v2: vec3<'v>): float = dot (norm v1) (norm v2)
    
    let angle<[<Measure>] 'u, [<Measure>] 'v> (v1: vec3<'u>) (v2: vec3<'v>): float<rad> = acos (cosAngle v1 v2) * 1.<rad>

    let zip<[<Measure>] 'u, [<Measure>] 'v, [<Measure>] 'w> (f: float<'u> -> float<'v> -> float<'w>) (v1: vec3<'u>) (v2: vec3<'v>): vec3<'w> =
        { x = f v1.x v2.x; y = f v1.y v2.y; z = f v1.z v2.z }

    let map<[<Measure>] 'u, [<Measure>] 'v> (f: float<'u> -> float<'v>) (v1: vec3<'u>) : vec3<'v> =
        { x = f v1.x; y = f v1.y; z = f v1.z }

    let project<[<Measure>] 'u, [<Measure>] 'v> (v: vec3<'u>) (axis: vec3<1>): vec3<'u> =
        axis * dot v axis
    
    let exclude<[<Measure>] 'u> (v: vec3<'u>) (axis: vec3<1>) =
        v - project v axis

    let cross<[<Measure>] 'u, [<Measure>] 'v> (a: vec3<'u>) (b: vec3<'v>): vec3<'u * 'v> =
        { x = a.y * b.z - a.z * b.y; y = a.z * b.x - a.x * b.z; z = a.x * b.y - a.y * b.x }
   
    let compassHeading (north: vec3<1>) (east: vec3<1>) (v: vec3<1>) =
        let vnorth = dot v north
        let veast = dot v east
        let angle = 90.<deg> - atan2 vnorth veast * 1.<rad> * degPerRad
        if angle < 0.<deg>
            then angle + 360.<deg>
            else angle
           
    let unitX: vec3<1> = pack (1., 0., 0.)
    let unitY: vec3<1> = pack (0., 1., 0.)
    let unitZ: vec3<1> = pack (0., 0., 1.)

    let zero<[<Measure>] 'u> = pack<'u> (0., 0., 0.)
