module FunctionalKSP.Telemetry

open KRPC.Client
open KRPC.Schema;
open KRPC.Client.Services.SpaceCenter;
open System

open Units;
open FunctionalKSP.LinearAlgebra

/// Some stream providing values
type IStream<'a> =
    abstract Value: 'a with get

/// A wrapper that allows streams being used with `use` and also applies post-processing.
[<AbstractClass>]
type MeasuredStream<'a, 'b>(stream: Stream<'a>) =
    interface IStream<'b> with
        member this.Value with get(): 'b = this.Value
    
    abstract member Value: 'b with get

    member this.RpcStream with get(): Stream<'a> = stream

    interface IDisposable with
        member this.Dispose() = stream.Remove()

type FloatMeasuredStream<[<Measure>]'u>(stream: Stream<float>) =
    inherit MeasuredStream<float, float<'u>>(stream)
    override this.Value with get() = LanguagePrimitives.FloatWithMeasure (stream.Get())

type Float32MeasuredStream<[<Measure>]'u>(stream: Stream<float32>) =
    inherit MeasuredStream<float32, float32<'u>>(stream)
    override this.Value with get() = LanguagePrimitives.Float32WithMeasure (stream.Get())
    
type GenericMeasuredStream<'a>(stream: Stream<'a>) =
    inherit MeasuredStream<'a, 'a>(stream)
    override this.Value with get() = stream.Get()

type Vec3FloatMeasuredStream<[<Measure>]'u>(stream: Stream<float * float * float>) =
    inherit MeasuredStream<float * float * float, vec3<'u>>(stream)
    override this.Value with get() = Vec3.pack (stream.Get())
    
type TwoVec3FloatMeasuredStream<[<Measure>]'u>(stream: Stream<(float * float * float) * (float * float * float)>) =
    inherit MeasuredStream<(float * float * float) * (float * float * float), vec3<'u> * vec3<'u>>(stream)
    override this.Value
        with get() =
            let v1, v2 = stream.Get()
            Vec3.pack v1, Vec3.pack v2

type Connection with
    member conn.UseStream<[<Measure>]'u>(expr: System.Linq.Expressions.Expression<Func<float>>): MeasuredStream<float, float<'u>> =
        let stream = conn.AddStream(expr)
        upcast new FloatMeasuredStream<'u>(stream)
        
    member conn.UseStream<[<Measure>]'u>(expr: System.Linq.Expressions.Expression<Func<float32>>): MeasuredStream<float32, float32<'u>> =
        let stream = conn.AddStream(expr)
        upcast new Float32MeasuredStream<'u>(stream)
        
    member conn.UseStream<[<Measure>]'u>(expr: System.Linq.Expressions.Expression<Func<float * float * float>>): MeasuredStream<float * float * float, vec3<'u>> =
        let stream = conn.AddStream(expr)
        upcast new Vec3FloatMeasuredStream<'u>(stream)
        
    member conn.UseStream<[<Measure>]'u>(expr: System.Linq.Expressions.Expression<Func<(float * float * float) * (float * float * float)>>): MeasuredStream<(float * float * float)*(float * float * float), vec3<'u> * vec3<'u>> =
        let stream = conn.AddStream(expr)
        upcast new TwoVec3FloatMeasuredStream<'u>(stream)

    member conn.UseStream<'a>(expr: System.Linq.Expressions.Expression<Func<'a>>): MeasuredStream<'a, 'a> =
        let stream = conn.AddStream(expr)
        upcast new GenericMeasuredStream<'a>(stream)