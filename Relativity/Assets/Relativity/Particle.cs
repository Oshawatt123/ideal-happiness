using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Particle : MonoBehaviour
{
    // Debug
    public bool DrawDebugGizmos = false;
    private List<KeyValuePair<Vector3, Color>> GizmosQueue = new List<KeyValuePair<Vector3, Color>>();

    // Globals
    public /*const*/ Vector3 GRAVITY = new Vector3(0f, -9.8f, 0f);        // Gravity
    public const float AIRDENSITY = 1.23f; // 1.23kg/m^3 -> measured at sea level on earth at 15C
    public const float COEFFICIENTOFDRAG = 0.47f; // measured result https://en.wikipedia.org/wiki/Drag_coefficient
    public const float WINDSPEED = 20.0f;                                // idk its whatever we choose, its wind speed
    public /*const*/ Vector3 WINDDIRECTION = new Vector3(1f, 0f, 0f);   // ^^  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~  ^^ direction


    public float Mass = 1.0f;                                   // Mass
    //public Vector3 Position;                                  // transform.position
    public Vector3 Velocity = Vector3.zero;                     // Velocity
    public float Speed = 0.0f;                                  // Speed (Magnitude of velocity)
    public Vector3 Forces = Vector3.zero;                       // Total force acting on this particle
    public float Radius = 0.1f;                                 // Radius of the particle

    // Constructor
    private void Start()
    {
        GRAVITY.y = GRAVITY.y * Mass; // dont like this too much as gravity is an acceleration not a force but I guess we're saving computation :/
    }

    private void Update()
    {
        // Debug
        if(DrawDebugGizmos)
        {
            GizmosQueue.Clear();
        }

        CalcLoads();

        UpdateBodyEuler(Time.deltaTime);
    }

    // Aggregate forces acting on this particle
    public void CalcLoads()
    {
        // Reset forces
        Forces = Vector3.zero;

        // Aggregate Forces

        // Gravity
        Gizmos.color = Color.yellow;
        AddForceInternal(GRAVITY, true);

#region Still Air Drag
        // Still Air Drag
        // F = 1/2 * p * V^2 * A * C
        // F -> drag force
        // p -> air density
        // V -> magnitude of velocity (speed)
        // A -> projected area of particle moving against air
        // C -> Coefficient of drag
        Vector3 AirDragForce = Vector3.zero;
        float AirDragForceMag = 0.0f;

        // Still Air Drag acts opposite to the direction of motion
        // Therefore, DragVelocity will be the negative velocity vector normalized,
        // which gives us a normal vector opposite to the direction of motion
        AirDragForce -= Velocity;
        AirDragForce = AirDragForce.normalized;

        // pre-calculate our projected area of resistance
        // for a sphere, a circular area would be impacting air
        // so Pi*r^2 for the area effected by air drag
        float A = Mathf.PI * Radius * Radius;

        // calculate drag magnitude
        AirDragForceMag = 0.5f * AIRDENSITY * Mathf.Pow(Speed, 2) * A * COEFFICIENTOFDRAG;

        // Times our direction vector by the force magnitude to get our velocity
        AirDragForce *= AirDragForceMag;

        // Apply Still Air Drag
        Gizmos.color = Color.blue;
        AddForceInternal(AirDragForce, true);
#endregion

#region Wind
        // We're using the same formula for wind as we did for still air drag
        Vector3 WindForce = Vector3.zero;
        float WindForceMag = 0.0f;

        // Get normalized direction of our force
        WindForce = WINDDIRECTION.normalized;

        // calculate drag magnitude (re-uing A from still air drag)
        WindForceMag = 0.5f * AIRDENSITY * Mathf.Pow(WINDSPEED, 2) * A * COEFFICIENTOFDRAG;

        // Times our direction vector by the force magnitude to get our velocity
        WindForce *= WindForceMag;

        // Apply Still Air Drag
        Gizmos.color = Color.red;
        AddForceInternal(WindForce, true);
#endregion

    }

    private void AddForceInternal(Vector3 force, bool gizmoColorSet = false)
    {
        if(DrawDebugGizmos && gizmoColorSet)
        {
            GizmosQueue.Add(new KeyValuePair<Vector3, Color>(force, Gizmos.color));
        }
        Forces += force;
    }

    // Integrate one time step dt
    public void UpdateBodyEuler(float deltaTime)
    {
        // Setup
        Vector3 acceleration = Vector3.zero;
        Vector3 deltaVelocity = Vector3.zero;
        Vector3 deltaPosition = Vector3.zero;

        // Calculate particle acceleration from aggregated forces
        acceleration = Forces / Mass;

        // Integrate velocity from acceleration
        deltaVelocity = acceleration * deltaTime;
        Velocity += deltaVelocity;

        // Integrate position from our new velocity value velocity
        deltaPosition = Velocity * deltaTime;
        transform.position += deltaPosition;

        // Misc. Calculations
        Speed = Velocity.magnitude;
    }

    private void OnDrawGizmos()
    {
        if(DrawDebugGizmos)
        {
            GizmosQueue.ForEach(pair =>
            {
                Gizmos.color = pair.Value;
                Gizmos.DrawLine(transform.position, transform.position + pair.Key);
            });
        }
    }
}
