using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Particle : MonoBehaviour
{
    // Debug
    enum GizmoType
    {
        Line,
        Sphere
    }
    class GizmoDrawOptions
    {
        public GizmoType gizmoType;

        // Line
        public Vector3 forceLine;

        // Sphere
        public Vector3 position;
        public float radius;

        public Color color;

        public float time = 0f;
    }
    public bool DrawDebugGizmos = false;
    private List<GizmoDrawOptions> GizmosQueue = new List<GizmoDrawOptions>();

    // Globals
    public /*const*/ Vector3 GRAVITY = new Vector3(0f, -9.8f, 0f);        // Gravity
    public const float AIRDENSITY = 1.23f; // 1.23kg/m^3 -> measured at sea level on earth at 15C
    public const float COEFFICIENTOFDRAG = 0.47f; // measured result https://en.wikipedia.org/wiki/Drag_coefficient
    public const float WINDSPEED = 20.0f;                                // idk its whatever we choose, its wind speed
    public /*const*/ Vector3 WINDDIRECTION = new Vector3(1f, 0f, 0f);   // ^^  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~  ^^ direction
    public /*const*/ float GROUND_Y_POSITION = 0f;


    public float Mass = 1.0f;                                   // Mass
    //public Vector3 Position;                                  // transform.position
    public Vector3 Velocity = Vector3.zero;                     // Velocity
    public float Speed = 0.0f;                                  // Speed (Magnitude of velocity)
    public Vector3 Forces = Vector3.zero;                       // Total force acting on this particle
    public float Radius = 0.1f;                                 // Radius of the particle

    // Collision
    private Vector3 PreviousPosition;
    private Vector3 ImpactForces;
    private bool HasCollided;
    [SerializeField]
    private float CoefficientOfRestitution = 0.6f;

    // Constructor
    private void Start()
    {
        GRAVITY.y = GRAVITY.y * Mass; // dont like this too much as gravity is an acceleration not a force but I guess we're saving computation :/
    }

    private void Update()
    {
        float timeStep = 0.5f;
        // Debug
        if (DrawDebugGizmos)
        {
            List<GizmoDrawOptions> gizmosToClear = new List<GizmoDrawOptions>();
            GizmosQueue.ForEach(drawOption =>
            {
                if(drawOption.time != -1) // time of -1 means persistent, skip removal
                {
                    if(drawOption.time == 0) // time of 0 means one frame, so force removal
                    {
                        gizmosToClear.Add(drawOption);
                    }
                    else
                    {
                        drawOption.time -= Time.deltaTime;
                        if (drawOption.time < 0)
                        {
                            gizmosToClear.Add(drawOption);
                        }
                    }
                }
            });

            gizmosToClear.ForEach(drawOption =>
            {
                GizmosQueue.Remove(drawOption);
            });
        }

        HasCollided = CheckForCollisions(timeStep);

        CalcLoads();

        PreviousPosition = transform.position;
        UpdateBodyEuler(timeStep);

    }

    public bool CheckForCollisions(float dt)
    {
        // Setup
        Vector3 collisionNormal = Vector3.zero;
        Vector3 relativeVelocity = Vector3.zero;
        float relativeVelocityDot = 0f;
        float collisionImpulse = 0f;
        Vector3 collisionImpactForce = Vector3.zero;
        bool hasCollided = false;

        ImpactForces = Vector3.zero;

        // Collision Detection

        // Ground Check
        if(transform.position.y <= GROUND_Y_POSITION)
        {
            collisionNormal.x = 0;
            collisionNormal.y = 1;          // ground is static, a flat plane, and we are only interested in collisions from above; we can determine our collision normal as being Y up
            relativeVelocity = Velocity;    // ground is static, so relative velocity i just the velocity of the particle
            relativeVelocityDot = Vector3.Dot(relativeVelocity, collisionNormal); // dot product determines how parallel our vector are

            // if dot is negative, aka vectors pointing away from each other
            // aka velocity is opposite to collision normal
            // aka we are moving towards the ground
            if(relativeVelocityDot < 0f)
            {
                collisionImpulse = -(relativeVelocityDot) * (CoefficientOfRestitution + 1) * Mass;
                collisionImpactForce = collisionNormal;
                collisionImpactForce.Scale(collisionImpactForce / dt); // vector3.scale is vector multiplication

                ImpactForces += collisionImpactForce;

                // reposition particle so that it collides with the ground rather than through it
                Vector3 newPosition = Vector3.zero;
                newPosition.z = transform.position.z;               // same as before
                newPosition.y = GROUND_Y_POSITION + Radius;         // Y position, plus radius of our particle

                // calculating our X position is slightly more difficult.
                // we need to calculate the percentage of the timestep moved in the Y and use that to calculate our new X
                float percentageOfTimestepMoved = (newPosition.y - PreviousPosition.y) / (newPosition.y - transform.position.y);

                float calculatedX = (transform.position.y - PreviousPosition.y) * percentageOfTimestepMoved;
                newPosition.x = calculatedX;

                GizmoDrawOptions newPosDrawOptions = new GizmoDrawOptions();
                newPosDrawOptions.gizmoType = GizmoType.Sphere;
                newPosDrawOptions.position = newPosition;
                newPosDrawOptions.color = Color.yellow;
                newPosDrawOptions.time = -1;
                AddGizmoToDraw(newPosDrawOptions);

                transform.position = newPosition;

                return true;
            }
        }

        return false;
    }

    // Aggregate forces acting on this particle
    public void CalcLoads()
    {
        // Reset forces
        Forces = Vector3.zero;

        // Aggregate Forces

        // Collision
        if(HasCollided)
        {
            AddForceInternal(ImpactForces);
            return; // early return on frames when we collide
        }

        // Gravity
        GizmoDrawOptions GravityDrawOptions = new GizmoDrawOptions();
        GravityDrawOptions.color = Color.yellow;
        GravityDrawOptions.gizmoType = GizmoType.Line;
        GravityDrawOptions.forceLine = GRAVITY;
        AddForceInternal(GRAVITY, GravityDrawOptions);

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
        GizmoDrawOptions AirDragDrawOptions = new GizmoDrawOptions();
        AirDragDrawOptions.color = Color.blue;
        AirDragDrawOptions.gizmoType = GizmoType.Line;
        AirDragDrawOptions.forceLine = AirDragForce;
        AddForceInternal(AirDragForce, AirDragDrawOptions);
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
        GizmoDrawOptions WindDrawOptions = new GizmoDrawOptions();
        WindDrawOptions.color = Color.red;
        WindDrawOptions.gizmoType = GizmoType.Line;
        WindDrawOptions.forceLine = WindForce;
        AddForceInternal(WindForce, WindDrawOptions);
#endregion

    }

    private void AddForceInternal(Vector3 force, GizmoDrawOptions drawOptions = null)
    {
        if(DrawDebugGizmos && drawOptions != null)
        {
            GizmosQueue.Add(drawOptions);
        }
        Forces += force;
    }

    private void AddGizmoToDraw(GizmoDrawOptions drawOptions)
    {
        if(DrawDebugGizmos)
        {
            GizmosQueue.Add(drawOptions);
        }
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
            GizmosQueue.ForEach(drawOption =>
            {
                Gizmos.color = drawOption.color;
                if(drawOption.gizmoType == GizmoType.Line)
                {
                    Gizmos.DrawLine(transform.position, transform.position + drawOption.forceLine);
                }
                else if(drawOption.gizmoType == GizmoType.Sphere)
                {
                    Gizmos.DrawWireSphere(drawOption.position, drawOption.radius);
                }
            });
        }
    }
}
