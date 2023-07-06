using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class PhysicsManager : MonoBehaviour
{

    List<PhysicsObject> PhysicsObjects = new List<PhysicsObject>();

    public Vector3 Gravity = new Vector3(0, -9.8f, 0);

    public float CollisionForce = 5;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(Time.deltaTime);
        foreach (PhysicsObject obj in PhysicsObjects)
        {
            // calculate net force this frame
            obj.SetForce(Vector3.zero);

            // apply impulse forces
            foreach (Vector3 force in obj.GetImpulseForces())
            {
                obj.ApplyForce(force); // THESE ARE FORCES WE ARE APPLYING, NOT ACCELERATIONS. F = MA. THEREFORE NO MASS NEEDED HERE
            }

            obj.ApplyForce(obj.GetMass() * Gravity); // apply gravity every frame // GRAVITY IS AN ACCELERATION THEREFORE WE TIMES BY MASS. F = MA

            // -------------- INTEGRATION -------------- //

            // semi-implicit euler integration
            // this calculates new velocity *before* updating position
            // this is generally more stable at a great range of dt values than explicit euler integration
            // which performs this the other way around

            // integrate velocity
            obj.SetVelocity(obj.GetVelocity() + obj.GetForce() * Time.deltaTime);

            // integrate position
            obj.SetPosition(obj.GetPosition() + obj.GetVelocity() * Time.deltaTime);
        }
    }


    public void Register(PhysicsObject physicsObject)
    {
        PhysicsObjects.Add(physicsObject);
    }

    private void OnDrawGizmos()
    {

    }
}
