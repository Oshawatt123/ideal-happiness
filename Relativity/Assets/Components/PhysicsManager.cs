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
        foreach (PhysicsObject Object in PhysicsObjects)
        {
            Vector3 ObjectAcceleration = Vector3.zero;

            // apply gravity
            if(Object.UseGravity)
            {
                ObjectAcceleration += Gravity;
            }
            // other external forces like collision etc. would go here

            foreach (PhysicsObject Object2 in PhysicsObjects)
            {
                // early out if matching. don't want to collide with ourselves
                if (Object == Object2)
                {
                    continue;
                }

                // sphere on sphere collision
                float distance = Vector3.Distance(Object.transform.position, Object2.transform.position);
                if (distance < Object.GetComponent<SphereCollider>().radius)
                {
                    // get collision normal
                    Vector3 collisionNormal = Object.transform.position - Object2.transform.position;
                    ObjectAcceleration += collisionNormal* CollisionForce;
                }

            }

            // end collision

            // integrate for velocity
            Vector3 diffVelocity = ObjectAcceleration * Time.deltaTime;
            Object.AddVelocity(diffVelocity);
            // end velocity

            // integrate for position
            Object.transform.position += (Object.GetVelocity() * Time.deltaTime);
            // end position
        }
    }


    public void Register(PhysicsObject physicsObject)
    {
        PhysicsObjects.Add(physicsObject);
    }
}
