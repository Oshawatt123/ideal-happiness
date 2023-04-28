using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PhysicsObject : MonoBehaviour
{
    List<Collider> Colliders = new List<Collider>();

    public bool UseGravity = false;

    public Vector3 velocity = Vector3.zero;
    Vector3 Acceleration = Vector3.zero;

    // Start is called before the first frame update
    void Start()
    {
        GameObject Manager = GameObject.FindWithTag("PhysManager");
        Manager.GetComponent<PhysicsManager>().Register(this);

        Colliders = GetComponents<Collider>().ToList();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public Vector3 GetVelocity() { return velocity; }
    public void AddVelocity(Vector3 addVelocity) { velocity += addVelocity; }

    public Vector3 GetAcceleration() { return Acceleration; }
    public void SetAcceleration(Vector3 newAcceleration) { Acceleration = newAcceleration; }
}
