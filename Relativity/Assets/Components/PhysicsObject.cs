using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PhysicsObject : MonoBehaviour
{

    //Vector3 Position; <- handled by Transform.Position
    Vector3 Velocity;
    Vector3 Force;
    [SerializeField]
    float Mass = 1;

    List<Vector3> ImpulseForces = new List<Vector3>();

    // Start is called before the first frame update
    void Start()
    {
        GameObject Manager = GameObject.FindWithTag("PhysManager");
        Manager.GetComponent<PhysicsManager>().Register(this);

        //Force += new Vector3(Random.Range(5f, 10f), 49f, 0f); // add random force

        Velocity = new Vector3(Random.Range(15f, 20f), 20f, 0f);

        //Colliders = GetComponents<Collider>().ToList();
    }

    private void Update()
    {
        if(Input.GetKeyDown(KeyCode.H))
        {
            ApplyImpulseForce(new Vector3(-17f, 0f, 0f));
        }
    }

    public Vector3 GetPosition() { return transform.position; }
    public void SetPosition(Vector3 position) { transform.position = position; }

    public Vector3 GetVelocity() { return Velocity; }
    public void SetVelocity(Vector3 velocity) { Velocity = velocity; }

    public Vector3 GetForce() { return Force; }
    public void SetForce(Vector3 force) { Force = force; }
    public void ApplyForce(Vector3 force) { Force += force; }

    public float GetMass() { return Mass; }

    public List<Vector3> GetImpulseForces() {  return ImpulseForces; }
    public void ApplyImpulseForce(Vector3 force) {  ImpulseForces.Add(force); }
}
