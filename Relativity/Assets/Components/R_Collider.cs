using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class R_Collider : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public virtual bool CollisionRule(Vector3 CollidingPosition)
    {
        return false;
    }
}
