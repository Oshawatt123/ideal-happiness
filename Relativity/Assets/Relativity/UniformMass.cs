using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UniformMass : MonoBehaviour
{

    public float Mass;
    // all uniform masses will be cuboids for now
    public Vector3 Dimensions;
    public Color drawColor = Color.black;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public virtual Vector3 GetCenterOfMass()
    {
        // in the case of base UniformMass, center of mass would be just the position of the object as mass distribution is, well, *uniform*
        return transform.position;
    }

    // currently gets moment of inertia about the Y(up) axis
    // since we are only dealing with 2D physics rn
    public virtual Vector3 GetMomentsOfInertia()
    {
        // moment of inertia for a cuboid about the y axis
        //Iyy = (m/12)*(Dimension.x^2 + Dimension.z^2)

        // "neutral" being out cartesian co-ordinate axes X, Z and Y
        Vector3 NeutralMomentsOfInertia = Vector3.zero;

        // get moment of inertia about the y axis
        NeutralMomentsOfInertia.y = (Mass / 12) * (Mathf.Pow(Dimensions.x, 2) + Mathf.Pow(Dimensions.z, 2));


        return NeutralMomentsOfInertia;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(GetCenterOfMass(), 1f);
    }

    public void DrawGizmo()
    {
        Gizmos.color = drawColor;
        Gizmos.DrawCube(transform.position, Dimensions);

        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(transform.position, Dimensions);
    }
}
