using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// A Body is a collection of masses. It derives from uniform mass but is itself *not* uniform
public class Body : MonoBehaviour
{
    [Header("Bodies")]
    public List<UniformMass> BodyMasses = new List<UniformMass>();
    private Vector3 CenterOfMass = Vector3.zero;

    [Header("Rotation")]
    private Vector3 NeutralMomentsOfInertia = Vector3.zero;

    [Header("Debug")]
    [SerializeField] private bool ShowCmAlways = false;
    [SerializeField] private bool UpdatePrivateFieldsInEditTime = false;
    // Start is called before the first frame update
    void Start()
    {
        NeutralMomentsOfInertia = GetMomentsOfInertia();
        CenterOfMass = GetCenterOfMass();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public virtual Vector3 GetCenterOfMass()
    {
        if(BodyMasses.Count == 0)
        {
            return Vector3.zero;
        }

        Vector3 CenterOfMass = Vector3.zero;
        foreach(UniformMass uniformMass in BodyMasses)
        {
            CenterOfMass += uniformMass.GetCenterOfMass() * uniformMass.Mass;
        }
        CenterOfMass /= TotalBodyMass();

        return CenterOfMass;
    }

    // only calculates moment of inertia about the neutral Y(up) axis for now as we are dealing with 2D space
    public virtual Vector3 GetMomentsOfInertia()
    {
        Vector3 neutralMomentsOfInertia = Vector3.zero;

        List<Vector3> TranslatedBodyMassesMomentsOfInertia = new List<Vector3>();
        foreach(UniformMass uniformMass in BodyMasses)
        {
            // get mass moment of inertia
            Vector3 BodyMassMomentsOfInertia = uniformMass.GetMomentsOfInertia();

            // translate to body axis by parallel axis theorom
            // Ia = Ib + md^2
            float dx = 0, dz = 0;
            dx = transform.position.x - uniformMass.transform.position.x;
            dz = transform.position.z - uniformMass.transform.position.z;

            // pythagorean theorom
            float YAxisDistanceSquared = Mathf.Pow(dx, 2) + Mathf.Pow(dz, 2);
            float TranslatedYAxisBodyMassMomentsOfInertia = BodyMassMomentsOfInertia.y + (uniformMass.Mass * YAxisDistanceSquared);

            Vector3 TranslatedBodyMassMomentsOfInertia = Vector3.zero;
            TranslatedBodyMassMomentsOfInertia.y = TranslatedYAxisBodyMassMomentsOfInertia;
            TranslatedBodyMassesMomentsOfInertia.Add(TranslatedBodyMassMomentsOfInertia);
        }

        foreach(Vector3 TranslatedBodyMassMomentsOfInertia in TranslatedBodyMassesMomentsOfInertia)
        {
            neutralMomentsOfInertia += TranslatedBodyMassMomentsOfInertia;
        }

        return neutralMomentsOfInertia;
    }

    private float TotalBodyMass()
    {
        float mass = 0;
        foreach (UniformMass uniformMass in BodyMasses)
        {
            mass += uniformMass.Mass;
        }

        return mass;
    }

    private void OnDrawGizmos()
    {
        // draw box surrounding extents of all bound masses??
        //Gizmos.color = Color.black;
        //Gizmos.DrawCube(transform.position, Dimensions);

        var sorted = BodyMasses.OrderByDescending((x) => Vector3.Distance(Camera.current.transform.position, x.transform.position));

        foreach(UniformMass uniformMass in sorted)
        {
            uniformMass.DrawGizmo();
        }

        if (ShowCmAlways)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(GetCenterOfMass(), 1f);
        }
        if(UpdatePrivateFieldsInEditTime)
        {
            NeutralMomentsOfInertia = GetMomentsOfInertia();
        }
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(GetCenterOfMass(), 1f);
    }
}
