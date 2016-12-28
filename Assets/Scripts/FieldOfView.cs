using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FieldOfView : MonoBehaviour {

    public bool DrawGizmos = false;

    public float viewRadius;

    [Range(0, 360)]
    public float viewAngle;

    public LayerMask targetMask;
    public LayerMask obstacleMask;

    public float meshResolution;
    public MeshFilter viewMeshFilter;

    public int edgeResolveIterations;
    public float edgeDistanceThreshold;

    public float maskCutawayDistance = .1f;

    [HideInInspector]
    public List<Transform> visibleTargets = new List<Transform>();

    Mesh viewMesh;

    private void Start()
    {
        viewMesh = new Mesh();
        viewMesh.name = "View Mesh";
        viewMeshFilter.mesh = viewMesh;

        StartCoroutine("FindTargetsWithDelay", .2f);
    }

    private void LateUpdate()
    {
        DrawFieldOfView();
    }

    private IEnumerator FindTargetsWithDelay(float delay)
    {
        while (true)
        {
            yield return new WaitForSeconds(delay);
            FindVisibleTargets();
        }
    }

    private void FindVisibleTargets()
    {
        visibleTargets.Clear();
        Collider[] targetsInViewRadius = Physics.OverlapSphere(transform.position, viewRadius, targetMask);

        for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
            Transform target = targetsInViewRadius[i].transform;
            Vector3 directionToTarget = (target.position - transform.position).normalized;

            if (Vector3.Angle(transform.forward, directionToTarget) < viewAngle / 2)
            {
                float distanceToTarget = Vector3.Distance(transform.position, target.position);

                if (!Physics.Raycast(transform.position, directionToTarget, distanceToTarget, obstacleMask))
                {
                    // No Obstacles
                    visibleTargets.Add(target);
                }
            }
        }
    }

    private void DrawFieldOfView()
    {
        int rayCount = Mathf.RoundToInt(viewAngle * meshResolution);
        float rayAngleSize = viewAngle / rayCount;

        List<Vector3> viewPoints = new List<Vector3>();
        ViewRaycastInformation oldViewRaycast = new ViewRaycastInformation();
        for (int i = 0; i <= rayCount; i++)
        {
            float currentAngle = transform.eulerAngles.y - viewAngle / 2 + rayAngleSize * i;

            ViewRaycastInformation newViewRaycast = ViewRaycast(currentAngle);

            if (i > 0)
            {
                bool edgeDistanceThresholdExceeded = Mathf.Abs(oldViewRaycast.distance - newViewRaycast.distance) > edgeDistanceThreshold;
                if (oldViewRaycast.hit != newViewRaycast.hit || (oldViewRaycast.hit && newViewRaycast.hit && edgeDistanceThresholdExceeded))
                {
                    // Find Edge
                    EdgeInformation edge = FindEdge(oldViewRaycast, newViewRaycast);

                    if (edge.pointA != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointA);
                    }
                    if (edge.pointB != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointB);
                    }
                }
            }

            viewPoints.Add(newViewRaycast.point);
            oldViewRaycast = newViewRaycast;

            // Draw Mesh Lines
            if (DrawGizmos)
            {
                Debug.DrawLine(transform.position, transform.position + DirectionFromAngle(currentAngle, true) * viewRadius, Color.blue);
            }
        }

        int vertexCount = viewPoints.Count + 1;
        Vector3[] vertices = new Vector3[vertexCount];
        int[] triangles = new int[(vertexCount - 2) * 3];

        vertices[0] = Vector3.zero;
        for (int i = 0; i < vertexCount - 1; i++)
        {
            vertices[i + 1] = transform.InverseTransformPoint(viewPoints[i]) + Vector3.forward * maskCutawayDistance;

            if (i < vertexCount - 2)
            {
                triangles[i * 3] = 0;
                triangles[i * 3 + 1] = i + 1;
                triangles[i * 3 + 2] = i + 2;
            }  
        }

        viewMesh.Clear();
        viewMesh.vertices = vertices;
        viewMesh.triangles = triangles;
        viewMesh.RecalculateNormals();
    }

    private EdgeInformation FindEdge(ViewRaycastInformation minViewRaycast, ViewRaycastInformation maxViewRaycast)
    {
        float minAngle = minViewRaycast.angle;
        float maxAngle = maxViewRaycast.angle;

        Vector3 minPoint = Vector3.zero;
        Vector3 maxPoint = Vector3.zero;

        for (int i = 0; i < edgeResolveIterations; i++)
        {
            float angle = (minAngle + maxAngle) / 2;
            ViewRaycastInformation newViewRaycast = ViewRaycast(angle);

            bool edgeDistanceThresholdExceeded = Mathf.Abs(minViewRaycast.distance - newViewRaycast.distance) > edgeDistanceThreshold;
            if (newViewRaycast.hit == minViewRaycast.hit && !edgeDistanceThresholdExceeded)
            {
                minAngle = angle;
                minPoint = newViewRaycast.point;
            }
            else
            {
                maxAngle = angle;
                maxPoint = newViewRaycast.point;
            }
        }

        return new EdgeInformation(minPoint, maxPoint);
    }

    private ViewRaycastInformation ViewRaycast(float globalAngle)
    {
        Vector3 direction = DirectionFromAngle(globalAngle, true);
        RaycastHit hit;

        if (Physics.Raycast(transform.position, direction, out hit, viewRadius, obstacleMask))
        {
            return new ViewRaycastInformation(true, hit.point, hit.distance, globalAngle);
        }

        return new ViewRaycastInformation(false, transform.position + direction * viewRadius, viewRadius, globalAngle);
    }

    public Vector3 DirectionFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
        if (!angleIsGlobal)
        {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad), 0, Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    public struct ViewRaycastInformation
    {
        public bool hit;
        public Vector3 point;
        public float distance;
        public float angle;

        public ViewRaycastInformation(bool _hit, Vector3 _point, float _distance, float _angle)
        {
            hit = _hit;
            point = _point;
            distance = _distance;
            angle = _angle;
        }
    }

    public struct EdgeInformation
    {
        public Vector3 pointA;
        public Vector3 pointB;

        public EdgeInformation(Vector3 _pointA, Vector3 _pointB)
        {
            pointA = _pointA;
            pointB = _pointB;
        }
    }
}
