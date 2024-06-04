using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Car : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform suspensionFL;
    [SerializeField] private Transform suspensionFR;
    [SerializeField] private Transform suspensionRL;
    [SerializeField] private Transform suspensionRR;
    [SerializeField] private GameObject wheelPrefab;
    
    [Header("Settings")]
    [SerializeField] private float suspensionHeight = 0.5f;
    [SerializeField] private float suspensionSpringStrength = 100f;
    [SerializeField] private float suspensionDamping = 10f;
    [SerializeField] private LayerMask layerMask;

    private Rigidbody rb;
    private Transform wheelFL;
    private Transform wheelFR;
    private Transform wheelRL;
    private Transform wheelRR;
    private Bounds wheelBounds;
    private Vector2 steeringInput;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Rigidbody not found on car object");
            enabled = false;
        }
        
        wheelFL = SpawnWheel(suspensionFL);
        wheelFR = SpawnWheel(suspensionFR);
        wheelRL = SpawnWheel(suspensionRL);
        wheelRR = SpawnWheel(suspensionRR);

        wheelBounds = wheelPrefab.GetComponentInChildren<MeshRenderer>().bounds;
    }

    private void FixedUpdate()
    {
        UpdateSuspension(suspensionFL);
        UpdateSuspension(suspensionFR);
        UpdateSuspension(suspensionRL);
        UpdateSuspension(suspensionRR);
    }

    public void Steering(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            steeringInput = context.ReadValue<Vector2>();
        }
        else if (context.canceled)
        {
            steeringInput = Vector2.zero;
        }
    }
    
    private void UpdateSuspension(Transform suspension)
    {
        if (Physics.Raycast(suspension.position, -suspension.up, out var hit, suspensionHeight, layerMask))
        {
            var springDir = suspension.up;
            var tireWorldVel = rb.GetPointVelocity(suspension.position);
            var offset = suspensionHeight - hit.distance;
            var vel = Vector3.Dot(springDir, tireWorldVel);
            var force = (offset * suspensionSpringStrength) - (vel * suspensionDamping);
            
            rb.AddForceAtPosition(springDir * force, suspension.position);

            var wheel = suspension.GetChild(0);
            var wheelPosition = hit.point + suspension.up * wheelBounds.extents.y;
            if (Vector3.Distance(wheelPosition, suspension.position) > suspensionHeight)
            {
                wheelPosition = suspension.position - suspension.up * suspensionHeight + suspension.up * wheelBounds.extents.y;
            }
            wheel.position = wheelPosition;
        }
        else
        {
            var wheel = suspension.GetChild(0);
            var wheelPosition = suspension.position - suspension.up * suspensionHeight + suspension.up * wheelBounds.extents.y;
            wheel.position = wheelPosition;
        }
    }

    private Transform SpawnWheel(Transform suspension)
    {
        var wheel = Instantiate(wheelPrefab, suspension.position, suspension.rotation);
        wheel.transform.parent = suspension;
        return wheel.transform;
    }
    
    private void DrawGizmoWheelSpeed(Transform suspension)
    {
        Gizmos.color = Color.red;
        Gizmos.DrawRay(suspension.position, suspension.right * rb.GetPointVelocity(suspension.position).x);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(suspension.position, suspension.up * rb.GetPointVelocity(suspension.position).y);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(suspension.position, suspension.forward * rb.GetPointVelocity(suspension.position).z);
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
            return;
        
        DrawGizmoWheelSpeed(suspensionFL);
        DrawGizmoWheelSpeed(suspensionFR);
        DrawGizmoWheelSpeed(suspensionRL);
        DrawGizmoWheelSpeed(suspensionRR);
    }
}
