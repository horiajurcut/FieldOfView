﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Controller : MonoBehaviour {

    public float moveSpeed = 6f;

    Rigidbody myRigidbody;
    Camera viewCamera;
    Vector3 velocity;

	private void Start ()
    {
        myRigidbody = GetComponent<Rigidbody>();
        viewCamera = Camera.main;
	}
	
	private void Update ()
    {
        Vector3 mousePosition = viewCamera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, viewCamera.transform.position.y));
        transform.LookAt(mousePosition + Vector3.up * transform.position.y);
        velocity = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical")).normalized * moveSpeed;

    }

    private void FixedUpdate()
    {
        myRigidbody.MovePosition(myRigidbody.position + velocity * Time.fixedDeltaTime);
    }
}
