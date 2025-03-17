using UnityEngine;
using System.Collections;
using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision
	float friction = 0.3f;
	Vector3 G = new Vector3(0.0f, -9.8f, 0.0f);


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	private Quaternion Add(Quaternion a, Quaternion b)
	{
		Quaternion result = new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
		return result;
	}

	private Matrix4x4 Matrix_sub(Matrix4x4 a, Matrix4x4 b)
	{
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                a[i, j] -= b[i, j];
            }
        }
        return a;
    }

	private Matrix4x4 Matrix_multiply(Matrix4x4 a, float b) 
	{
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                a[i, j] *= b;
            }
        }
        return a;
    }


	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		// get mesh vertex
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Vector3 x = transform.position; //当前质心

		Vector3 Rri_sum = new Vector3(0, 0, 0);
		Vector3 vi_sum = new Vector3(0, 0, 0);
		int collision_num = 0;

		for(int i = 0; i < vertices.Length; i++)
		{
			Vector3 ri = vertices[i];
			Vector3 Rri = R.MultiplyVector(ri);
			Vector3 xi = x + Rri; // xi = x + Rri

			if(Vector3.Dot(xi - P, N) < 0.0f)
			{
				Vector3 vi = v + Vector3.Cross(w, Rri);
				if(Vector3.Dot(vi, N)<0.0f)
				{
					// to compute average of all colliding vertices
					Rri_sum += Rri;
					vi_sum += vi;
					collision_num++;
				}
			}
		}

		if(collision_num == 0) return;
		Vector3 Rr_average = Rri_sum / collision_num;
		Vector3 v_average = vi_sum / collision_num;

		// v -> v_N, v_T
		Vector3 v_N = Vector3.Dot(v_average, N) * N;
		Vector3 v_T = v_average - v_N;

		// v_new
		float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0.0f);
		Vector3 v_N_new = -restitution * v_N;
		Vector3 v_T_new = a * v_T;
		Vector3 v_new = v_N_new + v_T_new;

		//update impulse j
		Matrix4x4 K = Matrix_sub(Matrix_multiply(Matrix4x4.identity, 1.0f/mass), Get_Cross_Matrix(Rr_average) * I_ref.inverse * Get_Cross_Matrix(Rr_average));
		Vector3 J = K.inverse.MultiplyVector(v_new - v_average);

		// update v, w
		v += 1.0f/mass * J;
		w += I_ref.inverse.MultiplyVector(Vector3.Cross(Rr_average, J));

		// 防止落地后一直抖动
		restitution *= 0.8f;
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		if(launched)
		{
			// Part I: Update velocities
			if(launched == false) return;
			v += dt * G;
			v *= linear_decay;
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x = transform.position;
			//Update angular status
			Quaternion q = transform.rotation;

			x += v * dt;
			Vector3 dw = 0.5f * dt * w;
			q = Add(q, new Quaternion(dw.x, dw.y, dw.z, 0.0f) * q);

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}
	}
}
