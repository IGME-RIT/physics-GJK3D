/*
Title: GJK-3D (OBB)
File Name: GameObject.cpp
Copyright © 2015
Original authors: Brockton Roth
Written under the supervision of David I. Schwartz, Ph.D., and
supported by a professional development seed grant from the B. Thomas
Golisano College of Computing & Information Sciences
(https://www.rit.edu/gccis) at the Rochester Institute of Technology.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Description:
This is a Gilbert-Johnson-Keerthi test. (Called GJK for short.) This is in 3D.
Contains two cubes, one that is stationary and one that is moving. They are bounded
by OBBs (Object-Oriented Bounding Boxes) and when these OBBs collide the moving object
"bounces" on the x axis (because that is the only direction the object is moving).
The algorithm will detect any axis of collision, but will not output the axis that was
collided (because it doesn't know). Thus, we assume x and hardcode in the x axis bounce.
There is a physics timestep such that every update runs at the same delta time, regardless
of how fast or slow the computer is running. The cubes would be the exact same as their OBBs,
since they are aligned on the same axis.
*/

#ifndef _GAME_OBJECT_CPP
#define _GAME_OBJECT_CPP

#include "GameObject.h"

// Note that the model does not actually get copied, but instead we just save a pointer to it.
// So make sure that model is stored and cleaned up elsewhere!
GameObject::GameObject(Model* inModel)
{
	model = inModel;

	// Initialize identity matrices.
	translation = glm::mat4();
	rotation = glm::mat4();
	scale = glm::mat4();
	transformation = glm::mat4();

	// And default vectors.
	position = glm::vec3();
	velocity = glm::vec3();
	acceleration = glm::vec3();

	// And a default quaternion.
	quaternion = glm::quat();
}

void GameObject::Update(float dt)
{
	// Do basic physics calcuations based on dt.
	velocity += acceleration * dt;
	position += velocity * dt;

	// Set the translation equal to the new position of the object.
	SetTranslation(position); // Note that this will also recalculate the transformation matrix.
}

// Calculates the transformation matrix based on translation, then rotation, then scale.
void GameObject::CalculateMatrices()
{
	transformation = translation * rotation * scale;
}

// Adds the incoming vec3 pos to the position, and then translates the object to that position.
void GameObject::AddPosition(glm::vec3 pos)
{
	position += pos;

	Translate(pos);
}

// Adds the incoming vec3 vel to the velocity.
void GameObject::AddVelocity(glm::vec3 vel)
{
	velocity += vel;
}

// Adds the incoming vec3 accel to the acceleration.
void GameObject::AddAcceleration(glm::vec3 accel)
{
	acceleration += accel;
}

// Scales the current scale value by the x, y and z values given. (So if the scale is [0.5, 0.5, 0.5] and we pass in [0.5, 0.5, 0.5] we end up with [0.25, 0.25, 0.25].)
void GameObject::Scale(glm::vec3 scaleFactor)
{
	// Scales the scale matrix.
	scale = glm::scale(scale, scaleFactor);

	// Then we have to recalculate the transformation matrix.
	CalculateMatrices();
}

// Sets the scale in the x, y, and z position to the given values.
void GameObject::SetScale(glm::vec3 scaleFactor)
{
	// Scales the identity matrix.
	scale = glm::scale(glm::mat4(), scaleFactor);

	// Then we have to recalculate the transformation matrix.
	CalculateMatrices();
}

// Rotates in x, y, and z radians based on given values.
void GameObject::Rotate(glm::vec3 rotFactor)
{
	// WARNING: These are interpreted as radian values, so be sure to specify them not as degrees.
	
	// Create a quaternion based on the euler angles given.
	glm::quat q = glm::quat(rotFactor);

	// Rotate our quaternion by that quaternion's value.
	quaternion *= q;

	// Turn our quaternion into a mat4.
	rotation = glm::toMat4(quaternion);

	// Then we have to recalculate the transformation matrix.
	CalculateMatrices();
}

// Sets the rotation matrix to a given value.
void GameObject::SetRotation(glm::mat4* rotMatrix)
{
	rotation = *rotMatrix;

	// Then we have to recalculate the transformation matrix.
	CalculateMatrices();
}

// Sets the rotation matrix to a given value of x, y, and z radians.
void GameObject::SetRotation(glm::vec3 rotFactor)
{
	// WARNING: These are interpreted as radian values, so be sure to specify them not as degrees.

	// Set our quaternion equal to a quaternion created from the given euler angles.
	quaternion = glm::quat(rotFactor);

	// Turn our quaternion into a mat4.
	rotation = glm::toMat4(quaternion);

	// Then we have to recalculate the transformation matrix.
	CalculateMatrices();
}

// Translates in the x, y, and z directions based on the given values.
void GameObject::Translate(glm::vec3 transFactor)
{
	// Translates the translation matrix.
	translation = glm::translate(translation, transFactor);

	// Then we have to recalculate the transformation matrix.
	CalculateMatrices();
}

// Sets the translations to the exact x, y, and z position values given.
void GameObject::SetTranslation(glm::vec3 transFactor)
{
	// Translates the identity matrix.
	translation = glm::translate(glm::mat4(), transFactor);

	// Then we have to recalculate the transformation matrix.
	CalculateMatrices();
}

#endif // _GAME_OBJECT_CPP