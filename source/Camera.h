#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"
#include <iostream>

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}
		{
			CalculateFov();
		}

		const float moveSpeed{ 5.f };
		const float lookSpeed{ 0.25f };

		Vector3 origin{};
		float fovAngle{90.f};
		float fov{};

		// Vector3 forward{0.266f, -0.453f, 0.860f};
		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};

		Matrix CalculateCameraToWorld()
		{
			right = Vector3::Cross(Vector3::UnitY, forward).Normalized();
			up = Vector3::Cross(forward, right).Normalized();
			return {right, up, forward, origin};
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();
			const float constMoveSpeed = moveSpeed * deltaTime;
			const float constLookSpeed = lookSpeed * deltaTime;

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
			
			origin += (pKeyboardState[SDL_SCANCODE_D] - pKeyboardState[SDL_SCANCODE_A]) * constMoveSpeed * right;
			origin += (pKeyboardState[SDL_SCANCODE_W] - pKeyboardState[SDL_SCANCODE_S]) * constMoveSpeed * forward;

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);
			if (mouseState & SDL_BUTTON_LMASK)
			{
				if (mouseState & SDL_BUTTON_RMASK)
					origin.y += mouseY * constMoveSpeed;
				else
				{
					origin.z += mouseY * constMoveSpeed;
					totalYaw += mouseX * constLookSpeed;
				}
			}

			if (mouseState & SDL_BUTTON_RMASK && !(mouseState & SDL_BUTTON_LMASK))
			{
				totalPitch += -mouseY * constLookSpeed;
				totalYaw += mouseX * constLookSpeed;
			}
			
			if (mouseState & SDL_BUTTON_LMASK || mouseState & SDL_BUTTON_RMASK)
			{
				const Matrix pitch{ Matrix::CreateRotationX(totalPitch) };
				const Matrix yaw{ Matrix::CreateRotationY(totalYaw) };
				const Matrix finalRotation{ pitch * yaw };
				forward = finalRotation.TransformVector(Vector3::UnitZ);
				forward.Normalize();
			}
		}

		float CalculateFov()
		{
			return fov = tanf(fovAngle / 2.f);
		}
	};
}