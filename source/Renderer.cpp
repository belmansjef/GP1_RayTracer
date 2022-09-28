//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"
#include <iostream>

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
	m_AspectRatio = static_cast<float>(m_Width) / static_cast<float>(m_Height);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();
	const Matrix cameraToWorld = camera.CalculateCameraToWorld();

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			const float cx{ ((2.0f * (px + 0.5f) / static_cast<float>(m_Width)) - 1) * m_AspectRatio * camera.fov };
			const float cy{ (1 - (2 * (py + 0.5f) / static_cast<float>(m_Height))) * camera.fov};

			Vector3 rayDirection = { cx, cy, 1.f };
			// rayDirection.Normalize(); // Decreases performance during W2 with about 5 FPS and has no visual impact
			rayDirection = cameraToWorld.TransformVector(rayDirection).Normalized();

			Ray viewRay{ camera.origin, rayDirection };
			ColorRGB finalColor{};

			HitRecord closestHit{};
			pScene->GetClosestHit(viewRay, closestHit);

			if (closestHit.didHit)
			{
				finalColor = materials[closestHit.materialIndex]->Shade();

				for (const Light& light : lights)
				{
					Ray hitTowardsLightRay{};
					hitTowardsLightRay.origin = closestHit.origin + closestHit.normal * 0.01f;
					hitTowardsLightRay.direction = LightUtils::GetDirectionToLight(light, hitTowardsLightRay.origin);
					hitTowardsLightRay.min = 0.0001f;
					hitTowardsLightRay.max = hitTowardsLightRay.direction.Magnitude();
					hitTowardsLightRay.direction.Normalize();

					if (pScene->DoesHit(hitTowardsLightRay))
					{
						finalColor *= 0.5f;
					}
				}
			}

			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}
